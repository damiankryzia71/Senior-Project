#include <opencv2/core/core.hpp>
#include <gst/gst.h>

#include "System.h"

#include <iostream>
#include <vector>
#include <thread>
#include <string>
#include <atomic>
#include <chrono>
#include <mutex>
#include <queue>
#include <cmath>
#include <functional>

struct ImageStamped {
    double timestamp;
    cv::Mat image;
};

struct PointCloudStamped {
    double timestamp;
    cv::Mat pointCloud;
};

bool LoadPointCloudGst(cv::Mat &pcd, GstElement *appsink);

std::atomic<bool> stopSLAM(false);
std::mutex img_mutex, lidar_mutex;
std::queue<ImageStamped> image_queue;
std::queue<PointCloudStamped> lidar_queue;

void listenForStopCommand() {
    std::string input;
    while (!stopSLAM) {
        std::cin >> input;
        if (input == "stop") {
            stopSLAM = true;
            std::cout << "Stopping ORB-SLAM3..." << std::endl;
        }
    }
}

void getImagesStamped(cv::VideoCapture &cap) {
    while (!stopSLAM) {
        cv::Mat img;
        cap.read(img);
        double ts = std::chrono::duration<double>(
                        std::chrono::steady_clock::now().time_since_epoch()
                    ).count();

        if (!img.empty()) {
            std::lock_guard<std::mutex> lock(img_mutex);
            image_queue.push({ts, img});
            if (image_queue.size() > 100) image_queue.pop();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void getPointCloudDataStamped(GstElement *appsink) {
    while (!stopSLAM) {
        cv::Mat pcd;
        if (LoadPointCloudGst(pcd, appsink)) {
            double ts = std::chrono::duration<double>(
                            std::chrono::steady_clock::now().time_since_epoch()
                        ).count();

            std::lock_guard<std::mutex> lock(lidar_mutex);
            lidar_queue.push({ts, pcd});
            if (lidar_queue.size() > 100) lidar_queue.pop();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

int main(int argc, char** argv) {
    gst_init(&argc, &argv);

    if (argc < 5) {
        std::cerr << "Usage: " << argv[0] << " path_to_vocabulary path_to_settings camera_port lidar_port" << std::endl;
        return 1;
    }

    std::string path_to_vocabulary = argv[1];
    std::string path_to_settings = argv[2];
    std::string camera_port = argv[3];
    std::string lidar_port = argv[4];

    std::string camera_pipeline_desc = 
    "udpsrc port=" + camera_port + " ! "
    "application/x-rtp, media=video, encoding-name=H264, payload=96 ! "
    "rtph264depay ! avdec_h264 ! videoconvert ! appsink";

    std::string lidar_pipeline_desc = 
    "udpsrc port=" + lidar_port + " ! "
    "appsink name=mysink";

    cv::VideoCapture cap(camera_pipeline_desc, cv::CAP_GSTREAMER);

    if (!cap.isOpened())
    {
        std::cerr << "Failed to open video stream" << std::endl;
        return 1;
    }

    GError *error = nullptr;
    GstElement *pipeline = gst_parse_launch(lidar_pipeline_desc.c_str(), &error);

    if (!pipeline)
    {
        std::cerr << "Failed to create pipeline: " << error->message << std::endl;
        g_error_free(error);
        return 1;
    }

    GstElement *appsink = gst_bin_get_by_name(GST_BIN(pipeline), "mysink");

    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    std::thread stopListener(listenForStopCommand);
    
    ORB_SLAM3::System SLAM(path_to_vocabulary, path_to_settings, ORB_SLAM3::System::RGBL, true);

    std::thread cameraThread(getImagesStamped, std::ref(cap));
    std::thread lidarThread(getPointCloudDataStamped, appsink);
    
    const double max_dt = 0.03;
    ImageStamped img;
    PointCloudStamped pcd;

    while (!stopSLAM) {
        {
            std::lock_guard<std::mutex> lock1(img_mutex);
            std::lock_guard<std::mutex> lock2(lidar_mutex);
    
            if (image_queue.empty() || lidar_queue.empty())
                continue;
    
            img = image_queue.front();
            pcd = lidar_queue.front();
    
            double dt = std::abs(img.timestamp - pcd.timestamp);
            if (dt < max_dt) {
                image_queue.pop();
                lidar_queue.pop();
            } 
            else if (img.timestamp < pcd.timestamp)
            {
                image_queue.pop();
                continue;
            } 
            else 
            {
                lidar_queue.pop();
                continue;
            }
        }

        SLAM.TrackRGBL(img.image, pcd.pointCloud, (img.timestamp + pcd.timestamp) / 2.0);
    }
    
    cameraThread.join();
    lidarThread.join();
    stopListener.join();

    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");
    SLAM.Shutdown();

    cap.release();
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(appsink);
    gst_object_unref(pipeline);
    
    return 0;
}

bool LoadPointCloudGst(cv::Mat &pcd, GstElement *appsink)
{
    GstSample *sample = nullptr;
    g_signal_emit_by_name(appsink, "pull-sample", &sample);
    if (!sample)
    {
        std::cerr << "Failed to pull sample." << std::endl;
        return false;
    }

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstMapInfo map;

    if (!gst_buffer_map(buffer, &map, GST_MAP_READ))
    {
        std::cerr << "Failed to map buffer." << std::endl;
        gst_sample_unref(sample);
        return false;
    }

    const float *data = reinterpret_cast<const float*>(map.data);

    const int count_h = int(data[0]);
    const int count_v = int(data[1]);
    const float angle_min_h = data[2];
    const float angle_step_h = data[3];
    const float angle_min_v = data[4];
    const float angle_step_v = data[5];
    const float *ranges = data + 6;

    std::vector<cv::Vec4f> validPoints;

    for (int v = 0; v < count_v; v++)
    {
        float angle_v = angle_min_v + v * angle_step_v;
        for (int h = 0; h < count_h; h++)
        {
            int i = v * count_h + h;
            float r = ranges[i];

            // Skip invalid ranges
            if (!std::isfinite(r) || r <= 0.05f || r > 200.0f)
                continue;

            float angle_h = angle_min_h + h * angle_step_h;

            float x = r * cosf(angle_v) * cosf(angle_h);
            float y = r * cosf(angle_v) * sinf(angle_h);
            float z = r * sinf(angle_v);

            // Skip points that would produce zero or invalid depth
            if (!std::isfinite(z) || fabs(z) < 1e-5f)
                continue;

            validPoints.emplace_back(cv::Vec4f(x, y, z, 1.0f));
        }
    }

    if (validPoints.empty())
    {
        std::cerr << "No valid LiDAR points found." << std::endl;
        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);
        return false;
    }

    // Convert to 4xN cv::Mat
    pcd = cv::Mat(4, validPoints.size(), CV_32F);
    for (size_t i = 0; i < validPoints.size(); ++i)
    {
        pcd.at<float>(0, i) = validPoints[i][0];
        pcd.at<float>(1, i) = validPoints[i][1];
        pcd.at<float>(2, i) = validPoints[i][2];
        pcd.at<float>(3, i) = validPoints[i][3];
    }

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);

    return true;
}
