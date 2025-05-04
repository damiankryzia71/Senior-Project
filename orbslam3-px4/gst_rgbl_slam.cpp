#include <opencv2/core/core.hpp>
#include <gst/gst.h>

#include "System.h"

#include <iostream>
#include <vector>
#include <thread>
#include <string>
#include <atomic>
#include <chrono>

bool LoadPointCloudGst(cv::Mat &pcd, GstElement *appsink);

std::atomic<bool> stopSLAM(false);

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
    
    cv::Mat img, pcd;
    bool success;

    double time_img = 0.0;
    double time_lidar = 0.0;

    while (!stopSLAM) {
        auto t_img_start = std::chrono::steady_clock::now();
        cap.read(img);
        time_img = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now().time_since_epoch()).count();
    
        auto t_lidar_start = std::chrono::steady_clock::now();
        success = LoadPointCloudGst(pcd, appsink);
        time_lidar = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now().time_since_epoch()).count();

        if (!success)
        {
            std::cerr << "Failed to load point cloud, skipping frame" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        if (img.empty())
        {
            std::cerr << "Failed to load image, skipping frame" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        double dt = std::abs(time_img - time_lidar);
        if (dt > 0.03) {
            std::cerr << "[Sync] Time mismatch (Δt = " << dt << " s), skipping frame pair" << std::endl;
            continue;
        }
    
        double avg_time = (time_img + time_lidar) / 2.0;
    
        SLAM.TrackRGBL(img, pcd, avg_time);
    }
    
    SLAM.Shutdown();
    
    cap.release();
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(appsink);
    gst_object_unref(pipeline);
    
    stopListener.join();

    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

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