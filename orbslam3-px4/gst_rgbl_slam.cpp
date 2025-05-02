#include <opencv2/core/core.hpp>
#include <gst/gst.h>

#include "System.h"

#include <iostream>
#include <vector>
#include <array>
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
    "application/x-rtp,media=video,encoding-name=H264,payload=96 ! "
    "rtph264depay ! avdec_h264 ! videoconvert ! "
    "appsink";

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

    // FPS monitoring
    auto last_time = std::chrono::steady_clock::now();
    int frame_count = 0;

    while (!stopSLAM) {
        cap.read(img);
        success = LoadPointCloudGst(pcd, appsink);

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

        double timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::steady_clock::now().time_since_epoch()
        ).count();

        SLAM.TrackRGBL(img, pcd, timestamp);

        // FPS monitoring
        frame_count++;

        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - last_time;

        if (elapsed.count() >= 1.0) {
            std::cout << "Frames per second: " << frame_count << std::endl;
            frame_count = 0;
            last_time = now;
        }
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

    if (gst_buffer_map(buffer, &map, GST_MAP_READ))
    {
        const float *data = reinterpret_cast<const float*>(map.data);

        // Extract data from message
        const int count_h = int(data[0]);
        const int count_v = int(data[1]);
        const float angle_min_h = data[2];
        const float angle_step_h = data[3];
        const float angle_min_v = data[4];
        const float angle_step_v = data[5];
        const float *ranges = data + 6;
        
        // Initialize pcd
        pcd = cv::Mat::zeros(4, count_h * count_v, CV_32F);

        // Calculate (x, y, z) for each point to fill pcd
        for (int v = 0; v < count_v; v++)
        {
            float angle_v = angle_min_v + v * angle_step_v;
            for (int h = 0; h < count_h; h++)
            {
                float angle_h = angle_min_h + h * angle_step_h;
                int i = v * count_h + h;
                float r = ranges[i];

                // skip invalid point
                if (std::isnan(r)) continue;

                // 3D point calculation
                float x = r * cosf(angle_v) * cosf(angle_h);
                float y = r * cosf(angle_v) * sinf(angle_h);
                float z = r * sinf(angle_v);

                // Fill pcd
                pcd.ptr<float>(0)[i] = x;
                pcd.ptr<float>(1)[i] = y;
                pcd.ptr<float>(2)[i] = z;
                pcd.ptr<float>(3)[i] = 1.0f;
            }
        }

        gst_buffer_unmap(buffer, &map);
    }
    else
    {
        std::cerr << "Failed to map buffer." << std::endl;
        return false;
    }

    gst_sample_unref(sample);
    return true;
}