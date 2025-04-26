#include <opencv2/core/core.hpp>
#include <gst/gst.h>

#include "System.h"

#include <iostream>
#include <vector>
#include <array>
#include <thread>
#include <string>
#include <atomic>

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
    "udpsrc port=" + lidar_port + " "
    "caps=\"application/octet-stream\" !" 
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
    bool successPcd;
    while (!stopSLAM)
    {
        successPcd = LoadPointCloudGst(pcd, appsink);

        if (!(successPcd && cap.read(img)))
        {
            std::cout << "Could not load frame, skipping\n";
            continue;
        }

        double timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count();
        
        SLAM.TrackRGBL(img, pcd, timestamp);
    }

    SLAM.Shutdown();

    cap.release();
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(appsink);
    gst_object_unref(pipeline);

    stopListener.join();

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
        size_t numPoints = map.size / (sizeof(float) * 3);
        const float *data = reinterpret_cast<const float*>(map.data);

        pcd = cv::Mat::zeros(3, numPoints, CV_32F);
        for (size_t i = 0; i < numPoints; i++)
        {
            pcd.at<float>(0, i) = data[i * 3 + 0]; // x
            pcd.at<float>(1, i) = data[i * 3 + 1]; // y
            pcd.at<float>(2, i) = data[i * 3 + 2]; // z
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
