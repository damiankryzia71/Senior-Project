#include <opencv2/core/core.hpp>
#include <gst/gst.h>

#include "System.h"

#include <thread>
#include <string>
#include <vector>
#include <atomic>

bool LoadPointcloudBinaryMatFromGst(cv::Mat &point_cloud, GstElement *appsink);
void VisualizePointCloud2D(const cv::Mat &point_cloud);

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

int main(int argc, char** argv)
{
  if (argc < 2)
    {
        std::cout << "USAGE: px4_stream UDP_PORT" << std::endl;
        return 1;
    }

    // Get udp port
    const std::string udpPort = argv[1];

    // Initialize GStreamer pipeline
    GstElement *pipeline = nullptr;
    GstElement *appsink = nullptr;

    gst_init(nullptr, nullptr);

    std::string pipelineString = "udpsrc port=" + udpPort + " caps=\"application/octet-stream\" ! appsink name=sink";

    GError* error;
    pipeline = gst_parse_launch(pipelineString.c_str(), &error);

    if (!pipeline)
    {
        std::cerr << "Failed to create GStreamer pipeline: " << error->message << std::endl;
        g_error_free(error);
        return 1;
    }

    appsink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    std::thread stopListener(listenForStopCommand);

    bool success;
    cv::Mat point_cloud;
    while (!stopSLAM)
    {
        success = LoadPointcloudBinaryMatFromGst(point_cloud, appsink);

        if (!success)
        {
            std::cout << "Skipping frame, could not load point_cloud\n";
            continue;
        }

        std::cout << "Processing frame\n";
        VisualizePointCloud2D(point_cloud);
    }

    std::cout << "Done processing" << std::endl;

    stopListener.join();

    return 0;
}

bool LoadPointcloudBinaryMatFromGst(cv::Mat &point_cloud, GstElement *appsink)
{
    GstSample *sample = nullptr;
    g_signal_emit_by_name(appsink, "pull-sample", &sample);

    if (!sample)
    {
        std::cerr << "No data received from GStreamer stream." << std::endl;
        return false;
    }

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstMapInfo map;

    if (!gst_buffer_map(buffer, &map, GST_MAP_READ))
    {
        std::cerr << "Failed to map GStreamer buffer." << std::endl;
        gst_sample_unref(sample);
        return false;
    }

    const float *data = reinterpret_cast<const float *>(map.data);
    const size_t numFloats = map.size / sizeof(float);
    const size_t numPoints = numFloats / 4;

    point_cloud = cv::Mat::zeros(cv::Size(numPoints, 4), CV_32F);

    for (size_t i = 0; i < numPoints; ++i)
    {
        point_cloud.at<float>(0, i) = data[i * 4 + 0]; // x
        point_cloud.at<float>(1, i) = data[i * 4 + 1]; // y
        point_cloud.at<float>(2, i) = data[i * 4 + 2]; // Z
        point_cloud.at<float>(3, i) = 1.0f;
    }

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);

    return true;
}

void VisualizePointCloud2D(const cv::Mat &point_cloud) {
    int width = 800;
    int height = 800;
    float scale = 10.0f;

    cv::Mat display = cv::Mat::zeros(height, width, CV_8UC3);

    for (int i = 0; i < point_cloud.cols; ++i) {
        float x = point_cloud.at<float>(0, i);
        float z = point_cloud.at<float>(2, i); // Top-down view: X-Z plane

        int u = static_cast<int>(x * scale + width / 2);
        int v = static_cast<int>(z * scale + height / 2);

        if (u >= 0 && u < width && v >= 0 && v < height) {
            cv::circle(display, cv::Point(u, v), 1, cv::Scalar(0, 255, 0), -1);
        }
    }

    cv::imshow("Point Cloud Viewer", display);
    cv::waitKey(30); // wait 30 ms between frames
}
