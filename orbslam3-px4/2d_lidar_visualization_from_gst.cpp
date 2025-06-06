#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <gst/gst.h>

#include <iostream>
#include <vector>
#include <array>
#include <string>

bool LoadPointCloudGst(cv::Mat &pcd, GstElement *appsink);
void DisplayPointCloud(const cv::Mat &pcd);

int main(int argc, char** argv) {
    gst_init(&argc, &argv);

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << "lidar_port" << std::endl;
        return 1;
    }

    std::string lidar_port = argv[1];

    std::string lidar_pipeline_desc = 
    "udpsrc port=" + lidar_port + " ! " 
    "appsink name=mysink";

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

    cv::Mat pcd;
    bool successPcd;
    while (true)
    {
        successPcd = LoadPointCloudGst(pcd, appsink);

        if (!successPcd)
        {
            std::cout << "Could not load frame, skipping\n";
            continue;
        }

        DisplayPointCloud(pcd);
    }

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

    const int count = static_cast<int>(data[0]);
    const float angle_min = data[1];
    const float angle_step = data[2];
    const float *ranges = data + 3;

    std::vector<cv::Vec2f> validPoints;

    for (int i = 0; i < count; ++i)
    {
        float r = ranges[i];
        if (!std::isfinite(r) || r <= 0.05f || r > 50.0f)
            continue;

        float angle = angle_min + i * angle_step;
        float x = r * cosf(angle);
        float y = r * sinf(angle);

        validPoints.emplace_back(cv::Vec2f(x, y));
    }

    if (validPoints.empty())
    {
        std::cerr << "No valid LiDAR points found." << std::endl;
        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);
        return false;
    }

    pcd = cv::Mat(2, validPoints.size(), CV_32F);
    for (size_t i = 0; i < validPoints.size(); ++i)
    {
        pcd.at<float>(0, i) = validPoints[i][0];
        pcd.at<float>(1, i) = validPoints[i][1];
    }

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
    
    return true;
}

void DisplayPointCloud(const cv::Mat &pcd) {
    int image_size = 800;
    cv::Mat display = cv::Mat::zeros(image_size, image_size, CV_8UC3);

    float scale = 50.0f;
    cv::Point2f center(image_size / 2.0f, image_size / 2.0f);

    for (int i = 0; i < pcd.cols; i++) {
        float x = pcd.at<float>(0, i);
        float y = pcd.at<float>(1, i);

        int u = static_cast<int>(x * scale + center.x);
        int v = static_cast<int>(y * scale + center.y);

        if (u >= 0 && u < image_size && v >= 0 && v < image_size) {
            cv::circle(display, cv::Point(u, v), 1, cv::Scalar(0, 255, 0), -1);
        }
    }

    cv::imshow("Point Cloud", display);
    cv::waitKey(30);
}