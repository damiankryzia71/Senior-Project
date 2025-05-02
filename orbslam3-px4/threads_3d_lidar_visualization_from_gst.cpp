#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <gst/gst.h>

#include <iostream>
#include <vector>
#include <string>
#include <thread>

bool LoadPointCloudGst(cv::Mat &pcd, GstElement *appsink);
void ComputePointCloudChunk(cv::Mat &pcd, const float *ranges, 
                            int start_idx, int end_idx, int count_h, 
                            float angle_min_h, float angle_step_h, 
                            float angle_min_v, float angle_step_v);
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
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
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

    if (gst_buffer_map(buffer, &map, GST_MAP_READ))
    {
        const float *data = reinterpret_cast<const float*>(map.data);

        // Extract data from message
        const int count_h = static_cast<int>(data[0]);
        const int count_v = static_cast<int>(data[1]);
        const float angle_min_h = data[2];
        const float angle_step_h = data[3];
        const float angle_min_v = data[4];
        const float angle_step_v = data[5];
        const float *ranges = data + 6;
        
        // Initialize pcd
        pcd = cv::Mat::zeros(3, count_h * count_v, CV_32F);

        int n = count_h * count_v; // total points
        int num_threads = (n / 1000) + 1;
        int chunk = n / num_threads;

        // Calculate (x, y, z) for each point to fill pcd
        std::vector<std::thread> threads;
        for (int i = 0; i < num_threads; i++)
        {
            int start = i * chunk;
            int end = (i == num_threads - 1) ? n : start + chunk;

            threads.emplace_back(ComputePointCloudChunk, std::ref(pcd), ranges, start, end, count_h, angle_min_h, angle_step_h, angle_min_v, angle_step_v);
        }
        
        for (auto &t : threads) t.join();

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

void ComputePointCloudChunk(cv::Mat &pcd, const float *ranges, 
    int start_idx, int end_idx, int count_h, 
    float angle_min_h, float angle_step_h, 
    float angle_min_v, float angle_step_v)
{
    for (int i = start_idx; i < end_idx; ++i) {
        int v = i / count_h;
        int h = i % count_h;

        float angle_v = angle_min_v + v * angle_step_v;
        float angle_h = angle_min_h + h * angle_step_h;
        float r = ranges[i];

        if (std::isnan(r)) continue;

        float x = r * cosf(angle_v) * cosf(angle_h);
        float y = r * cosf(angle_v) * sinf(angle_h);
        float z = r * sinf(angle_v);

        pcd.ptr<float>(0)[i] = x;
        pcd.ptr<float>(1)[i] = y;
        pcd.ptr<float>(2)[i] = z;
    }
}