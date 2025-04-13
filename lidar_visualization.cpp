#include <opencv2/core/core.hpp>

#include "System.h"

#include <thread>
#include <string>
#include <atomic>
#include <fstream>

void LoadFiles(const std::string &pathToSequence, std::vector<std::string> &filenames, std::vector<double> &timestamps);
bool LoadPointcloudBinaryMat(const std::string& FilePath, cv::Mat& point_cloud);
void VisualizePointCloud2D(const cv::Mat &point_cloud);

int main(int argc, char** argv)
{
  if (argc < 2)
    {
        std::cout << "USAGE: px4_stream PATH_TO_SEQUENCE" << std::endl;
        return 1;
    }

    std::string pathToSequence = argv[1];
    std::vector<std::string> filenames;
    std::vector<double> timestamps;

    LoadFiles(pathToSequence, filenames, timestamps);

    const int n = filenames.size();
    bool success;
    cv::Mat pcd;
    for (int i = 0; i < n; i++)
    {
        success = LoadPointcloudBinaryMat(filenames[i], pcd);

        if (!success)
        {
            std::cout << "Skipping frame, could not load pcd: " << filenames[i] << std::endl;
            continue;
        }

        std::cout << "Processing frame at: " << filenames[i] << std::endl;
        VisualizePointCloud2D(pcd);
    }

    std::cout << "Done processing" << std::endl;

    return 0;
}

void LoadFiles(const std::string &pathToSequence, std::vector<std::string> &filenames, std::vector<double> &timestamps)
{
    // Load timestamps
    std::ifstream fTimes;
    std::string pathTimestapms = pathToSequence + "color/sequences/00/times.txt";
    fTimes.open(pathTimestapms);
    while (!fTimes.eof())
    {
        std::string s;
        getline(fTimes, s);
        if (!s.empty())
        {
            std::stringstream ss;
            ss << s;
            double t;
            ss >> t;
            timestamps.push_back(t);
        }
    }

    // Load pointcloud files
    std::string pathPcd = pathToSequence + "velodyne/sequences/00/velodyne/";
    const int n = timestamps.size();
    filenames.resize(n);
    for (int i = 0; i < n; i++)
    {
        std::stringstream ss;
        ss << setfill('0') << setw(6) << i;
        filenames[i] = pathPcd + ss.str() + ".bin";
    }
}

bool LoadPointcloudBinaryMat(const std::string& FilePath, cv::Mat& point_cloud){
    // Initialization
    int32_t num = 1000000; // maximum Number of points to allocate
    float* data = (float*) malloc(num * sizeof(float));
    float* px = data + 0;
    float* py = data + 1;
    float* pz = data + 2;
    float* pr = data + 3;

    // load point cloud from file
    std::FILE* stream;
    stream = fopen(FilePath.c_str(), "rb");

    // Save data to variable
    num = fread(data, sizeof(float), num, stream)/4;

    // Clear Pointcloud variable
    point_cloud = cv::Mat::zeros(cv::Size(num, 4), CV_32F);

    // Format data as desired
    for (int32_t i = 0; i < num; i++) {
        point_cloud.at<float>(0, i) = (float)*px;
        point_cloud.at<float>(1, i) = (float)*py;
        point_cloud.at<float>(2, i) = (float)*pz;
        point_cloud.at<float>(3, i) = (float)1;
        px+=4; py+=4; pz+=4; pr+=4;
    }
    
    // Close Stream and Free Memory
    fclose(stream);
    free(data);

    // Feedback and return
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
