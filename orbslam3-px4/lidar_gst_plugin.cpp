#ifndef LIDAR_GST_PLUGIN
#define LIDAR_GST_PLUGIN

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <iostream>
#include <string>

namespace gazebo
{
    class LidarGstPlugin : public SensorPlugin
    {
        public:
            LidarGstPlugin() {}

            virtual void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
            {
                this->sensor = sensor;
                this->node = transport::NodePtr(new transport::Node());
                this->node->Init();
                this->sub = this->node->Subscribe(topicName, &LidarGstPlugin::Callback, this);

                std::cout << "Plugin loaded" << std::endl;
            }

            void VisualizePointCloud2D(const cv::Mat &point_cloud) {
                const int width = 800;
                const int height = 800;
                const float scale = 50.0f;
                
                cv::Mat display = cv::Mat::zeros(height, width, CV_8UC3);

                for (int i = 0; i <= width; i += 100)
                    cv::line(display, cv::Point(i, 0), cv::Point(i, height), cv::Scalar(50, 50, 50));
                for (int j = 0; j <= height; j += 100)
                    cv::line(display, cv::Point(0, j), cv::Point(width, j), cv::Scalar(50, 50, 50));
            
                cv::circle(display, cv::Point(width / 2, height / 2), 4, cv::Scalar(0, 0, 255), -1);
            
                for (int i = 0; i < point_cloud.cols; ++i) {
                    float x = point_cloud.at<float>(0, i);
                    float y = point_cloud.at<float>(1, i);
            
                    int u = static_cast<int>(x * scale + width / 2);
                    int v = static_cast<int>(-y * scale + height / 2);
            
                    if (u >= 0 && u < width && v >= 0 && v < height) {
                        cv::circle(display, cv::Point(u, v), 1, cv::Scalar(0, 255, 0), -1);
                    }
                }
            
                cv::imshow("LiDAR 2D View", display);
                cv::waitKey(30);
            }            

            void Callback(ConstLaserScanStampedPtr &msg)
            {
                const gazebo::msgs::LaserScan &scan = msg->scan();

                int num_points = scan.ranges_size();
                float angle_min = scan.angle_min();
                float angle_step = scan.angle_step();
                
                cv::Mat point_cloud(3, num_points, CV_32F);

                for (int i = 0; i < num_points; ++i)
                {
                    float range = scan.ranges(i);
                    float angle = angle_min + i * angle_step;
            
                    float x = range * cos(angle);
                    float y = range * sin(angle);
                    float z = 0.0f;
            
                    point_cloud.at<float>(0, i) = x;
                    point_cloud.at<float>(1, i) = y;
                    point_cloud.at<float>(2, i) = z;
                }

                VisualizePointCloud2D(point_cloud);
            }
            
        private:
            sensors::SensorPtr sensor;
            transport::NodePtr node;
            transport::SubscriberPtr sub;
            std::string topicName = "/gazebo/default/lidar_model/lidar_link/lidar_sensor/scan";
    };

    GZ_REGISTER_SENSOR_PLUGIN(LidarGstPlugin)
}

#endif
