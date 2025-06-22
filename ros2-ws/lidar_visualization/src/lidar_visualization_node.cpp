#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "opencv2/opencv.hpp"

class LidarVisualizationNode : public rclcpp::Node
{
public:
    LidarVisualizationNode() : Node("lidar_visualization_node")
    {
        this->declare_parameter<std::string>("pcd_topic");

        if (!this->get_parameter("pcd_topic", this->pcd_topic_))
        {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'pcd_topic' is required but was not set");
            rclcpp::shutdown();
            return;
        }

        this->sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(this->pcd_topic_, 10, std::bind(&LidarVisualizationNode::topic_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Successfully subscribed to topic: %s", this->pcd_topic_.c_str());
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2 &msg)
    {
        if (msg.fields.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "PointCloud2 has no points!");
            return;
        }

        std::vector<cv::Vec4f> valid_points;

        sensor_msgs::msg::PointCloud2 msg_copy = msg;
        sensor_msgs::PointCloud2Iterator<float> iter_x(msg_copy, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg_copy, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(msg_copy, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                continue;

            valid_points.emplace_back(cv::Vec4f(x, y, z, 1.0f));
        }

        if (valid_points.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "No valid points found!");
            return;
        }

        cv::Mat pcd(4, valid_points.size(), CV_32F);
        for (size_t i = 0; i < valid_points.size(); ++i)
        {
            pcd.at<float>(0, i) = valid_points[i][0];
            pcd.at<float>(1, i) = valid_points[i][1];
            pcd.at<float>(2, i) = valid_points[i][2];
            pcd.at<float>(3, i) = valid_points[i][3];
        }

        int image_size = 800;
        cv::Mat display = cv::Mat::zeros(image_size, image_size, CV_8UC3);

        float scale = 50.0f;
        cv::Point2f center(image_size / 2.0f, image_size / 2.0f);

        for (int i = 0; i < pcd.cols; i++)
        {
            float x = pcd.at<float>(0, i);
            float y = pcd.at<float>(1, i);

            int u = static_cast<int>(x * scale + center.x);
            int v = static_cast<int>(y * scale + center.y);

            if (u >= 0 && u < image_size && v >= 0 && v < image_size)
            {
                cv::circle(display, cv::Point(u, v), 1, cv::Scalar(0, 255, 0), -1);
            }
        }

        cv::imshow("Point Cloud", display);
        cv::waitKey(30);
    }

    std::string pcd_topic_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarVisualizationNode>());
    rclcpp::shutdown();
    return 0;
}