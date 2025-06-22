#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "System.h"

class RealsenseSubscriber : public rclcpp::Node
{
public:
    RealsenseSubscriber() : Node("realsense_subscriber")
    {
        this->declare_parameter<std::string>("topic_name");

        if (!this->get_parameter("topic_name", this->topic_name_))
        {
            RCLCPP_ERROR(this->get_logger(), "Parameter topic_name is required but was not provided.");
            return;
        }

        this->SLAM_ = std::make_unique<ORB_SLAM3::System>("/home/rescue1/ORB_SLAM3_RGBL/Vocabulary/ORBvoc.txt",
            "/home/rescue1/ros2-ws/src/orbslam3_ros2/config.yaml",
            ORB_SLAM3::System::MONOCULAR,
            true
        );

        RCLCPP_INFO(this->get_logger(), "SLAM initialized succesfully");

        this->sub_ = this->create_subscription<sensor_msgs::msg::Image>(this->topic_name_, 10, std::bind(&RealsenseSubscriber::img_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Successfully subscribed to topic %s", this->topic_name_.c_str());
    }

    ~RealsenseSubscriber()
    {
      RCLCPP_INFO(this->get_logger(), "Shutting down SLAM...");
      this->SLAM_->SaveTrajectoryKITTI("CameraTrajectory.txt");
      this->SLAM_->Shutdown();
      RCLCPP_INFO(this->get_logger(), "SLAM shutdown complete.");
    }

private:
    void img_callback(const sensor_msgs::msg::Image &msg)
    {
        RCLCPP_INFO(this->get_logger(), "Image callback entered");

        cv::Mat img_original = cv_bridge::toCvCopy(msg, "rgb8")->image;
        RCLCPP_INFO(this->get_logger(), "Image converted successfully.");

        cv::Mat img_resized;
        cv::resize(img_original, img_resized, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);
        RCLCPP_INFO(this->get_logger(), "Image resized to 640x480");

        double ts = rclcpp::Time(msg.header.stamp).seconds();
        RCLCPP_INFO(this->get_logger(), "Timestamp obtained successfully");

        this->SLAM_->TrackMonocular(img_resized, ts);
    }

    std::string topic_name_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    std::unique_ptr<ORB_SLAM3::System> SLAM_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RealsenseSubscriber>());
    rclcpp::shutdown();
    return 0;
}