#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "System.h"

class TrackMonocularNode : public rclcpp::Node
{
public:
    TrackMonocularNode() : Node("track_monocular_node")
    {
        this->declare_parameter<std::string>("topic_name");
        this->declare_parameter<std::string>("slam_vocab_path");
        this->declare_parameter<std::string>("slam_config_path");
        this->declare_parameter<bool>("with_viewer", true);

        if (!this->get_parameter("topic_name", this->topic_name_))
        {
            RCLCPP_ERROR(this->get_logger(), "Parameter topic_name is required but was not provided.");
            return;
        }

        if (!this->get_parameter("slam_vocab_path", this->slam_vocab_path_))
        {
            RCLCPP_ERROR(this->get_logger(), "Parameter slam_vocab_path is required but was not provided.");
            return;
        }

        if (!this->get_parameter("slam_config_path", this->slam_config_path_))
        {
            RCLCPP_ERROR(this->get_logger(), "Parameter slam_config_path is required but was not provided.");
            return;
        }

        this->get_parameter("with_viewer", this->with_viewer_);

        this->SLAM_ = std::make_unique<ORB_SLAM3::System>(this->slam_vocab_path_, this->slam_config_path_, ORB_SLAM3::System::MONOCULAR, this->with_viewer_);
        RCLCPP_INFO(this->get_logger(), "SLAM initialized succesfully");

        this->sub_ = this->create_subscription<sensor_msgs::msg::Image>(this->topic_name_, 10, std::bind(&TrackMonocularNode::img_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Successfully subscribed to topic %s", this->topic_name_.c_str());
    }

    ~TrackMonocularNode()
    {
      RCLCPP_INFO(this->get_logger(), "Shutting down SLAM...");
      this->SLAM_->SaveTrajectoryKITTI("CameraTrajectory.txt");
      this->SLAM_->Shutdown();
      RCLCPP_INFO(this->get_logger(), "SLAM shutdown complete.");
    }

private:
    void img_callback(const sensor_msgs::msg::Image &msg)
    {
        cv::Mat img_original = cv_bridge::toCvCopy(msg, "rgb8")->image;
        cv::Mat img_resized;
        cv::resize(img_original, img_resized, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);
        double ts = rclcpp::Time(msg.header.stamp).seconds();
        this->SLAM_->TrackMonocular(img_resized, ts);
    }

    std::string topic_name_;
    std::string slam_vocab_path_;
    std::string slam_config_path;
    bool with_viewer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    std::unique_ptr<ORB_SLAM3::System> SLAM_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrackMonocularNode>());
    rclcpp::shutdown();
    return 0;
}