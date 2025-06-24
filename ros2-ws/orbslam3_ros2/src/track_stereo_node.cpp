#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "System.h"
#include <opencv2/opencv.hpp>

class TrackStereoNode : public rclcpp::Node
{
public:
    TrackStereoNode() : Node("track_stereo_node")
    {
        this->declare_parameter<std::string>("left_topic");
        this->declare_parameter<std::string>("right_topic");
        this->declare_parameter<std::string>("slam_vocab_path");
        this->declare_parameter<std::string>("slam_config_path");
        this->declare_parameter<bool>("with_viewer", true);

        this->get_parameter("left_topic", this->left_topic_);
        this->get_parameter("right_topic", this->right_topic_);
        this->get_parameter("slam_vocab_path", this->slam_vocab_path_);
        this->get_parameter("slam_config_path", this->slam_config_path_);
        this->get_parameter("with_viewer", this->with_viewer_);

        this->SLAM_ = std::make_unique<ORB_SLAM3::System>(this->slam_vocab_path_, this->slam_config_path_, ORB_SLAM3::System::STEREO, this->with_viewer_);
        this->sub_left_ = this->create_subscription<sensor_msgs::msg::Image>(this->left_topic_, 10, std::bind(&TrackStereoNode::left_callback, this, std::placeholders::_1));
        this->sub_right_ = this->create_subscription<sensor_msgs::msg::Image>(this->right_topic_, 10, std::bind(&TrackStereoNode::right_callback, this, std::placeholders::_1));
    }

    ~TrackStereoNode()
    {
        this->SLAM_->SaveTrajectoryKITTI("CameraTrajectory.txt");
        this->SLAM_->Shutdown();
    }

private:
    void left_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        this->left_image_ = cv_bridge::toCvCopy(msg, "mono8")->image;
        this->left_stamp_ = rclcpp::Time(msg->header.stamp).seconds();
        try_track();
    }

    void right_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        this->right_image_ = cv_bridge::toCvCopy(msg, "mono8")->image;
        this->right_stamp_ = rclcpp::Time(msg->header.stamp).seconds();
        try_track();
    }

    void try_track()
    {
        if (!this->left_image_.empty() && !this->right_image_.empty() && std::abs(this->left_stamp_ - this->right_stamp_) < 0.01)
        {
            this->SLAM_->TrackStereo(this->left_image_, this->right_image_, this->left_stamp_);
            this->left_image_.release();
            this->right_image_.release();
        }
    }

    std::string left_topic_, right_topic_, slam_vocab_path_, slam_config_path_;
    bool with_viewer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_left_, sub_right_;
    std::unique_ptr<ORB_SLAM3::System> SLAM_;

    cv::Mat left_image_, right_image_;
    double left_stamp_ = -1.0, right_stamp_ = -1.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrackStereoNode>());
    rclcpp::shutdown();
    return 0;
}