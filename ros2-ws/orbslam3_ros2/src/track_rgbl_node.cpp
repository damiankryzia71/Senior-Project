#include <string>
#include <mutex>
#include <queue>
#include <vector>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "System.h"

static constexpr size_t kMaxQueueSize = 100;

class TrackRGBLNode : public rclcpp::Node
{
public:
  TrackRGBLNode() : Node("track_rgbl_node"), slam_running_(true)
  {
    this->declare_parameter<std::string>("image_topic");
    this->declare_parameter<std::string>("pcd_topic");
    this->declare_parameter<std::string>("slam_vocab_path");
    this->declare_parameter<std::string>("slam_config_path");
    this->declare_parameter<bool>("with_viewer", true);

    if (!this->get_parameter("image_topic", this->image_topic_))
    {
      RCLCPP_ERROR(this->get_logger(), "Parameter 'image_topic' is required but not set.");
      rclcpp::shutdown();
      return;
    }

    if (!this->get_parameter("pcd_topic", this->pcd_topic_))
    {
      RCLCPP_ERROR(this->get_logger(), "Parameter 'pcd_topic' is required but not set.");
      rclcpp::shutdown();
      return;
    }

    if (!this->get_parameter("slam_vocab_path", this->slam_vocab_path_))
    {
      RCLCPP_ERROR(this->get_logger(), "Parameter 'slam_vocab_path' is required but not set.");
      rclcpp::shutdown();
      return;
    }

    if (!this->get_parameter("slam_config_path", this->slam_config_path_))
    {
      RCLCPP_ERROR(this->get_logger(), "Parameter 'slam_config_path' is required but not set.");
      rclcpp::shutdown();
      return;
    }

    this->get_parameter("with_viewer", this->with_viewer_);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                   .reliability(rclcpp::ReliabilityPolicy::Reliable)
                   .durability(rclcpp::DurabilityPolicy::Volatile);

    this->image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(image_topic_, qos, std::bind(&TrackRGBLNode::image_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribed to image topic: %s", image_topic_.c_str());

    this->pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(pcd_topic_, qos, std::bind(&TrackRGBLNode::pcd_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribed to point cloud topic: %s", pcd_topic_.c_str());

    this->SLAM_ = std::make_unique<ORB_SLAM3::System>(slam_vocab_path_, slam_config_path_, ORB_SLAM3::System::RGBL, this->with_viewer_);

    this->slam_thread_ = std::thread(&TrackRGBLNode::slam_loop, this);
  }

  ~TrackRGBLNode()
  {
    this->slam_running_ = false;
    if (this->slam_thread_.joinable())
      this->slam_thread_.join();

    shutdown_slam();
  }

private:
  void slam_loop()
  {
    rclcpp::Rate rate(100);

    while (rclcpp::ok() && this->slam_running_)
    {
      std::pair<cv::Mat, rclcpp::Time> img;
      std::pair<cv::Mat, rclcpp::Time> pcd;

      {
        std::lock_guard<std::mutex> lock_img(this->img_mutex);
        std::lock_guard<std::mutex> lock_pcd(this->pcd_mutex);

        if (this->img_queue.empty() || this->pcd_queue.empty())
        {
          rate.sleep();
          continue;
        }

        img = this->img_queue.front();
        pcd = this->pcd_queue.front();

        double dt = std::abs(img.second.seconds() - pcd.second.seconds());
        if (dt > 0.02)
        {
          RCLCPP_WARN(this->get_logger(), "Timestamp mismatch too large (%.6f s), skipping frame pair", dt);
          if (img.second < pcd.second)
            this->img_queue.pop();
          else
            this->pcd_queue.pop();
          continue;
        }

        this->img_queue.pop();
        this->pcd_queue.pop();
      }

      if (img.first.empty() || img.first.cols == 0 || img.first.rows == 0)
      {
        RCLCPP_WARN(this->get_logger(), "Skipped SLAM frame: empty or invalid image matrix");
        continue;
      }

      if (pcd.first.empty() || pcd.first.cols == 0 || pcd.first.rows == 0)
      {
        RCLCPP_WARN(this->get_logger(), "Skipped SLAM frame: empty or invalid point cloud matrix");
        continue;
      }

      double ts_img = img.second.seconds();
      double ts_pcd = pcd.second.seconds();
      double ts_avg = (ts_img + ts_pcd) / 2.0;

      RCLCPP_INFO(this->get_logger(), "Processing synchronized RGB-L frame at avg time: %.6f", ts_avg);
      this->SLAM_->TrackRGBL(img.first, pcd.first, ts_avg);
    }
  }

  void shutdown_slam()
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down SLAM...");
    this->SLAM_->SaveTrajectoryKITTI("CameraTrajectory.txt");
    this->SLAM_->Shutdown();
    RCLCPP_INFO(this->get_logger(), "SLAM shutdown complete.");
  }

  void image_callback(const sensor_msgs::msg::Image &msg)
  {
    std::lock_guard<std::mutex> lock(img_mutex);

    RCLCPP_INFO(this->get_logger(), "Image callback entered");

    cv::Mat img_original = cv_bridge::toCvCopy(msg, "rgb8")->image;
    RCLCPP_INFO(this->get_logger(), "Image converted successfully.");

    cv::Mat img_resized;
    cv::resize(img_original, img_resized, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);
    RCLCPP_INFO(this->get_logger(), "Image resized to 640x480");

    rclcpp::Time ts = msg.header.stamp;
    RCLCPP_INFO(this->get_logger(), "Timestamp obtained successfully");

    this->img_queue.push(std::make_pair(img_resized, ts));
    if (this->img_queue.size() > kMaxQueueSize)
      this->img_queue.pop();
  }

  void pcd_callback(const sensor_msgs::msg::PointCloud2 &msg)
  {
    std::lock_guard<std::mutex> lock(pcd_mutex);

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

    rclcpp::Time ts = msg.header.stamp;

    this->pcd_queue.push(std::make_pair(pcd, ts));
    if (this->pcd_queue.size() > kMaxQueueSize)
      this->pcd_queue.pop();
  }

  std::string image_topic_;
  std::string pcd_topic_;
  std::string slam_vocab_path_;
  std::string slam_config_path_;
  bool with_viewer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  std::mutex img_mutex, pcd_mutex;
  std::queue<std::pair<cv::Mat, rclcpp::Time>> img_queue, pcd_queue;
  std::unique_ptr<ORB_SLAM3::System> SLAM_;
  std::thread slam_thread_;
  std::atomic<bool> slam_running_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackRGBLNode>());
  rclcpp::shutdown();
  return 0;
}
