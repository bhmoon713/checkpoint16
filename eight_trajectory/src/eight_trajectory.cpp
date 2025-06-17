#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <vector>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class AbsoluteMotion : public rclcpp::Node
{
public:
  AbsoluteMotion() : Node("absolute_motion_node")
  {
    phi_ = 0.0;
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("wheel_speed", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&AbsoluteMotion::odomCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(10ms, std::bind(&AbsoluteMotion::timerCallback, this));

    // motion sequence (dx, dy)
    motions_ = {
      { 1.0,  0.0},   // forward
      { 0.0,  1.0},   // left
      {-1.0,  0.0},   // backward
      { 0.0, -1.0}    // right
    };

    motion_index_ = 0;
    iteration_ = 0;
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    phi_ = yaw;  // yaw angle
  }

  std::tuple<double, double, double> velocity2twist(double dphi, double dx, double dy)
  {
    double c = std::cos(phi_);
    double s = std::sin(phi_);
    double wz = dphi;
    double vx = c * dx + s * dy;
    double vy = -s * dx + c * dy;
    return {wz, vx, vy};
  }

  std::vector<float> twist2wheels(double wz, double vx, double vy)
  {
    double l = 0.500 / 2.0;
    double w = 0.548 / 2.0;
    double r = 0.254 / 2.0;

    double H[4][3] = {
      {-l - w, 1, -1},
      { l + w, 1,  1},
      { l + w, 1, -1},
      {-l - w, 1,  1}
    };

    std::vector<float> u(4);
    for (int i = 0; i < 4; ++i)
    {
      u[i] = (H[i][0] * wz + H[i][1] * vx + H[i][2] * vy) / r;
    }

    return u;
  }

  void timerCallback()
  {
    if (motion_index_ >= motions_.size()) {
      // Stop the robot
      std_msgs::msg::Float32MultiArray msg;
      msg.data = {0.0, 0.0, 0.0, 0.0};
      publisher_->publish(msg);
      rclcpp::shutdown();
      return;
    }

    auto [dx, dy] = motions_[motion_index_];
    double dphi = M_PI / 6.0;  // 30 degrees/s

    auto [wz, vx, vy] = velocity2twist(dphi, dx, dy);
    std::vector<float> wheels = twist2wheels(wz, vx, vy);

    std_msgs::msg::Float32MultiArray msg;
    msg.data = wheels;
    publisher_->publish(msg);

    iteration_++;
    if (iteration_ >= 300) {
      iteration_ = 0;
      motion_index_++;
    }
  }

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::pair<double, double>> motions_;
  size_t motion_index_;
  size_t iteration_;
  double phi_;  // robot orientation in radians
};
