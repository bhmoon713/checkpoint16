#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <vector>

class KinematicModel : public rclcpp::Node
{
public:
  KinematicModel() : Node("kinematic_model")
  {
    sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/wheel_speed", 10,
      std::bind(&KinematicModel::wheelCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Parameters (adjust these to your robot)
    r_ = 0.05;   // wheel radius in meters
    l_ = 0.085;    // half length
    w_ = 0.134845;    // half width
  }

private:
  void wheelCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 4) return;

    float u1 = msg->data[0];
    float u2 = msg->data[1];
    float u3 = msg->data[2];
    float u4 = msg->data[3];

    float sum = l_ + w_;
    float vx = (u1 + u2 + u3 + u4) * r_ / 4.0;
    float vy = (-u1 + u2 - u3 + u4) * r_ / 4.0;
    float wz = (-u1 + u2 + u3 - u4) * r_ / (4.0 * sum);

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = vx;
    cmd_vel.linear.y = vy;
    cmd_vel.angular.z = wz;

    pub_->publish(cmd_vel);
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  float r_, l_, w_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinematicModel>());
  rclcpp::shutdown();
  return 0;
}
