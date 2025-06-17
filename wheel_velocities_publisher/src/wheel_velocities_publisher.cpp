#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <vector>
#include <chrono>

using namespace std::chrono_literals;

class WheelVelocitiesPublisher : public rclcpp::Node
{
public:
  WheelVelocitiesPublisher()
  : Node("wheel_velocities_publisher"), step_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/wheel_speed", 10);

    timer_ = this->create_wall_timer(1s, std::bind(&WheelVelocitiesPublisher::publish_sequence, this));
  }

private:
  void publish_sequence()
  {
    std_msgs::msg::Float32MultiArray msg;

    // Define speeds for motions: [FL, FR, RL, RR]
    std::vector<std::vector<float>> motions = {
      { 1.0,  1.0,  1.0,  1.0},  // forward
      {-1.0, -1.0, -1.0, -1.0},  // backward
      {-1.0, 1.0, 1.0, -1.0},    // left
      { 1.0, -1.0, -1.0, 1.0},   // right
      { 1.0, -1.0, 1.0, -1.0},   // clockwise
      {-1.0, 1.0, -1.0, 1.0},    // counter-clockwise
      { 0.0,  0.0,  0.0,  0.0}   // stop
    };

    std::vector<std::string> motion_names = {
      "Moving forward",
      "Moving backward",
      "Moving left",
      "Moving right",
      "Turning clockwise",
      "Turning counter-clockwise",
      "Stopping"
    };

    if (step_ < motions.size() * 3) {
      int motion_index = step_ / 3;

      if (step_ % 3 == 0) {
        RCLCPP_INFO(this->get_logger(), "%s", motion_names[motion_index].c_str());
      }

      msg.data = motions[motion_index];
      publisher_->publish(msg);
      step_++;
    } else {
      rclcpp::shutdown();  // Optional: shutdown after all motions complete
    }
  }

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int step_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelVelocitiesPublisher>());
  rclcpp::shutdown();
  return 0;
}
