#include <chrono>
#include <cmath>
#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <tf2/transform_datatypes.h>
// #include <tf2/LinearMath/Matrix3x3.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class AbsoluteMotionNode : public rclcpp::Node
{
public:
    AbsoluteMotionNode() : Node("absolute_motion"), phi_(0.0)
    {
        pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("wheel_speed", 10);
        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&AbsoluteMotionNode::odom_callback, this, std::placeholders::_1));

        this->declare_parameter("sleep_sec", 1.0);
        double sleep_sec = this->get_parameter("sleep_sec").as_double();
        rclcpp::sleep_for(std::chrono::duration<double>(sleep_sec));

        // Define motions
        std::vector<std::tuple<double, double, double>> motions = {
            {0.0, 1, -1},
            {0.0, 1, 1},
            {0.0, 1, 1},
            {-1.5708, 1, -1},
            {-1.5708, -1, -1},
            {0.0, -1, 1},
            {0.0, -1, 1},
            {0.0, -1, -1}
        };

        for (const auto &[mphi, mx, my] : motions)
        {
            double dx = mx;
            double dy = my;
            double dphi = mphi;

            for (int i = 0; i < 300; ++i)
            {
                auto [wz, vx, vy] = velocity2twist(dphi, dx, dy);
                std::vector<float> u = twist2wheels(wz, vx, vy);

                std_msgs::msg::Float32MultiArray msg;
                msg.data = u;
                pub_->publish(msg);
                rclcpp::sleep_for(10ms);
            }
        }

        // Stop
        std_msgs::msg::Float32MultiArray stop_msg;
        stop_msg.data = {0, 0, 0, 0};
        pub_->publish(stop_msg);
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    double phi_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        phi_ = yaw;
    }

    std::tuple<double, double, double> velocity2twist(double dphi, double dx, double dy)
    {
        double cos_phi = std::cos(phi_);
        double sin_phi = std::sin(phi_);
        double wz = dphi;
        double vx = cos_phi * dx + sin_phi * dy;
        double vy = -sin_phi * dx + cos_phi * dy;
        return {wz, vx, vy};
    }

    std::vector<float> twist2wheels(double wz, double vx, double vy)
    {
        double r = 0.05;
        double l = 0.085;
        double w = 0.134845;

        std::vector<std::vector<double>> H = {
            {-l - w, 1, -1},
            {l + w, 1, 1},
            {l + w, 1, -1},
            {-l - w, 1, 1}
        };

        std::vector<float> u(4);
        for (int i = 0; i < 4; ++i)
        {
            u[i] = static_cast<float>((H[i][0] * wz + H[i][1] * vx + H[i][2] * vy) / r);
        }

        return u;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AbsoluteMotionNode>());
    rclcpp::shutdown();
    return 0;
}
