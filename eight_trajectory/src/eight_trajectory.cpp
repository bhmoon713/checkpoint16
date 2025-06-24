#include <chrono>
#include <cmath>
#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

class AbsoluteMotionNode : public rclcpp::Node
{
public:
    AbsoluteMotionNode() : Node("absolute_motion_node"), phi_(0.0)
    {
        pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("wheel_speed", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&AbsoluteMotionNode::odomCallback, this, _1));
        timer_ = this->create_wall_timer(10ms, std::bind(&AbsoluteMotionNode::motionLoop, this));

        // Set motion sequence (phi, dx, dy)
        motions_ = {
            {0.0, 1, -1},
            {0.0, 1, 1},
            {0.0, 1, 1},
            {-1.5708, 1, -1},
            {-1.5708, -1, -1},
            {0.0, -1, 1},
            {0.0, -1, 1},
            {0.0, -1, -1}
        };
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::tuple<double, double, double>> motions_;
    size_t current_motion_index_ = 0;
    int iteration_ = 0;

    double phi_;  // yaw angle from odometry
    bool is_pausing_ = false;
    int pause_counter_ = 0;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
            double roll, pitch;
            tf2::Matrix3x3(q).getRPY(roll, pitch, phi_);
            // RCLCPP_INFO(this->get_logger(), "odomCallback: phi_ updated to %.3f", phi_);
    }

    std::tuple<double, double, double> velocity2twist(double dphi, double dx, double dy)
    {
        double R[3][3] = {
            {1.0, 0.0, 0.0},
            {0.0, std::cos(phi_), std::sin(phi_)},
            {0.0, -std::sin(phi_), std::cos(phi_)}
        };
        double v[3] = {dphi, dx, dy};
        double twist[3];

        for (int i = 0; i < 3; ++i)
        {
            twist[i] = R[i][0] * v[0] + R[i][1] * v[1] + R[i][2] * v[2];
        }
        return {twist[0], twist[1], twist[2]};
    }

    std::vector<float> twist2wheels(double wz, double vx, double vy)
    {
        double r = 0.05;
        double l = 0.085;
        double w = 0.134845;

        double H[4][3] = {
            {-l - w, 1, -1},
            { l + w, 1,  1},
            { l + w, 1, -1},
            {-l - w, 1,  1}
        };

        std::vector<float> u(4, 0.0);
        for (int i = 0; i < 4; ++i)
        {
            u[i] = (H[i][0] * wz + H[i][1] * vx + H[i][2] * vy) / r;
        }
        return u;
    }

    void motionLoop()
    {
        static rclcpp::Rate rate(100);  // 0.01 sec = 100 Hz

        // All motions complete
        if (current_motion_index_ >= motions_.size())
        {
            auto msg = std_msgs::msg::Float32MultiArray();
            msg.data = {0.0, 0.0, 0.0, 0.0};
            pub_->publish(msg);
            timer_->cancel();  // stop the loop
            rclcpp::shutdown();      // shut down ROS 2 node cleanly
            return;
        }

        // Handle 2-second pause (200 x 0.01s)
        if (is_pausing_)
        {
            std_msgs::msg::Float32MultiArray msg;
            msg.data = {0.0, 0.0, 0.0, 0.0};
            pub_->publish(msg);

            pause_counter_++;
            if (pause_counter_ >= 200)
            {
                is_pausing_ = false;
                pause_counter_ = 0;
                current_motion_index_++;
            }
            return;
        }

        // Run motion step
        if (iteration_ >= 265)
        {
            iteration_ = 0;
            is_pausing_ = true;  // start 2-second stop after this motion
            return;
        }

        double dphi, dx, dy;
        std::tie(dphi, dx, dy) = motions_[current_motion_index_];
        double wz, vx, vy;
        std::tie(wz, vx, vy) = velocity2twist(dphi, dx, dy);
        auto wheel_speeds = twist2wheels(wz*0.8, vx, vy);

        std_msgs::msg::Float32MultiArray msg;
        msg.data = wheel_speeds;
        pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Step %ld | phi: %.3f", current_motion_index_, phi_);

        iteration_++;
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AbsoluteMotionNode>());
    rclcpp::shutdown();
    return 0;
}
