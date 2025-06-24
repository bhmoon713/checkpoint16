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

        // Set motion sequence: [dphi, dx, dy]
        motions_ = {
            {0.0,     1, -1},
            {0.0,     1,  1},
            {0.0,     1,  1},
            {-1.5708, 1, -1},
            {-1.5708, -1, -1},
            {0.0,    -1,  1},
            {0.0,    -1,  1},
            {0.0,    -1, -1}
        };
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::tuple<double, double, double>> motions_;
    size_t current_motion_index_ = 0;

    double phi_ = 0.0;
    double current_x_ = 0.0;
    double current_y_ = 0.0;

    double start_x_ = 0.0;
    double start_y_ = 0.0;
    double start_phi_ = 0.0;
    bool start_pose_set_ = false;

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
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
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
        if (current_motion_index_ >= motions_.size())
        {
            std_msgs::msg::Float32MultiArray stop_msg;
            stop_msg.data = {0.0, 0.0, 0.0, 0.0};
            pub_->publish(stop_msg);
            timer_->cancel();
            rclcpp::shutdown();
            return;
        }

        if (is_pausing_)
        {
            std_msgs::msg::Float32MultiArray stop_msg;
            stop_msg.data = {0.0, 0.0, 0.0, 0.0};
            pub_->publish(stop_msg);

            pause_counter_++;
            if (pause_counter_ >= 200) // ~2s pause
            {
                is_pausing_ = false;
                pause_counter_ = 0;
                current_motion_index_++;
                start_pose_set_ = false;
            }
            return;
        }


        if (!start_pose_set_)
        {
            start_x_ = current_x_;
            start_y_ = current_y_;
            start_phi_ = phi_;
            start_pose_set_ = true;
        }

        double dphi, dx, dy;
        std::tie(dphi, dx, dy) = motions_[current_motion_index_];
        double target_x = start_x_ + dx;
        double target_y = start_y_ + dy;
        double target_phi = start_phi_ + dphi;

        double dx_err = target_x - current_x_;
        double dy_err = target_y - current_y_;
        double dist_error = std::sqrt(dx_err * dx_err + dy_err * dy_err);

        double angle_error = target_phi - phi_;
        while (angle_error > M_PI) angle_error -= 2 * M_PI;
        while (angle_error < -M_PI) angle_error += 2 * M_PI;
        angle_error = std::fabs(angle_error);

        if (dist_error <= 0.02 && angle_error <= 0.00052)
        {
            is_pausing_ = true;
            start_pose_set_ = false;

            std_msgs::msg::Float32MultiArray stop_msg;
            stop_msg.data = {0.0, 0.0, 0.0, 0.0};
            pub_->publish(stop_msg);
            return;
        }

        double wz, vx, vy;
        std::tie(wz, vx, vy) = velocity2twist(dphi, dx, dy);
        RCLCPP_INFO(this->get_logger(),
        "cmd_vel -> wz: %.3f, vx: %.3f, vy: %.3f", wz, vx, vy);
        auto wheel_speeds = twist2wheels(wz * 0.8, vx, vy);

        std_msgs::msg::Float32MultiArray msg;
        msg.data = wheel_speeds;
        pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Motion %ld | dist err: %.4f | angle err: %.6f",
                    current_motion_index_, dist_error, angle_error);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AbsoluteMotionNode>());
    rclcpp::shutdown();
    return 0;
}
