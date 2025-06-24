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


    double start_x_ = 0.0;
    double start_y_ = 0.0;
    double start_phi_ = 0.0;
    double current_phi_ = 0.0;
    bool start_pose_set_ = false;

    double phi_;  // yaw angle from odometry
    double current_x_ = 0.0;
    double current_y_ = 0.0;
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
        current_phi_ = phi_;
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

        // All motions complete and shutdown
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
            if (pause_counter_ >= 100)
            {
                is_pausing_ = false;
                pause_counter_ = 0;
                current_motion_index_++;
            }
            return;
        }

        // Save start pose for this motion if not already set
        if (!start_pose_set_) {
            start_x_ = current_x_;
            start_y_ = current_y_;
            start_phi_ = phi_;
            start_pose_set_ = true;
        }
        double dphi, dx, dy;
        std::tie(dphi, dx, dy) = motions_[current_motion_index_];
        double wz, vx, vy;
        std::tie(wz, vx, vy) = velocity2twist(dphi*0.25, dx*0.25, dy*0.25);  //reduce speed 1/4

        double target_x = start_x_ + dx;
        double target_y = start_y_ + dy;
        double target_phi = start_phi_ + dphi;

        double dx_err = target_x - current_x_;
        double dy_err = target_y - current_y_;
        double angle_error = std::fabs(target_phi - phi_);

        // Normalize angle error to [-pi, pi]
        while (angle_error > M_PI) angle_error -= 2 * M_PI;
        while (angle_error < -M_PI) angle_error += 2 * M_PI;
        angle_error = std::fabs(angle_error);

        // ✅ NEW STOP CONDITION BASED ON dx and dy errors
        // === Precise stop condition ===
        if (std::fabs(dx_err) < 0.02 && std::fabs(dy_err) < 0.02) {
            RCLCPP_INFO(this->get_logger(),
                "Reached target (x,y) for motion %ld — pausing", current_motion_index_);

            std_msgs::msg::Float32MultiArray stop_msg;
            stop_msg.data = {0.0, 0.0, 0.0, 0.0};
            pub_->publish(stop_msg);

            is_pausing_ = true;
            pause_counter_ = 0;
            start_pose_set_ = false;
            return;
        }

        // === Fine error recovery zone (within 10cm) ===
        if (std::fabs(dx_err) < 0.1 || std::fabs(dy_err) < 0.1) {
            double fine_dx = 0.025 * (dx_err / (std::fabs(dx_err) + 1e-6));
            double fine_dy = 0.025 * (dy_err / (std::fabs(dy_err) + 1e-6));

            double fine_dphi = 0.0;  // default no angle correction

            // Only activate angle correction if angular error is also in fine-tuning range
            if (std::fabs(angle_error) < 0.05) {
                double sign_phi = (target_phi - phi_) >= 0 ? 1.0 : -1.0;
                fine_dphi = 0.01 * sign_phi;
                RCLCPP_INFO(this->get_logger(),
                    "Fine recovery — error_x: %.4f, error_x: %.4f, error_angle: %.4f",dx_err, dy_err, angle_error);
            } else {
                RCLCPP_INFO(this->get_logger(),
                    "Fine XY recovery only — dx: %.4f, dy: %.4f", fine_dx, fine_dy);
            }

            double wz, vx, vy;
            std::tie(wz, vx, vy) = velocity2twist(fine_dphi, fine_dx, fine_dy);
            auto wheel_speeds = twist2wheels(wz, vx, vy);

            std_msgs::msg::Float32MultiArray msg;
            msg.data = wheel_speeds;
            pub_->publish(msg);
            return;
        }


                
        auto wheel_speeds = twist2wheels(wz, vx, vy);


        std_msgs::msg::Float32MultiArray msg;
        msg.data = wheel_speeds;
        pub_->publish(msg);

        // RCLCPP_INFO(this->get_logger(), "Step %ld | phi: %.3f", current_motion_index_, phi_);

        iteration_++;
        
        RCLCPP_INFO(this->get_logger(), "Motion %ld | x_: %.4f | y_: %.6f",
            current_motion_index_, current_x_,current_y_);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AbsoluteMotionNode>());
    rclcpp::shutdown();
    return 0;
}