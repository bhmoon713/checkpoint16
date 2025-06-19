#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using std::placeholders::_1;

class AbsoluteMotion : public rclcpp::Node
{
public:
    AbsoluteMotion() : Node("absolute_motion"), phi_(0.0)
    {
        pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("wheel_speed", 10);
        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&AbsoluteMotion::odom_callback, this, _1));

        // Wait a moment for subscription to register
        rclcpp::sleep_for(std::chrono::seconds(1));

        // Motions: forward, left, backward, right
        std::vector<std::pair<int, int>> motions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};

        rclcpp::Rate loop_rate(100);  // 0.01s per iteration

        for (const auto& motion : motions)
        {
            double dx = motion.first;
            double dy = motion.second;
            double dphi = M_PI / 6.0;  // 30 degrees/sec

            for (int i = 0; i < 300; ++i)
            {
                double wz, vx, vy;
                velocity2twist(dphi, dx, dy, wz, vx, vy);
                auto wheel_speeds = twist2wheels(wz, vx, vy);

                std_msgs::msg::Float32MultiArray msg;
                msg.data = wheel_speeds;
                pub_->publish(msg);
                loop_rate.sleep();
            }
        }

        // Stop at end
        std_msgs::msg::Float32MultiArray stop_msg;
        stop_msg.data = {0.0, 0.0, 0.0, 0.0};
        pub_->publish(stop_msg);
        rclcpp::sleep_for(std::chrono::seconds(3));
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    double phi_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const auto& q = msg->pose.pose.orientation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(quat);
        double roll, pitch;
        m.getRPY(roll, pitch, phi_);
    }

    void velocity2twist(double dphi, double dx, double dy,
                        double& wz, double& vx, double& vy)
    {
        double cos_phi = std::cos(phi_);
        double sin_phi = std::sin(phi_);

        wz = dphi;
        vx = cos_phi * dx + sin_phi * dy;
        vy = -sin_phi * dx + cos_phi * dy;
    }

    std::vector<float> twist2wheels(double wz, double vx, double vy)
    {
        double r = 0.05;
        double l = 0.085;
        double w = 0.134845;

        std::vector<float> u(4);
        u[0] = ((-l - w) * wz + vx - vy) / r;
        u[1] = (( l + w) * wz + vx + vy) / r;
        u[2] = (( l + w) * wz + vx - vy) / r;
        u[3] = ((-l - w) * wz + vx + vy) / r;

        return u;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AbsoluteMotion>());
    rclcpp::shutdown();
    return 0;
}
