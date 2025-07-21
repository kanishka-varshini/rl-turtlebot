// ttb4_sim_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class Simple2DSimulator : public rclcpp::Node {
public:
    Simple2DSimulator() : Node("ttb4_sim_cpp") {
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&Simple2DSimulator::cmd_callback, this, std::placeholders::_1)
        );

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/robot_state", 10);

        timer_ = this->create_wall_timer(
            33ms, std::bind(&Simple2DSimulator::update, this));

        x_ = 250.0;
        y_ = 250.0;
        theta_ = 0.0;
    }

private:
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        cmd_ = *msg;
    }

    void update() {
        double dt = 0.033; // 33 ms
        double speed = 50.0;
        double rot_speed = 2.0;

        double v = cmd_.linear.x * speed * dt;
        double w = cmd_.angular.z * rot_speed * dt;

        theta_ += w;
        x_ += v * std::cos(theta_);
        y_ += v * std::sin(theta_);

        x_ = std::max(10.0, std::min(490.0, x_));
        y_ = std::max(10.0, std::min(490.0, y_));

        auto odom = nav_msgs::msg::Odometry();
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.orientation.z = std::sin(theta_ / 2.0);
        odom.pose.pose.orientation.w = std::cos(theta_ / 2.0);
        odom_pub_->publish(odom);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Twist cmd_;
    double x_, y_, theta_;
};