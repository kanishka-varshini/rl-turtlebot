// ttb4_sim_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class Simple2DSimulator : public rclcpp::Node {
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Twist cmd_;
    double x_, y_, theta_, v_;

public:
    Simple2DSimulator() : Node("ttb4_sim_cpp") {
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg){this->cmd_callback(msg);}
        );

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/robot_state", 10);

        timer_ = this->create_wall_timer(
            33ms, 
            [this](){this->update();}
        );

        x_ = 250.0;
        y_ = 250.0;
        theta_ = 0.0;
        v_ = 0.0;
    }

private:
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        cmd_ = *msg;
    }

    void update() {
        double dt = 0.033; // 33 ms
        double a;
        float tarv = cmd_.linear.x;

        if(v_<tarv){
            a = 0.9;
        }
        else if(v_>tarv){
            a = -0.9;
        }
        else{
            a = 0.0;
        }

        v_ += a * dt;
        theta_ += cmd_.angular.z * dt;

        x_ += v_ * std::cos(theta_) * dt;
        y_ += v_ * std::sin(theta_) * dt;
                

        nav_msgs::msg::Odometry odom = nav_msgs::msg::Odometry();
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.orientation.z = std::sin(theta_ / 2.0);
        odom.pose.pose.orientation.w = std::cos(theta_ / 2.0);
        odom_pub_->publish(odom);
    }
};