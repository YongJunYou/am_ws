#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <Eigen/Dense>
#include <optional>
#include <mutex>

#include "mani_controller/dynamixel_sdk_interface.hpp"
#include "mani_controller/gravitational_moment_calculator.hpp"

class RobotArm : public rclcpp::Node
{
public:
    RobotArm();
    void run();

    rclcpp::Time last_time_;

    std::unique_ptr<DynamixelSdkInterface> dxl_interface_;

    // PID Controller
    std::unique_ptr<GravitationalMomentCalculator> gravitational_moment_calculator_;

    // ROS2 Publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    
    // ROS2 Subscribers
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr desired_theta_sub_;
    
    // Desired joint positions (theta)
    Eigen::VectorXd desired_theta_;

    // Topic 발행 함수
    void publishJointState(const DynamixelSdkInterface::State &state);
    
    // Subscriber callback
    void desiredThetaCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
};

