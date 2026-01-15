#include <array>
#include <memory>

#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

class URDFBroadcaster : public rclcpp::Node {
public:
    URDFBroadcaster()
    : rclcpp::Node("urdf_broadcaster"),
    tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this)) {
        using std::placeholders::_1;

        auto logging_qos = rclcpp::QoS(rclcpp::KeepLast(10));
        logging_qos.best_effort();

        pos_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
          "/record/pos", logging_qos,
          std::bind(&URDFBroadcaster::onPos, this, _1));

        rpy_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
          "/record/rpy", logging_qos,
          std::bind(&URDFBroadcaster::onRpy, this, _1));

        servo_cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
          "/record/rotor_servo_command", logging_qos,
          std::bind(&URDFBroadcaster::onServoCommand, this, _1));

        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
          "/joint_states", rclcpp::QoS(rclcpp::KeepLast(10)));
    }

private:
  void onPos(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
    latest_pos_ = msg->vector;
    has_pos_ = true;
    publishTransform(msg->header.stamp);
  }

  void onRpy(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
    latest_rpy_ = msg->vector;
    has_rpy_ = true;
    publishTransform(msg->header.stamp);
  }

  void onServoCommand(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    constexpr std::array<size_t, 4> indices = {5, 6, 7, 8};
    if (msg->data.size() <= indices.back()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "rotor_servo_command length %zu < required index %zu",
        msg->data.size(), indices.back());
      return;
    }

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = get_clock()->now();
    joint_state.name = {
      "arm_1_revolute_joint",
      "arm_2_revolute_joint",
      "arm_3_revolute_joint",
      "arm_4_revolute_joint"};
    joint_state.position.resize(indices.size());
    for (size_t i = 0; i < indices.size(); ++i) {
      joint_state.position[i] = msg->data[indices[i]];
    }

    joint_state_pub_->publish(joint_state);
  }

  void publishTransform(const rclcpp::Time & stamp) {
    if (!has_pos_ || !has_rpy_) {
      return;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = "world";
    tf_msg.child_frame_id = "base_link";
    tf_msg.transform.translation.x = latest_pos_.y;
    tf_msg.transform.translation.y = latest_pos_.x;
    tf_msg.transform.translation.z = -latest_pos_.z;

    tf2::Matrix3x3 R_world_body;
    R_world_body.setRPY(latest_rpy_.x, latest_rpy_.y, latest_rpy_.z);

    // NED -> ENU axis swap
    tf2::Matrix3x3 axis_swap_1(
      0.0, 1.0, 0.0,
      1.0, 0.0, 0.0,
      0.0, 0.0,-1.0);
      tf2::Matrix3x3 axis_swap_2(
        0.0, 1.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 0.0,-1.0);
    tf2::Matrix3x3 adjusted_rotation = axis_swap_1 * R_world_body * axis_swap_2;

    tf2::Quaternion q_adjusted;
    adjusted_rotation.getRotation(q_adjusted);
    tf_msg.transform.rotation = tf2::toMsg(q_adjusted);

    tf_broadcaster_->sendTransform(tf_msg);
  }

  geometry_msgs::msg::Vector3 latest_pos_;
  geometry_msgs::msg::Vector3 latest_rpy_;
  bool has_pos_{false};
  bool has_rpy_{false};
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr pos_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr rpy_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr servo_cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<URDFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}