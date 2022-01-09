#include <string.h>
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ypspur.h>

#include <cmath>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class YpspurROS2Bridge : public rclcpp::Node
{
public:
  YpspurROS2Bridge();
  ~YpspurROS2Bridge(){};

private:
  void CmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr cmd_vel);
  void timerCallback();
  void getParams();
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;


private:
  std::string param_file_;
  std::string serial_port_;

  double linear_vel_max_;
  double angular_vel_max_;
  double linear_acc_max_;
  double angular_acc_max_;

  double x_;
  double y_;
  double th_;

  double vx_;
  double vy_;
  double vth_;

  double l_ang_;
  double r_ang_;

  double pub_hz_;
  sensor_msgs::msg::JointState js_;

  std::string left_wheel_joint_;
  std::string right_wheel_joint_;

  std::string frame_id_;
  std::string child_frame_id_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
  rclcpp::Time current_time_;
  rclcpp::TimerBase::SharedPtr timer_control_;

  tf2_ros::TransformBroadcaster tf2_broadcaster_;
};
