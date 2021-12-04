#include "ypspur_ros2_bridge/ypspur_ros2_bridge_core.hpp"

YpspurROS2Bridge::YpspurROS2Bridge()
    : rclcpp::Node("ypspur_ros2_bridge"), tf2_broadcaster_(*this)
{
  if (Spur_init() < 0)
    RCLCPP_ERROR(this->get_logger(), "can't open ypspur");

  linear_vel_max_ = this->declare_parameter<double>("linear_vel_max", 1.1);
  angular_vel_max_ = this->declare_parameter<double>("angular_vel_max", M_PI);
  linear_acc_max_ = this->declare_parameter<double>("linear_acc_max", 1.0);
  angular_acc_max_ = this->declare_parameter<double>("angular_acc_max", M_PI);
  pub_hz_ = this->declare_parameter<double>("pub_hz_", 25.0);

  Spur_set_vel(linear_vel_max_);
  Spur_set_accel(linear_acc_max_);
  Spur_set_angvel(angular_vel_max_);
  Spur_set_angaccel(angular_acc_max_);

  double rate = 1.0 / pub_hz_;
  auto timer_control_callback = std::bind(&YpspurROS2Bridge::timerCallback, this);
  auto period_control =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(rate));
  timer_control_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_control_callback)>>(
      get_clock(), period_control, std::move(timer_control_callback),
      this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_control_, nullptr);

  left_wheel_joint_ = this->declare_parameter<std::string>("left_wheel_joint", "left_wheel_joint");
  right_wheel_joint_ = this->declare_parameter<std::string>("right_wheel_joint", "right_wheel_joint");
  frame_id_ = this->declare_parameter<std::string>("frame_id", "odom");
  child_frame_id_ = this->declare_parameter<std::string>("child_frame_id", "base_link");

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1,
                                                                      std::bind(&YpspurROS2Bridge::CmdVelCallback, this, std::placeholders::_1));
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS{10});
  twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("twist", rclcpp::QoS{10});
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", rclcpp::QoS{10});
  js_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::QoS{10});
  odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(
      std::shared_ptr<rclcpp::Node>(this, [](auto) {}));

  js_.name.push_back(left_wheel_joint_);
  js_.name.push_back(right_wheel_joint_);
  js_.position.resize(2);
}

YpspurROS2Bridge::~YpspurROS2Bridge()
{
  Spur_stop();
  Spur_free();
  RCLCPP_INFO(this->get_logger(), "Stop the robot");
}

void YpspurROS2Bridge::CmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr cmd_vel)
{
  Spur_vel(cmd_vel->linear.x, cmd_vel->angular.z);
}

void YpspurROS2Bridge::timerCallback()
{
  geometry_msgs::msg::Quaternion odom_quat;
  geometry_msgs::msg::TransformStamped odom_trans;
  nav_msgs::msg::Odometry odom;
  geometry_msgs::msg::TwistStamped twist;
  geometry_msgs::msg::PoseStamped pose;
  rclcpp::Clock ros_clock(RCL_ROS_TIME);

  Spur_get_pos_GL(&x_, &y_, &th_);

  Spur_get_vel(&vx_, &vth_);

  YP_get_wheel_ang(&l_ang_, &r_ang_);

  tf2::Quaternion yp_quat;
  yp_quat.setRPY(0, 0, th_);
  odom_quat = tf2::toMsg(yp_quat);

  odom_trans.header.stamp = ros_clock.now();
  odom_trans.header.frame_id = frame_id_;
  odom_trans.child_frame_id = child_frame_id_;
  odom_trans.transform.translation.x = x_;
  odom_trans.transform.translation.y = y_;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  tf2_broadcaster_.sendTransform(odom_trans);

  odom.header.stamp = ros_clock.now();
  odom.header.frame_id = frame_id_;
  odom.child_frame_id = child_frame_id_;
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  odom.twist.twist.linear.x = vx_;
  odom.twist.twist.linear.y = vy_;
  odom.twist.twist.angular.z = vth_;

  odom_pub_->publish(odom);

  twist.header.stamp = ros_clock.now();
  twist.header.frame_id = child_frame_id_;
  twist.twist.linear.x = vx_;
  twist.twist.linear.y = vy_;
  twist.twist.angular.z = vth_;
  twist_pub_->publish(twist);

  pose.header.stamp = ros_clock.now();
  pose.header.frame_id = frame_id_;
  pose.pose.position = odom.pose.pose.position;
  pose.pose.orientation = odom.pose.pose.orientation;
  pose_pub_->publish(pose);

  js_.header.stamp = ros_clock.now();
  js_.name.push_back(left_wheel_joint_);
  js_.name.push_back(right_wheel_joint_);
  js_.position.push_back(l_ang_);
  js_.position.push_back(r_ang_);
  js_pub_->publish(js_);
}
