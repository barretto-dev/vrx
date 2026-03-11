#pragma once

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ros_gz_interfaces/msg/param_vec.hpp"
#include "rcl_interfaces/msg/parameter.hpp"

class WildlifeInterface
{
public:
  explicit WildlifeInterface(rclcpp::Node * node);

  void publish_thrusters(double left, double right);
  void publish_thruster_angles(double left_angle, double right_angle);
  void stop_vehicle();

  bool have_all_data() const;

  // getters
  double boat_lat() const { return boat_lat_; }
  double boat_lon() const { return boat_lon_; }
  double boat_x() const { return boat_x_; }
  double boat_y() const { return boat_y_; }
  double yaw() const { return yaw_; }

  double origin_lat() const { return origin_lat_; }
  double origin_lon() const { return origin_lon_; }

  double crocodile_lat() const { return crocodile_lat_; }
  double crocodile_lon() const { return crocodile_lon_; }

  double platypus_lat() const { return platypus_lat_; }
  double platypus_lon() const { return platypus_lon_; }

  double turtle_lat() const { return turtle_lat_; }
  double turtle_lon() const { return turtle_lon_; }

  bool task_info_seen_once() const { return task_info_seen_once_; }
  double last_score() const { return last_score_; }
  std::chrono::steady_clock::time_point last_task_info_wall_time() const
  {
    return last_task_info_wall_time_;
  }

private:
  rclcpp::Node * node_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_thrust_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_thrust_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_pos_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_pos_pub_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr crocodile_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr platypus_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr turtle_sub_;
  rclcpp::Subscription<ros_gz_interfaces::msg::ParamVec>::SharedPtr task_info_sub_;

  double boat_lat_{0.0};
  double boat_lon_{0.0};
  double boat_x_{0.0};
  double boat_y_{0.0};
  double yaw_{0.0};

  double origin_lat_{0.0};
  double origin_lon_{0.0};

  double crocodile_lat_{0.0};
  double crocodile_lon_{0.0};

  double platypus_lat_{0.0};
  double platypus_lon_{0.0};

  double turtle_lat_{0.0};
  double turtle_lon_{0.0};

  bool have_gps_{false};
  bool have_imu_{false};
  bool have_origin_{false};
  bool have_crocodile_{false};
  bool have_platypus_{false};
  bool have_turtle_{false};

  bool task_info_seen_once_{false};
  double last_score_{0.0};
  std::chrono::steady_clock::time_point last_task_info_wall_time_;

  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void crocodile_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void platypus_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void turtle_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void task_info_callback(const ros_gz_interfaces::msg::ParamVec::SharedPtr msg);
};