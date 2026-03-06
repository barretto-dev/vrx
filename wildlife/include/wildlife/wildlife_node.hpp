#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>

#include "wildlife/pid.hpp"
#include "wildlife/geo.hpp"

class Wildlife : public rclcpp::Node
{
public:
  Wildlife();
  ~Wildlife() override;

  void cancel_timer();
  void stop_vehicle();


private:
  enum class MissionState
  {
    WAIT_DATA,
    CHOOSE_TARGET,
    APPROACH_PLATYPUS,
    ORBIT_PLATYPUS_CW,
    APPROACH_TURTLE,
    ORBIT_TURTLE_CCW,
    DONE
  };

  enum class OrbitDirection
  {
    CW,
    CCW
  };

  void control_step();
  bool have_all_data();
  void reset_controllers();

  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void crocodile_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void platypus_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void turtle_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  void publish_thrusters(double left, double right);
  void publish_cmd(double forward_cmd, double yaw_cmd);
  bool have_all_required_data() const;

  bool approach_point(double gx, double gy, double accept_radius_m, double dt);
  bool approach_orbit_entry(double cx, double cy, double orbit_radius, double dt);
  bool avoid_crocodile(double croc_x, double croc_y, double dt);
  bool orbit_animal(double cx, double cy, double radius, OrbitDirection dir, double dt);

  void compute_orbit_entry_point(double cx, double cy, double radius, double &gx, double &gy);
  void start_orbit(double cx, double cy);
  void update_orbit_accumulator(double cx, double cy);
  bool orbit_completed(OrbitDirection dir) const;

  double distance_xy(double x1, double y1, double x2, double y2);

  void load_parameters();

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_pub_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr crocodile_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr platypus_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr turtle_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_control_time_;

  MissionState state_{MissionState::WAIT_DATA};

  bool platypus_done_{false};
  bool turtle_done_{false};
  bool avoiding_crocodile_{false};

  bool have_gps_{false};
  bool have_imu_{false};
  bool have_yaw_{false};
  bool have_origin_{false};

  bool have_crocodile_{false};
  bool have_platypus_{false};
  bool have_turtle_{false};

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

  PID pid_dist_;
  PID pid_yaw_;

  double orbit_radius_m_{8.0};
  double max_orbit_radius_m_{10.0};
  double entry_tolerance_m_{1.2};
  double crocodile_safe_radius_m_{10.0};
  double crocodile_release_radius_m_{12.0};
  double lookahead_m_{4.0};
  double max_forward_cmd_{900.0};
  double max_turn_cmd_{350.0};
  double base_forward_cmd_{250.0};

  double orbit_center_x_m_{0.0};
  double orbit_center_y_m_{0.0};
  double prev_orbit_angle_rad_{0.0};
  double accumulated_orbit_angle_rad_{0.0};
};