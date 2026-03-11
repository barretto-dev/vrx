#pragma once

#include <utility>
#include <vector>
#include "rclcpp/rclcpp.hpp"

#include "wildlife/pid.hpp"

class WildlifeController
{
public:
  WildlifeController(rclcpp::Node * node);

  void reset_controllers();
  void reset_contour();

  double distance_xy(double x1, double y1, double x2, double y2) const;

  bool avoid_crocodile(
      double boat_x,
      double boat_y,
      double yaw,
      double croc_x,
      double croc_y,
      double dt,
      double & left_thrust,
      double & right_thrust,
      double & left_pos,
      double & right_pos);

  void build_contour_waypoints(
      double boat_x,
      double boat_y,
      double target_x,
      double target_y,
      bool clockwise);

  bool contour_initialized() const { return contour_initialized_; }

  bool step_approach_target(
      double boat_x,
      double boat_y,
      double yaw,
      double target_x,
      double target_y,
      bool clockwise,
      double dt,
      double & left_thrust,
      double & right_thrust,
      double & left_pos,
      double & right_pos);

  bool step_contour_vectored(
      double boat_x,
      double boat_y,
      double yaw,
      double target_x,
      double target_y,
      bool clockwise,
      double dt,
      double & left_thrust,
      double & right_thrust,
      double & left_pos,
      double & right_pos);

private:

  rclcpp::Node * node_;

  PID pid_yaw_;
  PID pid_dist_;

  bool avoiding_crocodile_{false};

  std::vector<std::pair<double,double>> contour_waypoints_;
  std::size_t contour_index_{0};
  bool contour_initialized_{false};

  double crocodile_safe_radius_m_{11.0};
  double crocodile_release_radius_m_{11.5};

  double max_forward_cmd_{600.0};
  double max_turn_cmd_{350.0};
  double base_forward_cmd_{500.0};

  double contour_radius_m_{7.0};
  double contour_forward_cmd_{300.0};
  double waypoint_tolerance_m_{2.0};
  double approach_tolerance_m_{3.0};
  double max_yaw_error_for_full_speed_rad_{0.8};

  double max_thruster_angle_rad_{0.8};
  double orbit_thruster_bias_rad_{0.22};
  double min_contour_forward_cmd_{120.0};
  double contour_radius_gain_{0.10};

  bool go_to_point(
      double boat_x,
      double boat_y,
      double yaw,
      double goal_x,
      double goal_y,
      double dt,
      double forward_base_cmd,
      double tolerance_m,
      double & left_thrust,
      double & right_thrust,
      double & left_pos,
      double & right_pos);
};