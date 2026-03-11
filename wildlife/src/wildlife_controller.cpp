#include "wildlife/wildlife_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "wildlife/geo.hpp"

WildlifeController::WildlifeController(rclcpp::Node * node): node_(node)
{
  pid_yaw_.kp = 500.0;
  pid_yaw_.ki = 0.0;
  pid_yaw_.kd = 100.0;

  pid_dist_.kp = 180.0;
  pid_dist_.ki = 0.0;
  pid_dist_.kd = 100.0;
}

void WildlifeController::reset_controllers()
{
  pid_dist_.reset();
  pid_yaw_.reset();
}

void WildlifeController::reset_contour()
{
  contour_waypoints_.clear();
  contour_index_ = 0;
  contour_initialized_ = false;
}

double WildlifeController::distance_xy(double x1, double y1, double x2, double y2) const
{
  return std::hypot(x2 - x1, y2 - y1);
}

bool WildlifeController::go_to_point(
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
    double & right_pos)
{
  const double dx = goal_x - boat_x;
  const double dy = goal_y - boat_y;
  const double dist = std::hypot(dx, dy);

  if (dist <= tolerance_m)
  {
    left_thrust = 0.0;
    right_thrust = 0.0;
    left_pos = 0.0;
    right_pos = 0.0;
    reset_controllers();
    return true;
  }

  const double desired_yaw = std::atan2(dy, dx);
  const double yaw_error = geo_utils::wrap_pi(desired_yaw - yaw);

  double yaw_cmd = pid_yaw_.step(yaw_error, dt);
  yaw_cmd = std::clamp(yaw_cmd, -max_turn_cmd_, max_turn_cmd_);

  double forward_cmd = forward_base_cmd;

  if (std::abs(yaw_error) > max_yaw_error_for_full_speed_rad_ || dist < 4.0)
  {
    forward_cmd *= 0.4;
  }


  double left = forward_cmd - yaw_cmd;
  double right = forward_cmd + yaw_cmd;

  left_thrust = std::clamp(left, -max_forward_cmd_, max_forward_cmd_);
  right_thrust = std::clamp(right, -max_forward_cmd_, max_forward_cmd_);

  left_pos = 0.0;
  right_pos = 0.0;

  return false;
}

bool WildlifeController::avoid_crocodile(
    double boat_x,
    double boat_y,
    double yaw,
    double croc_x,
    double croc_y,
    double dt,
    double & left_thrust,
    double & right_thrust,
    double & left_pos,
    double & right_pos)
{
  const double dx = boat_x - croc_x;
  const double dy = boat_y - croc_y;
  const double dist = std::hypot(dx, dy);

  if (!avoiding_crocodile_)
  {
    if (dist >= crocodile_safe_radius_m_)
      return false;

    avoiding_crocodile_ = true;
    reset_controllers();
    RCLCPP_WARN(node_->get_logger(), "Too close to crocodile (%.2f m). Starting avoidance.", dist);
  }
  else
  {
    if (dist >= crocodile_release_radius_m_)
    {
      avoiding_crocodile_ = false;
      reset_controllers();
      RCLCPP_INFO(node_->get_logger(), "Crocodile avoidance cleared (%.2f m).", dist);
      return false;
    }
  }

  const double desired_yaw = std::atan2(dy, dx);
  const double yaw_error = geo_utils::wrap_pi(desired_yaw - yaw);

  double yaw_cmd = pid_yaw_.step(yaw_error, dt);
  yaw_cmd = std::clamp(yaw_cmd, -max_turn_cmd_, max_turn_cmd_);

  double forward_cmd = base_forward_cmd_;

  if (std::abs(yaw_error) > 1.0)
    forward_cmd *= 0.35;
  
  double left = forward_cmd - yaw_cmd;
  double right = forward_cmd + yaw_cmd;

  left_thrust = std::clamp(left, -max_forward_cmd_, max_forward_cmd_);
  right_thrust = std::clamp(right, -max_forward_cmd_, max_forward_cmd_);

  left_pos = 0.0;
  right_pos = 0.0;

  return true;
}

void WildlifeController::build_contour_waypoints(
    double boat_x,
    double boat_y,
    double target_x,
    double target_y,
    bool clockwise)
{
  contour_waypoints_.clear();
  contour_index_ = 0;
  contour_initialized_ = true;

  const double r = contour_radius_m_;

  const std::pair<double,double> east  = {target_x + r, target_y};
  const std::pair<double,double> north = {target_x, target_y + r};
  const std::pair<double,double> west  = {target_x - r, target_y};
  const std::pair<double,double> south = {target_x, target_y - r};

  std::vector<std::pair<double,double>> base_points;

  if (clockwise)
    base_points = {east, south, west, north};
  else
    base_points = {east, north, west, south};

  // Encontrar o ponto mais próximo da posição atual do barco
  std::size_t start_idx = 0;
  double best_dist = std::numeric_limits<double>::max();

  for (std::size_t i = 0; i < base_points.size(); ++i)
  {
    const double d = distance_xy(
      boat_x, boat_y,
      base_points[i].first, base_points[i].second
    );

    if (d < best_dist)
    {
      best_dist = d;
      start_idx = i;
    }
  }

  // Montar a sequência começando do ponto mais próximo,
  // mantendo o sentido de contorno
  for (std::size_t k = 0; k < base_points.size(); ++k)
  {
    const std::size_t idx = (start_idx + k) % base_points.size();
    contour_waypoints_.push_back(base_points[idx]);
  }

  // Repetir o primeiro no final para "fechar a volta"
  contour_waypoints_.push_back(contour_waypoints_.front());

  RCLCPP_INFO(
    node_->get_logger(),
    "Contour built. clockwise=%s start_idx=%zu closest_dist=%.2f first_wp=(%.2f, %.2f)",
    clockwise ? "true" : "false",
    start_idx,
    best_dist,
    contour_waypoints_.front().first,
    contour_waypoints_.front().second
  );

}

bool WildlifeController::step_approach_target(
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
    double & right_pos)
{
  if (!contour_initialized_)
    build_contour_waypoints(boat_x, boat_y, target_x, target_y, clockwise);
  
  if (contour_waypoints_.empty())
    return false;

  const auto & first_wp = contour_waypoints_.front();

  return go_to_point(
      boat_x, boat_y, yaw,
      first_wp.first, first_wp.second,
      dt,
      contour_forward_cmd_,
      approach_tolerance_m_,
      left_thrust, right_thrust, left_pos, right_pos);
}

bool WildlifeController::step_contour_vectored(
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
    double & right_pos)
{
  if (!contour_initialized_ || contour_waypoints_.empty())
    return false;

  if (contour_index_ >= contour_waypoints_.size())
  {
    //stop vehicle
    left_thrust = 0.0;
    right_thrust = 0.0;
    left_pos = 0.0;
    right_pos = 0.0;
    
    reset_controllers();
    return true;
  }

  const auto & wp = contour_waypoints_[contour_index_];
  //const double dist_wp = std::hypot(wp.first - boat_x, wp.second - boat_y);

  const bool reached = go_to_point(
    boat_x, boat_y, yaw,
    wp.first, wp.second,
    dt,
    contour_forward_cmd_,
    approach_tolerance_m_,
    left_thrust, right_thrust, left_pos, right_pos
  );

  if (reached)
  {
    contour_index_++;

    if (contour_index_ >= contour_waypoints_.size())
    {
        //stop vehicle
        left_thrust = 0.0;
        right_thrust = 0.0;
        left_pos = 0.0;
        right_pos = 0.0;

        reset_controllers();
      return true;
    }
  }

  return false;
}