#include "wildlife/wildlife_node2.hpp"

#include <chrono>
#include <cmath>

#include "wildlife/geo.hpp"

using namespace std::chrono_literals;

Wildlife::Wildlife()
: Node("wildlife_node")
{
  interface_ = std::make_unique<WildlifeInterface>(this);
  controller_ = std::make_unique<WildlifeController>(this);

  control_timer_ = this->create_wall_timer(
      50ms, std::bind(&Wildlife::control_step, this));

  watchdog_timer_ = this->create_wall_timer(
      500ms, std::bind(&Wildlife::check_task_info_alive, this));
}

void Wildlife::shutdown_with_score(const std::string & reason)
{
  RCLCPP_WARN(
      get_logger(),
      "Encerrando nó. Motivo: %s | Último score = %.3f",
      reason.c_str(),
      interface_->last_score());

  interface_->stop_vehicle();
  rclcpp::shutdown();
}

void Wildlife::check_task_info_alive()
{
  if (!interface_->task_info_seen_once()) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - interface_->last_task_info_wall_time());

  if (elapsed > std::chrono::milliseconds(3000))
  {
    shutdown_with_score("timeout em /vrx/task/info");
  }
}

void Wildlife::control_step()
{
  if (!interface_->have_all_data())
  {
    interface_->stop_vehicle();
    state_ = MissionState::WAIT_DATA;
    return;
  }

  const double dt = 0.05;

  double croc_x, croc_y;
  double plat_x, plat_y;
  double turtle_x, turtle_y;

  geo_utils::latlon_to_local_xy(
      interface_->origin_lat(), interface_->origin_lon(),
      interface_->crocodile_lat(), interface_->crocodile_lon(),
      croc_x, croc_y);

  geo_utils::latlon_to_local_xy(
      interface_->origin_lat(), interface_->origin_lon(),
      interface_->platypus_lat(), interface_->platypus_lon(),
      plat_x, plat_y);

  geo_utils::latlon_to_local_xy(
      interface_->origin_lat(), interface_->origin_lon(),
      interface_->turtle_lat(), interface_->turtle_lon(),
      turtle_x, turtle_y);

  double left_thrust = 0.0, right_thrust = 0.0;
  double left_pos = 0.0, right_pos = 0.0;

  if (controller_->avoid_crocodile(
          interface_->boat_x(), interface_->boat_y(), interface_->yaw(),
          croc_x, croc_y, dt,
          left_thrust, right_thrust, left_pos, right_pos))
  {
    RCLCPP_INFO(get_logger(), "Avoiding crocodile");
    interface_->publish_thruster_angles(left_pos, right_pos);
    interface_->publish_thrusters(left_thrust, right_thrust);
    return;
  }

  const std::string state_name =  mission_state_to_string(state_);
  RCLCPP_INFO(get_logger(), "Current state: %s", state_name.c_str());

  switch (state_)
  {
    case MissionState::WAIT_DATA:
      controller_->reset_controllers();
      controller_->reset_contour();
      state_ = MissionState::CHOOSE_TARGET;
      break;

    case MissionState::CHOOSE_TARGET:
    {
      controller_->reset_controllers();
      controller_->reset_contour();

      if (!platypus_done_ && !turtle_done_)
      {
        const double d1 = controller_->distance_xy(
            interface_->boat_x(), interface_->boat_y(), plat_x, plat_y);
        const double d2 = controller_->distance_xy(
            interface_->boat_x(), interface_->boat_y(), turtle_x, turtle_y);

        state_ = (d1 < d2)
            ? MissionState::APPROACH_PLATYPUS
            : MissionState::APPROACH_TURTLE;
      }
      else if (!platypus_done_)
      {
        state_ = MissionState::APPROACH_PLATYPUS;
      }
      else if (!turtle_done_)
      {
        state_ = MissionState::APPROACH_TURTLE;
      }
      else
      {
        state_ = MissionState::DONE;
      }
      break;
    }

    case MissionState::APPROACH_PLATYPUS:
    {
      const bool reached = controller_->step_approach_target(
          interface_->boat_x(), interface_->boat_y(), interface_->yaw(),
          plat_x, plat_y, true, dt,
          left_thrust, right_thrust, left_pos, right_pos);

      interface_->publish_thruster_angles(left_pos, right_pos);
      interface_->publish_thrusters(left_thrust, right_thrust);

      if (reached)
      {
        controller_->reset_controllers();
        state_ = MissionState::ORBIT_PLATYPUS_CW;
      }
      break;
    }

    case MissionState::ORBIT_PLATYPUS_CW:
    {
      const bool finished = controller_->step_contour_vectored(
          interface_->boat_x(), interface_->boat_y(), interface_->yaw(),
          plat_x, plat_y, true, dt,
          left_thrust, right_thrust, left_pos, right_pos);

      interface_->publish_thruster_angles(left_pos, right_pos);
      interface_->publish_thrusters(left_thrust, right_thrust);

      if (finished)
      {
        platypus_done_ = true;
        controller_->reset_controllers();
        controller_->reset_contour();
        state_ = MissionState::CHOOSE_TARGET;
      }
      break;
    }

    case MissionState::APPROACH_TURTLE:
    {
      const bool reached = controller_->step_approach_target(
          interface_->boat_x(), interface_->boat_y(), interface_->yaw(),
          turtle_x, turtle_y, false, dt,
          left_thrust, right_thrust, left_pos, right_pos);

      interface_->publish_thruster_angles(left_pos, right_pos);
      interface_->publish_thrusters(left_thrust, right_thrust);

      if (reached)
      {
        controller_->reset_controllers();
        state_ = MissionState::ORBIT_TURTLE_CCW;
      }
      break;
    }

    case MissionState::ORBIT_TURTLE_CCW:
    {
      const bool finished = controller_->step_contour_vectored(
          interface_->boat_x(), interface_->boat_y(), interface_->yaw(),
          turtle_x, turtle_y, false, dt,
          left_thrust, right_thrust, left_pos, right_pos);

      interface_->publish_thruster_angles(left_pos, right_pos);
      interface_->publish_thrusters(left_thrust, right_thrust);

      if (finished)
      {
        turtle_done_ = true;
        controller_->reset_controllers();
        controller_->reset_contour();
        state_ = MissionState::CHOOSE_TARGET;
      }
      break;
    }

    case MissionState::DONE:
      interface_->stop_vehicle();
      break;
  }
}