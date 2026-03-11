#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "wildlife/wildlife_interface.hpp"
#include "wildlife/wildlife_controller.hpp"

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

class Wildlife : public rclcpp::Node
{
public:
  Wildlife();

  std::string mission_state_to_string(MissionState state) {
    switch (state) {
        case MissionState::WAIT_DATA:
            return "WAIT_DATA";
        case MissionState::CHOOSE_TARGET:
            return "CHOOSE_TARGET";
        case MissionState::APPROACH_PLATYPUS:
            return "APPROACH_PLATYPUS";
          case MissionState::ORBIT_PLATYPUS_CW:
            return "ORBIT_PLATYPUS_CW";
        case MissionState::APPROACH_TURTLE:
            return "APPROACH_TURTLE";
        case MissionState::ORBIT_TURTLE_CCW:
            return "ORBIT_TURTLE_CCW";
        case MissionState::DONE:
            return "DONE";
        default:
            return "Unknown"; // Handle unexpected values
    }
  }

private:
  std::unique_ptr<WildlifeInterface> interface_;
  std::unique_ptr<WildlifeController> controller_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  MissionState state_{MissionState::WAIT_DATA};
  bool platypus_done_{false};
  bool turtle_done_{false};

  void control_step();
  void check_task_info_alive();
  void shutdown_with_score(const std::string & reason);
};