#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <ros_gz_interfaces/msg/param_vec.hpp>
#include <rcl_interfaces/msg/parameter.hpp>

#include "wildlife/pid.hpp"
#include "wildlife/geo.hpp"

class Wildlife : public rclcpp::Node
{
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

public:
  Wildlife();
  ~Wildlife() override;

  void cancel_timer();
  void stop_vehicle();

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

  bool have_all_data();
  void reset_controllers();
  void load_parameters();

  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void crocodile_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void platypus_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void turtle_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void task_info_callback(const ros_gz_interfaces::msg::ParamVec::SharedPtr msg);

  void publish_cmd(double forward_cmd, double yaw_cmd);
  bool have_all_required_data() const;

  bool avoid_crocodile(double croc_x, double croc_y, double dt);
  double distance_xy(double x1, double y1, double x2, double y2);

  void build_contour_waypoints(double target_x, double target_y, bool clockwise);
  void reset_contour();
  bool step_contour(double dt);
  bool go_to_point(double goal_x, double goal_y, double dt, double forward_base_cmd, double tolerance_m);
  bool step_approach_target(double target_x, double target_y, bool clockwise, double dt);

  void control_step();
  void check_task_info_alive();

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_thurster_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_thurster_pub_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr crocodile_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr platypus_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr turtle_sub_;

  rclcpp::Subscription<ros_gz_interfaces::msg::ParamVec>::SharedPtr task_info_sub_;
  rclcpp::TimerBase::SharedPtr task_watchdog_timer_;
  std::chrono::steady_clock::time_point last_task_info_wall_time_;

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

  double crocodile_safe_radius_m_{10.5};
  double crocodile_release_radius_m_{11.5};

  double max_forward_cmd_{600.0};
  double max_turn_cmd_{350.0};

  //Empuxo para se aproxima do animal
  double base_forward_cmd_{500.0};

  //Empuxo para contornar animal
  double contour_forward_cmd_{300.0};

  //Array de contorno. Contem pontos os quais serão usados para contorna um animal
  std::vector<std::pair<double,double>> contour_waypoints_;

  //Indica qual elemento no array de contorno está sendo aproximado atualmente
  std::size_t contour_index_ = 0;

  //Indica se o array de contorno foi
  bool contour_initialized_ = false;

  //Raio de trajetoria do contorno
  double contour_radius_m_{7.0};

  double waypoint_tolerance_m_{2.0};
  double approach_tolerance_m_{3.0};

  double max_yaw_error_for_full_speed_rad_{0.8};

  bool task_info_seen_once_{false};
  double last_score_{0.0};

};