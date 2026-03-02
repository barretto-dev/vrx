#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include <chrono>

#include "stationkeeping/pid.hpp"
#include "stationkeeping/geo.hpp"

class Stationkeeping : public rclcpp::Node
{
public:
  Stationkeeping();

  void cancel_timer();
  void send_stop_once();

private:
  void publish(const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr &pub, double v);
  void set_cmd(double l_pos, double r_pos, double l_thr, double r_thr);
  void control_tick();

  // === Params ===
  bool use_sim_time_{true};         // Utilizar tempo da simulação ao invez do real
  double deadband_m_{0.5};          // Distancia minima aceitavel em relação ao objetivo
  double yaw_deadband_deg_{5.0};    // Angulo mínimo de aceitavel para alinhamento

  double thrust_max_{700.0};        // Empuxo máximo permitido
  double thrust_min_{-300.0};       // Empuxo mínimo permitido
  double yaw_mix_max_{400.0};       // Empuxo max/min de rotação
  double thrust_bias_{80.0};        // Empuxo extra para evitar que comandos pequenos não movam o WAMV

  double forward_pos_{0.0};         // Posicionar thurster para avanço reto
  double left_pos_ccw_{+1.0};
  double right_pos_ccw_{-1.0};
  bool use_azimuth_spin_{true};

  // === Freio automático ===
  double brake_radius_m_{12.0};   // ativa freio se dist < isso
  double brake_thrust_{-180.0};   // empuxo negativo (freio/ré)
  int brake_ticks_total_{8};      // duração do freio em ticks
  int brake_ticks_{0};            // contador interno (estado)
  double prev_dist_{1e9};         // para detectar se começou a se afastar

  PID pid_dist_;
  PID pid_yaw_;

  // === State ===
  bool have_gps_{false};
  bool have_yaw_{false};
  bool have_goal_{false};

  double lat_deg_{0.0};
  double lon_deg_{0.0};
  double yaw_rad_{0.0};

  double goal_lat_deg_{0.0};
  double goal_lon_deg_{0.0};

  rclcpp::Time last_time_;

  // === ROS ===
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_l_thrust_, pub_r_thrust_, pub_l_pos_, pub_r_pos_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::TimerBase::SharedPtr timer_;
};