#include "stationkeeping/stationkeeping_node.hpp"

#include <thread>
#include <cmath>

using namespace std::chrono_literals;

Stationkeeping::Stationkeeping() : Node("stationkeeping")
{
  // === Parâmetros ===
  //use_sim_time_ = this->declare_parameter<bool>("use_sim_time", true);
  this->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time_));

  deadband_m_ = this->declare_parameter<double>("deadband_m", 0.6);
  yaw_deadband_deg_ = this->declare_parameter<double>("yaw_deadband_deg", 5.0);

  thrust_max_ = this->declare_parameter<double>("thrust_max", 600.0);
  thrust_min_ = this->declare_parameter<double>("thrust_min", -200.0);
  yaw_mix_max_ = this->declare_parameter<double>("yaw_mix_max", 300.0);
  thrust_bias_ = this->declare_parameter<double>("thrust_bias", 80.0);

  forward_pos_ = this->declare_parameter<double>("forward_pos", 0.0);
  left_pos_ccw_  = this->declare_parameter<double>("left_pos_ccw",  +1.5708);
  right_pos_ccw_ = this->declare_parameter<double>("right_pos_ccw", -1.5708);
  use_azimuth_spin_ = this->declare_parameter<bool>("use_azimuth_spin", true);

  brake_radius_m_   = this->declare_parameter<double>("brake_radius_m", 12.0);
  brake_thrust_     = this->declare_parameter<double>("brake_thrust", -220.0);
  brake_ticks_      = this->declare_parameter<double>("brake_ticks", 0.0);
  brake_ticks_total_= this->declare_parameter<int>("brake_ticks_total", 8);

  // PID distância
  pid_dist_.kp = this->declare_parameter<double>("dist_kp", 20.0);
  pid_dist_.ki = this->declare_parameter<double>("dist_ki", 1.0);
  pid_dist_.kd = this->declare_parameter<double>("dist_kd", 1.0);
  pid_dist_.i_min = this->declare_parameter<double>("dist_i_min", -4.0);
  pid_dist_.i_max = this->declare_parameter<double>("dist_i_max",  4.0);
  pid_dist_.out_min = thrust_min_;
  pid_dist_.out_max = thrust_max_;

  // PID yaw
  pid_yaw_.kp = this->declare_parameter<double>("yaw_kp", 250.0);
  pid_yaw_.ki = this->declare_parameter<double>("yaw_ki", 0.0);
  pid_yaw_.kd = this->declare_parameter<double>("yaw_kd", 40.0);
  pid_yaw_.i_min = this->declare_parameter<double>("yaw_i_min", -1.5);
  pid_yaw_.i_max = this->declare_parameter<double>("yaw_i_max",  1.5);
  pid_yaw_.out_min = -yaw_mix_max_;
  pid_yaw_.out_max =  yaw_mix_max_;

  // === Publishers ===
  auto qos_cmd = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  pub_l_thrust_ = create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/left/thrust", qos_cmd);
  pub_r_thrust_ = create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/right/thrust", qos_cmd);
  pub_l_pos_    = create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/left/pos", qos_cmd);
  pub_r_pos_    = create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/right/pos", qos_cmd);

  // === Subs ===
  sub_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    "/wamv/sensors/gps/gps/fix", rclcpp::QoS(10),
    [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
      if (std::isnan(msg->latitude) || std::isnan(msg->longitude)) return;
      lat_deg_ = msg->latitude;
      lon_deg_ = msg->longitude;
      have_gps_ = true;
    });

  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    "/wamv/sensors/imu/imu/data", rclcpp::QoS(50),
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
      const auto &q = msg->orientation;

      //Angulo (rad) de guinada do veículo
      yaw_rad_ = yaw_from_quat(q.x, q.y, q.z, q.w);
      have_yaw_ = true;
    });

  sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/vrx/stationkeeping/goal", rclcpp::QoS(10),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      goal_lat_deg_ = msg->pose.position.x;
      goal_lon_deg_ = msg->pose.position.y;
      have_goal_ = true;

      pid_dist_.reset();
      pid_yaw_.reset();

      RCLCPP_INFO(this->get_logger(),
                  "New goal: lat=%.7f lon=%.7f (PIDs reset)",
                  goal_lat_deg_, goal_lon_deg_);
    });

  last_time_ = this->now();
  timer_ = create_wall_timer(50ms, [this]() { control_tick(); });

  RCLCPP_INFO(this->get_logger(), "PID stationkeeping running (20 Hz).");
}

void Stationkeeping::cancel_timer()
{
  if (timer_) timer_->cancel();
}

void Stationkeeping::publish(const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr &pub, double v)
{
  std_msgs::msg::Float64 m;
  m.data = v;
  pub->publish(m);
}

void Stationkeeping::set_cmd(double l_pos, double r_pos, double l_thr, double r_thr)
{
  publish(pub_l_pos_, l_pos);
  publish(pub_r_pos_, r_pos);
  publish(pub_l_thrust_, l_thr);
  publish(pub_r_thrust_, r_thr);
}

void Stationkeeping::send_stop_once()
{
  publish(pub_l_thrust_, 0.0);
  publish(pub_r_thrust_, 0.0);
  publish(pub_l_pos_, forward_pos_);
  publish(pub_r_pos_, forward_pos_);
}

void Stationkeeping::control_tick()
{ 
  if (!have_gps_ || !have_yaw_ || !have_goal_) return; 
  
  const rclcpp::Time now = this->now(); 
  const double dt = (now - last_time_).seconds(); 
  last_time_ = now; 
  if (dt <= 0.0) return; 

  double east_m = 0.0, north_m = 0.0; 
  latlon_error_m(lat_deg_, lon_deg_, goal_lat_deg_, goal_lon_deg_, east_m, north_m); 
  
  const double dist = std::hypot(east_m, north_m); 
  
  // Detecta overshoot: estava chegando, agora dist aumentou 
  const bool getting_farther = (dist > prev_dist_ + 0.05); 
  
  if (dist < brake_radius_m_ && getting_farther && dist > deadband_m_) {
     brake_ticks_ = brake_ticks_total_; 
  } 
  
  if (brake_ticks_ > 0) { 
    // freia reto (sem mistura de yaw pra não bagunçar) 
    set_cmd(forward_pos_, forward_pos_, brake_thrust_, brake_thrust_); 
    brake_ticks_--; 
    prev_dist_ = dist; 
    return; 
  } 
  
  prev_dist_ = dist; 

  const double desired_yaw = std::atan2(north_m, east_m); 
  double yaw_err = wrap_pi(desired_yaw - yaw_rad_); 
  
  const double yaw_deadband_rad = yaw_deadband_deg_ * M_PI / 180.0; 

  double dist_e = (dist < deadband_m_) ? 0.0 : dist; 
  double yaw_e = (std::fabs(yaw_err) < yaw_deadband_rad) ? 0.0 : yaw_err; 
  
  double u_fwd = pid_dist_.step(dist_e, dt); 
  double u_yaw = pid_yaw_.step(yaw_e, dt); 
  
  // Só aplica bias quando estiver longe 
  const double bias_radius = 8.0; // m (parametrizável) 

  if (u_fwd > 0.0 && dist > bias_radius) { 
    u_fwd += thrust_bias_; 
  } 
  
  double thrust_cap = thrust_max_; 
  double slow_radius_m_ = 15; 
  double thrust_at_slow_ = 220.0; 
  double thrust_at_goal_ = 0.0; 
  
  if (dist < slow_radius_m_) { 
    //rampa linear: dist=slow_radius -> thrust_at_slow, dist=deadband -> thrust_at_goal 
    double alpha = (dist - deadband_m_) / (slow_radius_m_ - deadband_m_); 
    
    // 0..1 alpha = clamp(alpha, 0.0, 1.0); 
    thrust_cap = thrust_at_goal_ + alpha * (thrust_at_slow_ - thrust_at_goal_); 
  } 
  
  // aplica teto no comando de avanço 
  u_fwd = clamp(u_fwd, thrust_min_, thrust_cap); 
  
  if (dist < deadband_m_) {
    // não zera nem reseta: deixa o I segurar contra ondas/corrente
    // só limita para não oscilar
    const double hold_cap = 60.0;  // ajuste (40~120)
    u_fwd = clamp(u_fwd, thrust_min_, hold_cap);
  }
  
  // Empuxo que será definido nos thurster, garantido que não utrapasse os valores mínimos e máximos 
  
  double l_thr = clamp(u_fwd - u_yaw, thrust_min_, thrust_max_); 
  double r_thr = clamp(u_fwd + u_yaw, thrust_min_, thrust_max_); 
  
  double l_pos = forward_pos_; 
  double r_pos = forward_pos_; 
  
  if (use_azimuth_spin_ && dist < 1.0 && std::fabs(yaw_err) > 15.0 * M_PI / 180.0) { 
    const double spin = clamp(std::fabs(u_yaw), 120.0, yaw_mix_max_); 
    
    if (yaw_err > 0) { 
      l_pos = left_pos_ccw_; 
      r_pos = right_pos_ccw_; 
      l_thr = spin; 
      r_thr = spin; 
    } else { 
      l_pos = -left_pos_ccw_; 
      r_pos = -right_pos_ccw_; 
      l_thr = spin; 
      r_thr = spin; 
    } 
  } 
  
  set_cmd(l_pos, r_pos, l_thr, r_thr); 
  RCLCPP_INFO_THROTTLE( 
    get_logger(), *get_clock(), 1000, 
    "dist=%.2fm (E=%.2f N=%.2f) yaw_err=%.1fdeg | u_fwd=%.1f u_yaw=%.1f | L=%.1f R=%.1f", 
    dist, east_m, north_m, yaw_err * 180.0 / M_PI, u_fwd, u_yaw, l_thr, r_thr 
  ); 
}