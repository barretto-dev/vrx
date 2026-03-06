#include "wildlife/wildlife_node.hpp"
#include "wildlife/geo.hpp"

#include <cmath>
#include <algorithm>

using std::placeholders::_1;

Wildlife::Wildlife()
: Node("wildlife_node")
{
  load_parameters();

  left_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/wamv/thrusters/left/thrust" , 10);

  right_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/wamv/thrusters/right/thrust", 10);

  gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/wamv/sensors/gps/gps/fix", 10,
      std::bind(&Wildlife::gps_callback, this, _1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/wamv/sensors/imu/imu/data", 10,
      std::bind(&Wildlife::imu_callback, this, _1));

  crocodile_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/vrx/wildlife/animal0/pose", 10,
      std::bind(&Wildlife::crocodile_callback, this, _1));

  platypus_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/vrx/wildlife/animal1/pose", 10,
      std::bind(&Wildlife::platypus_callback, this, _1));

  turtle_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/vrx/wildlife/animal2/pose", 10,
      std::bind(&Wildlife::turtle_callback, this, _1));

  timer_ = create_wall_timer(
      std::chrono::milliseconds(int(1000.0 / 20)),
      std::bind(&Wildlife::control_step, this));
}

Wildlife::~Wildlife()
{
  stop_vehicle();
}

void Wildlife::load_parameters()
{
  declare_parameter("control_rate_hz", 10.0);

  declare_parameter("orbit_radius_m", 8.0);
  declare_parameter("max_orbit_radius_m", 10.0);
  declare_parameter("entry_tolerance_m", 1.2);
  declare_parameter("lookahead_m", 4.0);

  declare_parameter("crocodile_safe_radius_m", 10.0);
  declare_parameter("crocodile_release_radius_m", 12.0);

  declare_parameter("max_forward_cmd", 900.0);
  declare_parameter("max_turn_cmd", 350.0);
  declare_parameter("base_forward_cmd", 250.0);

  declare_parameter("pid_dist.kp", 180.0);
  declare_parameter("pid_dist.ki", 0.0);
  declare_parameter("pid_dist.kd", 30.0);

  declare_parameter("pid_yaw.kp", 500.0);
  declare_parameter("pid_yaw.ki", 0.0);
  declare_parameter("pid_yaw.kd", 100.0);

  get_parameter("orbit_radius_m", orbit_radius_m_);
  get_parameter("max_orbit_radius_m", max_orbit_radius_m_);
  get_parameter("entry_tolerance_m", entry_tolerance_m_);
  get_parameter("lookahead_m", lookahead_m_);

  get_parameter("crocodile_safe_radius_m", crocodile_safe_radius_m_);
  get_parameter("crocodile_release_radius_m", crocodile_release_radius_m_);

  get_parameter("max_forward_cmd", max_forward_cmd_);
  get_parameter("max_turn_cmd", max_turn_cmd_);
  get_parameter("base_forward_cmd", base_forward_cmd_);

  pid_dist_.kp = get_parameter("pid_dist.kp").as_double();
  pid_dist_.ki = get_parameter("pid_dist.ki").as_double();
  pid_dist_.kd = get_parameter("pid_dist.kd").as_double();

  pid_yaw_.kp = get_parameter("pid_yaw.kp").as_double();
  pid_yaw_.ki = get_parameter("pid_yaw.ki").as_double();
  pid_yaw_.kd = get_parameter("pid_yaw.kd").as_double();

}

void Wildlife::publish_thrusters(double left, double right)
{
  std_msgs::msg::Float64 l;
  std_msgs::msg::Float64 r;

  l.data = left;
  r.data = right;

  left_pub_->publish(l);
  right_pub_->publish(r);
}

void Wildlife::publish_cmd(double forward, double yaw)
{
  double left = forward - yaw;
  double right = forward + yaw;

  left = std::clamp(left, -max_forward_cmd_, max_forward_cmd_);
  right = std::clamp(right, -max_forward_cmd_, max_forward_cmd_);

  publish_thrusters(left, right);
}

void Wildlife::stop_vehicle()
{
  publish_thrusters(0.0, 0.0);
}

double Wildlife::distance_xy(double x1,double y1,double x2,double y2)
{
  return std::hypot(x2-x1,y2-y1);
}

void Wildlife::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  boat_lat_ = msg->latitude;
  boat_lon_ = msg->longitude;
  have_gps_ = true;

  if(!have_origin_)
  {
    origin_lat_ = boat_lat_;
    origin_lon_ = boat_lon_;
    have_origin_ = true;
  }

  geo_utils::latlon_to_local_xy(
      origin_lat_,origin_lon_,
      boat_lat_,boat_lon_,
      boat_x_,boat_y_);
}

void Wildlife::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  yaw_ = geo_utils::yaw_from_quat(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w);

  have_imu_ = true;
}

void Wildlife::crocodile_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  crocodile_lat_ = msg->pose.position.x;
  crocodile_lon_ = msg->pose.position.y;
  have_crocodile_ = true;
}

void Wildlife::platypus_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  platypus_lat_ = msg->pose.position.x;
  platypus_lon_ = msg->pose.position.y;
  have_platypus_ = true;
}

void Wildlife::turtle_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  turtle_lat_ = msg->pose.position.x;
  turtle_lon_ = msg->pose.position.y;
  have_turtle_ = true;
}

bool Wildlife::have_all_data()
{
  RCLCPP_INFO(get_logger(), 
    "GPS: %s | IMU: %s | Plat: %s | Turtle: %s | Croc: %s ",
    have_gps_ ? "true" : "false", 
    have_imu_ ? "true" : "false", 
    have_platypus_ ? "true" : "false", 
    have_turtle_ ? "true" : "false", 
    have_crocodile_ ? "true" : "false"
  );
  return have_gps_ && have_imu_ &&
         have_platypus_ &&
         have_turtle_ &&
         have_crocodile_;
}

void Wildlife::reset_controllers()
{
  pid_dist_.reset();
  pid_yaw_.reset();
}

bool Wildlife::avoid_crocodile(double croc_x, double croc_y, double dt)
{
  const double dx = boat_x_ - croc_x;
  const double dy = boat_y_ - croc_y;
  const double dist = std::hypot(dx, dy);

  if (!avoiding_crocodile_) {
    if (dist >= crocodile_safe_radius_m_) {
      return false;
    }
    avoiding_crocodile_ = true;
    reset_controllers();
    RCLCPP_WARN(get_logger(), "Too close to crocodile (%.2f m). Starting avoidance.", dist);
  } else {
    if (dist >= crocodile_release_radius_m_) {
      avoiding_crocodile_ = false;
      reset_controllers();
      RCLCPP_INFO(get_logger(), "Crocodile avoidance cleared (%.2f m).", dist);
      return false;
    }
  }

  // Direção oposta ao jacaré
  const double desired_yaw = std::atan2(dy, dx);
  const double yaw_error = geo_utils::wrap_pi(desired_yaw - yaw_);

  const double yaw_cmd = pid_yaw_.step(yaw_error, dt);

  double forward_cmd = base_forward_cmd_;
  if (std::abs(yaw_error) > 1.0) {
    forward_cmd *= 0.35;
  }

  publish_cmd(forward_cmd, yaw_cmd);
  return true;
}

void Wildlife::control_step()
{
  if(!have_all_data())
  {
    stop_vehicle();
    return;
  }

  RCLCPP_INFO(get_logger(), "COMEÇOU !!!!!!!!!!!");
  const double dt = 0.05;

  double croc_x,croc_y;
  double plat_x,plat_y;
  double turtle_x,turtle_y;

  geo_utils::latlon_to_local_xy(origin_lat_,origin_lon_,
                                crocodile_lat_,crocodile_lon_,
                                croc_x,croc_y);

  geo_utils::latlon_to_local_xy(origin_lat_,origin_lon_,
                                platypus_lat_,platypus_lon_,
                                plat_x,plat_y);

  geo_utils::latlon_to_local_xy(origin_lat_,origin_lon_,
                                turtle_lat_,turtle_lon_,
                                turtle_x,turtle_y);

  if(avoid_crocodile(croc_x,croc_y,dt))
      return;

  switch(state_)
  {
    case MissionState::WAIT_DATA:
      state_=MissionState::CHOOSE_TARGET;
      break;

    case MissionState::CHOOSE_TARGET:
    {
      if(!platypus_done_ && !turtle_done_)
      {
        double d1 = distance_xy(boat_x_,boat_y_,plat_x,plat_y);
        double d2 = distance_xy(boat_x_,boat_y_,turtle_x,turtle_y);

        state_ = (d1<d2)?
            MissionState::APPROACH_PLATYPUS:
            MissionState::APPROACH_TURTLE;
      }
      else if(!platypus_done_)
        state_=MissionState::APPROACH_PLATYPUS;
      else if(!turtle_done_)
        state_=MissionState::APPROACH_TURTLE;
      else
        state_=MissionState::DONE;

      break;
    }

    case MissionState::DONE:
      stop_vehicle();
      break;

    default:
      break;
  }
}