#include "wildlife/wildlife_node.hpp"
#include "wildlife/geo.hpp"

#include <cmath>
#include <algorithm>

using std::placeholders::_1;
using namespace std::chrono_literals;

Wildlife::Wildlife()
: Node("wildlife_node")
{
  //load_parameters();

  pid_dist_.kp = 180.0;
  pid_dist_.ki = 0.0;
  pid_dist_.kd = 30.0;

  pid_yaw_.kp = 500.0;
  pid_yaw_.ki = 0.0;
  pid_yaw_.kd = 100.0;

  left_thurster_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/wamv/thrusters/left/thrust" , 10);

  right_thurster_pub_ = create_publisher<std_msgs::msg::Float64>(
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
  
  task_info_sub_ = this->create_subscription<ros_gz_interfaces::msg::ParamVec>(
    "/vrx/task/info", 10,
    std::bind(&Wildlife::task_info_callback, this, std::placeholders::_1));

  timer_ = create_wall_timer(
      std::chrono::milliseconds(int(1000.0 / 20)),
      std::bind(&Wildlife::control_step, this));
  
  task_watchdog_timer_ = this->create_wall_timer(
    500ms,
    std::bind(&Wildlife::check_task_info_alive, this));
  
  last_task_info_wall_time_ = std::chrono::steady_clock::now();
}

Wildlife::~Wildlife()
{
  stop_vehicle();
}

void Wildlife::load_parameters()
{
  declare_parameter("pid_dist.kp", 180.0);
  declare_parameter("pid_dist.ki", 0.0);
  declare_parameter("pid_dist.kd", 30.0);

  declare_parameter("pid_yaw.kp", 500.0);
  declare_parameter("pid_yaw.ki", 0.0);
  declare_parameter("pid_yaw.kd", 100.0);

  pid_dist_.kp = get_parameter("pid_dist.kp").as_double();
  pid_dist_.ki = get_parameter("pid_dist.ki").as_double();
  pid_dist_.kd = get_parameter("pid_dist.kd").as_double();

  pid_yaw_.kp = get_parameter("pid_yaw.kp").as_double();
  pid_yaw_.ki = get_parameter("pid_yaw.ki").as_double();
  pid_yaw_.kd = get_parameter("pid_yaw.kd").as_double();

}

void Wildlife::task_info_callback(
  const ros_gz_interfaces::msg::ParamVec::SharedPtr msg)
{
  task_info_seen_once_ = true;
  last_task_info_wall_time_ = std::chrono::steady_clock::now();

  // opcional: inspecionar o conteúdo do tópico
  for (const auto & p : msg->params)
  {
    RCLCPP_DEBUG(this->get_logger(), "task param: %s", p.name.c_str());
    if(p.name == "score")
      last_score_ = p.value.double_value;
  }
}

void Wildlife::check_task_info_alive()
{
  if (!task_info_seen_once_) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const auto elapsed =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          now - last_task_info_wall_time_);

  if (elapsed > std::chrono::milliseconds(3000))
  {
    RCLCPP_WARN(
        this->get_logger(),
        "Sem mensagens em /vrx/task/info há %ld ms. Último score = %.3f .",
        elapsed.count(),
        last_score_
    );

    stop_vehicle();
    rclcpp::shutdown();
  }
}

void Wildlife::publish_cmd(double forward, double yaw)
{
  double left = forward - yaw;
  double right = forward + yaw;

  left = std::clamp(left, -max_forward_cmd_, max_forward_cmd_);
  right = std::clamp(right, -max_forward_cmd_, max_forward_cmd_);

  std_msgs::msg::Float64 l;
  std_msgs::msg::Float64 r;

  l.data = left;
  r.data = right;

  left_thurster_pub_->publish(l);
  right_thurster_pub_->publish(r);
}

void Wildlife::stop_vehicle()
{
  std_msgs::msg::Float64 l;
  std_msgs::msg::Float64 r;

  l.data = 0;
  r.data = 0;

  left_thurster_pub_->publish(l);
  right_thurster_pub_->publish(r);
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

  //Não está é modo de evasão
  if (!avoiding_crocodile_) {

    //Caso verdadeito, significa que ja está em uma distancia segura
    if (dist >= crocodile_safe_radius_m_)
      return false;
  

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

  // Dir
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

// void Wildlife::build_contour_waypoints(double target_x, double target_y, bool clockwise)
// {
//   contour_waypoints_.clear();
//   contour_index_ = 0;
//   contour_initialized_ = true;

//   const double r = contour_radius_m_;

//   const std::pair<double,double> east  = {target_x + r, target_y};
//   const std::pair<double,double> north = {target_x, target_y + r};
//   const std::pair<double,double> west  = {target_x - r, target_y};
//   const std::pair<double,double> south = {target_x, target_y - r};

//   if (clockwise)
//   {
//     // horário
//     contour_waypoints_.push_back(west);
//     contour_waypoints_.push_back(north);
//     contour_waypoints_.push_back(east);
//     contour_waypoints_.push_back(south);
//     contour_waypoints_.push_back(west);
//   }
//   else
//   {
//     // anti-horário
//     contour_waypoints_.push_back(east);
//     contour_waypoints_.push_back(north);
//     contour_waypoints_.push_back(west);
//     contour_waypoints_.push_back(south);
//     contour_waypoints_.push_back(east);
//   }
// }

void Wildlife::build_contour_waypoints(double target_x, double target_y, bool clockwise)
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
  {
    // sentido horário
    base_points = {east, south, west, north};
  }
  else
  {
    // sentido anti-horário
    base_points = {east, north, west, south};
  }

  // Encontrar o ponto mais próximo da posição atual do barco
  std::size_t start_idx = 0;
  double best_dist = std::numeric_limits<double>::max();

  for (std::size_t i = 0; i < base_points.size(); ++i)
  {
    const double d = distance_xy(
      boat_x_, boat_y_,
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
    get_logger(),
    "Contour built. clockwise=%s start_idx=%zu closest_dist=%.2f first_wp=(%.2f, %.2f)",
    clockwise ? "true" : "false",
    start_idx,
    best_dist,
    contour_waypoints_.front().first,
    contour_waypoints_.front().second
  );
}

void Wildlife::reset_contour()
{
  contour_waypoints_.clear();
  contour_index_ = 0;
  contour_initialized_ = false;
}

bool Wildlife::go_to_point(double goal_x, double goal_y, double dt, double forward_base_cmd, double tolerance_m)
{
  const double dx = goal_x - boat_x_;
  const double dy = goal_y - boat_y_;
  const double dist = std::hypot(dx, dy);

  if (dist <= tolerance_m)
  {
    stop_vehicle();
    reset_controllers();
    return true;
  }

  const double desired_yaw = std::atan2(dy, dx);
  const double yaw_error = geo_utils::wrap_pi(desired_yaw - yaw_);

  const double yaw_cmd = pid_yaw_.step(yaw_error, dt);

  double forward_cmd = forward_base_cmd;

  if (std::abs(yaw_error) > max_yaw_error_for_full_speed_rad_ || dist < 4.0)
  {
    forward_cmd *= 0.4;
  }

  publish_cmd(forward_cmd, yaw_cmd);
  return false;
}

bool Wildlife::step_approach_target(double target_x, double target_y, bool clockwise, double dt)
{
  if (!contour_initialized_)
    build_contour_waypoints(target_x, target_y, clockwise);

  if (contour_waypoints_.empty())
    return false;

  const auto & first_wp = contour_waypoints_.front();

  RCLCPP_INFO(
    get_logger(),
    "Approaching first contour waypoint: index=0 goal=(%.2f, %.2f)",
    first_wp.first, first_wp.second
  );

  return go_to_point(
    first_wp.first,
    first_wp.second,
    dt,
    contour_forward_cmd_,
    approach_tolerance_m_
  );
}

bool Wildlife::step_contour(double dt)
{
  if (!contour_initialized_ || contour_waypoints_.empty())
    return false;

  if (contour_index_ >= contour_waypoints_.size())
  {
    stop_vehicle();
    reset_controllers();
    return true;
  }

  const auto & wp = contour_waypoints_[contour_index_];

  RCLCPP_INFO(
    get_logger(),
    "Contour waypoint %zu/%zu -> (%.2f, %.2f)",
    contour_index_ + 1,
    contour_waypoints_.size(),
    wp.first,
    wp.second
  );

  const bool reached = go_to_point(
    wp.first,
    wp.second,
    dt,
    contour_forward_cmd_,
    waypoint_tolerance_m_
  );

  if (reached)
  {
    contour_index_++;

    if (contour_index_ >= contour_waypoints_.size())
    {
      stop_vehicle();
      reset_controllers();
      return true;
    }
  }

  return false;
}

void Wildlife::control_step()
{
  if (!have_all_data())
  {
    stop_vehicle();
    state_ = MissionState::WAIT_DATA;
    return;
  }

  const double dt = 0.05;

  double croc_x, croc_y;
  double plat_x, plat_y;
  double turtle_x, turtle_y;

  geo_utils::latlon_to_local_xy(
      origin_lat_, origin_lon_,
      crocodile_lat_, crocodile_lon_,
      croc_x, croc_y);

  geo_utils::latlon_to_local_xy(
      origin_lat_, origin_lon_,
      platypus_lat_, platypus_lon_,
      plat_x, plat_y);

  geo_utils::latlon_to_local_xy(
      origin_lat_, origin_lon_,
      turtle_lat_, turtle_lon_,
      turtle_x, turtle_y);

  // prioridade máxima: manter distância do crocodilo
  if (avoid_crocodile(croc_x, croc_y, dt))
  {
    return;
  }
  
  const std::string state_name =  mission_state_to_string(state_);

  RCLCPP_INFO(get_logger(), "Current state: %s", state_name.c_str());

  switch (state_)
  {
    case MissionState::WAIT_DATA:
    {
      reset_controllers();
      reset_contour();
      state_ = MissionState::CHOOSE_TARGET;
      break;
    }

    case MissionState::CHOOSE_TARGET:
    {
      reset_controllers();
      reset_contour();

      if (!platypus_done_ && !turtle_done_)
      {
        const double d1 = distance_xy(boat_x_, boat_y_, plat_x, plat_y);
        const double d2 = distance_xy(boat_x_, boat_y_, turtle_x, turtle_y);

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
      const bool reached_start = step_approach_target(plat_x, plat_y, true, dt);

      if (reached_start)
      {
        reset_controllers();
        contour_index_ = 0;
        state_ = MissionState::ORBIT_PLATYPUS_CW;
      }

      break;
    }

    case MissionState::ORBIT_PLATYPUS_CW:
    {
      const bool finished = step_contour(dt);

      if (finished)
      {
        platypus_done_ = true;
        reset_controllers();
        reset_contour();
        state_ = MissionState::CHOOSE_TARGET;
      }

      break;
    }

    case MissionState::APPROACH_TURTLE:
    {
      const bool reached_start = step_approach_target(turtle_x, turtle_y, false, dt);

      if (reached_start)
      {
        reset_controllers();
        contour_index_ = 0;
        state_ = MissionState::ORBIT_TURTLE_CCW;
      }

      break;
    }

    case MissionState::ORBIT_TURTLE_CCW:
    {
      const bool finished = step_contour(dt);

      if (finished)
      {
        turtle_done_ = true;
        reset_controllers();
        reset_contour();
        state_ = MissionState::CHOOSE_TARGET;
      }

      break;
    }

    case MissionState::DONE:
    {
      stop_vehicle();
      break;
    }

    default:
    {
      stop_vehicle();
      break;
    }
  }
}