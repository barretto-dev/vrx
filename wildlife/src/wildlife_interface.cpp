#include "wildlife/wildlife_interface.hpp"

#include "wildlife/geo.hpp"

WildlifeInterface::WildlifeInterface(rclcpp::Node * node)
: node_(node)
{
  left_thrust_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
      "/wamv/thrusters/left/thrust", 10);
  right_thrust_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
      "/wamv/thrusters/right/thrust", 10);

  left_pos_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
      "/wamv/thrusters/left/pos", 10);
  right_pos_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
      "/wamv/thrusters/right/pos", 10);

  gps_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/wamv/sensors/gps/gps/fix", 10,
      std::bind(&WildlifeInterface::gps_callback, this, std::placeholders::_1));

  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      "/wamv/sensors/imu/imu/data", 10,
      std::bind(&WildlifeInterface::imu_callback, this, std::placeholders::_1));

  crocodile_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/vrx/wildlife/animal0/pose", 10,
      std::bind(&WildlifeInterface::crocodile_callback, this, std::placeholders::_1));

  platypus_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/vrx/wildlife/animal1/pose", 10,
      std::bind(&WildlifeInterface::platypus_callback, this, std::placeholders::_1));

  turtle_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/vrx/wildlife/animal2/pose", 10,
      std::bind(&WildlifeInterface::turtle_callback, this, std::placeholders::_1));

  task_info_sub_ = node_->create_subscription<ros_gz_interfaces::msg::ParamVec>(
      "/vrx/task/info", 10,
      std::bind(&WildlifeInterface::task_info_callback, this, std::placeholders::_1));

  last_task_info_wall_time_ = std::chrono::steady_clock::now();
}

void WildlifeInterface::publish_thrusters(double left, double right)
{
  std_msgs::msg::Float64 l;
  std_msgs::msg::Float64 r;
  l.data = left;
  r.data = right;
  left_thrust_pub_->publish(l);
  right_thrust_pub_->publish(r);
}

void WildlifeInterface::publish_thruster_angles(double left_angle, double right_angle)
{
  std_msgs::msg::Float64 l;
  std_msgs::msg::Float64 r;
  l.data = left_angle;
  r.data = right_angle;
  left_pos_pub_->publish(l);
  right_pos_pub_->publish(r);
}

void WildlifeInterface::stop_vehicle()
{
  publish_thrusters(0.0, 0.0);
  publish_thruster_angles(0.0, 0.0);
}

bool WildlifeInterface::have_all_data() const
{
    // RCLCPP_INFO(
    //     node_->get_logger(),
    //     "have_gps=%s | have_imu=%s | have_origin=%s | have_crocodile=%s | have_platypus=%s | have_turtle=%s",
    //     have_gps_ ? "true" : "false",
    //     have_imu_ ? "true" : "false",
    //     have_origin_ ? "true" : "false",
    //     have_crocodile_ ? "true" : "false",
    //     have_platypus_ ? "true" : "false",
    //     have_turtle_ ? "true" : "false"
    // );


    return have_gps_ && have_imu_ &&
            have_crocodile_ && have_platypus_ && have_turtle_;
}

void WildlifeInterface::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  boat_lat_ = msg->latitude;
  boat_lon_ = msg->longitude;
  have_gps_ = true;

  if (!have_origin_)
  {
    origin_lat_ = boat_lat_;
    origin_lon_ = boat_lon_;
    have_origin_ = true;
  }

  geo_utils::latlon_to_local_xy(
      origin_lat_, origin_lon_,
      boat_lat_, boat_lon_,
      boat_x_, boat_y_);
}

void WildlifeInterface::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  yaw_ = geo_utils::yaw_from_quat(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w);
  have_imu_ = true;
}

void WildlifeInterface::crocodile_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  crocodile_lat_ = msg->pose.position.x;
  crocodile_lon_ = msg->pose.position.y;
  have_crocodile_ = true;
}

void WildlifeInterface::platypus_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  platypus_lat_ = msg->pose.position.x;
  platypus_lon_ = msg->pose.position.y;
  have_platypus_ = true;
}

void WildlifeInterface::turtle_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  turtle_lat_ = msg->pose.position.x;
  turtle_lon_ = msg->pose.position.y;
  have_turtle_ = true;
}

void WildlifeInterface::task_info_callback(const ros_gz_interfaces::msg::ParamVec::SharedPtr msg)
{
  task_info_seen_once_ = true;
  last_task_info_wall_time_ = std::chrono::steady_clock::now();

  for (const auto & p : msg->params)
  {
    if(p.name == "score")
      last_score_ = p.value.double_value;
    
  }
}