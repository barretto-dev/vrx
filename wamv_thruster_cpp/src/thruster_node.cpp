#include <memory>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

static std::atomic_bool g_stop_requested{false};
static void sigint_handler(int) { g_stop_requested.store(true); }

class SpinCCW : public rclcpp::Node
{
public:
  SpinCCW() : Node("wamv_spin_ccw")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    l_thrust_ = create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/left/thrust", qos);
    r_thrust_ = create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/right/thrust", qos);
    l_pos_    = create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/left/pos", qos);
    r_pos_    = create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/right/pos", qos);

    // >>> CCW no próprio eixo (de acordo com sua observação: right_pos=-1 dá CCW)
    left_pos_cmd_  = +1.0;
    right_pos_cmd_ = -1.0;

    thrust_cmd_ = 200.0; // ajuste: 200-600 costuma ser bom

    timer_ = create_wall_timer(100ms, [this]() {
      publish(l_pos_, left_pos_cmd_);
      publish(r_pos_, right_pos_cmd_);
      publish(l_thrust_, thrust_cmd_);
      publish(r_thrust_, thrust_cmd_);
    });

    RCLCPP_INFO(get_logger(), "Spin CCW: left/pos=%.1f right/pos=%.1f thrust=%.1f",
                left_pos_cmd_, right_pos_cmd_, thrust_cmd_);
  }

  void cancel_timer() { if (timer_) timer_->cancel(); }

  void send_stop_once()
  {
    publish(l_thrust_, 0.0); publish(r_thrust_, 0.0);
    publish(l_pos_, 0.0);    publish(r_pos_, 0.0);
  }

private:
  void publish(const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr &pub, double v)
  {
    std_msgs::msg::Float64 msg;
    msg.data = v;
    pub->publish(msg);
  }

  double left_pos_cmd_{0.0}, right_pos_cmd_{0.0}, thrust_cmd_{0.0};

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr l_thrust_, r_thrust_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr l_pos_, r_pos_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  // Não deixe o ROS desligar sozinho no SIGINT, senão você perde o contexto antes do STOP
  rclcpp::InitOptions opts;
  opts.shutdown_on_signal = false;
  rclcpp::init(argc, argv, opts);

  std::signal(SIGINT, sigint_handler);

  auto node = std::make_shared<SpinCCW>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  while (rclcpp::ok() && !g_stop_requested.load()) {
    exec.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  // Para de mandar comandos "de giro"
  node->cancel_timer();

  // Evita usar RCLCPP_* aqui (rosout pode falhar em shutdown). Use stderr:
  std::cerr << "[wamv_spin_ccw] Ctrl+C received. Sending STOP to all thruster topics...\n";

  // Envia STOP repetidamente por ~1s para garantir que o plugin receba
  for (int i = 0; i < 50; ++i) {
    node->send_stop_once();
    exec.spin_some();
    std::this_thread::sleep_for(20ms);
  }

  exec.remove_node(node);
  rclcpp::shutdown();
  return 0;
}