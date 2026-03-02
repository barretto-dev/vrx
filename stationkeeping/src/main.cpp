#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <iostream>

#include "stationkeeping/stationkeeping_node.hpp"

using namespace std::chrono_literals;

static std::atomic_bool g_stop_requested{false};
static void sigint_handler(int) { g_stop_requested.store(true); }

int main(int argc, char **argv)
{
  rclcpp::InitOptions opts;
  opts.shutdown_on_signal = false;
  rclcpp::init(argc, argv, opts);

  std::signal(SIGINT, sigint_handler);

  auto node = std::make_shared<Stationkeeping>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  while (rclcpp::ok() && !g_stop_requested.load()) {
    exec.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  node->cancel_timer();
  std::cerr << "[stationkeeping] Ctrl+C received. Sending STOP...\n";

  for (int i = 0; i < 50; ++i) {
    node->send_stop_once();
    exec.spin_some();
    std::this_thread::sleep_for(20ms);
  }

  exec.remove_node(node);
  rclcpp::shutdown();
  return 0;
}