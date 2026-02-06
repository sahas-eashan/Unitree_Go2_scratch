#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"

using namespace std::chrono_literals;

static double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

class CmdVelUnitreeBridge : public rclcpp::Node {
 public:
  CmdVelUnitreeBridge() : Node("cmd_vel_unitree_bridge") {
    // Params
    max_vx_ = this->declare_parameter<double>("max_vx", 0.6);
    max_vy_ = this->declare_parameter<double>("max_vy", 0.3);
    max_wz_ = this->declare_parameter<double>("max_wz", 1.2);
    timeout_s_ = this->declare_parameter<double>("timeout_s", 0.25);

    pub_ = this->create_publisher<unitree_api::msg::Request>(
        "/api/sport/request", 10);

    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::QoS(10),
        std::bind(&CmdVelUnitreeBridge::on_cmd_vel, this,
                  std::placeholders::_1));

    last_cmd_time_ = this->now();

    watchdog_ = this->create_wall_timer(
        50ms, std::bind(&CmdVelUnitreeBridge::watchdog, this));

    RCLCPP_INFO(this->get_logger(),
                "cmd_vel -> /api/sport/request bridge running (api_id=1008). "
                "Limits: max_vx=%.3f max_vy=%.3f max_wz=%.3f timeout=%.2f",
                max_vx_, max_vy_, max_wz_, timeout_s_);
  }

 private:
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    last_cmd_time_ = this->now();

    double vx = clamp(msg->linear.x, -max_vx_, max_vx_);
    double vy = clamp(msg->linear.y, -max_vy_, max_vy_);
    double wz = clamp(msg->angular.z, -max_wz_, max_wz_);

    publish_move(vx, vy, wz, /*noreply=*/true);
  }

  void watchdog() {
    const auto dt = (this->now() - last_cmd_time_).seconds();
    if (dt > timeout_s_) {
      // auto-stop once, then keep quiet
      if (!stopped_) {
        publish_move(0.0, 0.0, 0.0, /*noreply=*/true);
        stopped_ = true;
        RCLCPP_WARN(this->get_logger(),
                    "cmd_vel timeout -> STOP published");
      }
    } else {
      stopped_ = false;
    }
  }

  void publish_move(double vx, double vy, double wz, bool noreply) {
    unitree_api::msg::Request req;

    // --- header (matching observed structure) ---
    // identity.id: use monotonically increasing id (timestamp ns is fine)
    req.header.identity.id =
        static_cast<uint64_t>(this->now().nanoseconds());
    req.header.identity.api_id = 1008;  // observed move command
    req.header.lease.id = 0;

    req.header.policy.priority = 0;
    req.header.policy.noreply = noreply;

    // parameter JSON exactly like your echo: {"x":...,"y":...,"z":...}
    std::ostringstream ss;
    ss << std::setprecision(12);
    ss << "{\"x\":" << vx << ",\"y\":" << vy << ",\"z\":" << wz
       << "}";
    req.parameter = ss.str();

    req.binary.clear();  // observed empty

    pub_->publish(req);
  }

  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;

  rclcpp::Time last_cmd_time_;
  bool stopped_{false};

  double max_vx_{0.6};
  double max_vy_{0.3};
  double max_wz_{1.2};
  double timeout_s_{0.25};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelUnitreeBridge>());
  rclcpp::shutdown();
  return 0;
}
