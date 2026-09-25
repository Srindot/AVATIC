// Copyright 2026 AVATIC contributors.
//
// fc_link: the simulator's stand-in for the Pluto X Wi-Fi/MSP link.
//
//   ROS pluto_x_interfaces/RcCommand  ->  gz gz.msgs.Int32_V (8 channels,
//                                         AETR1234, microseconds)
//                                         (on hardware: MSP_SET_RAW_RC)
//   gz gz.msgs.Double_V telemetry     ->  ROS pluto_x_interfaces/
//                                         FlightControllerStatus
//                                         (on hardware: MSP_STATUS,
//                                         MSP_ATTITUDE, MSP_ALTITUDE, ...)
//
// The same ROS interface will drive the real Pluto X through a hardware
// bridge, so participant code sees one interface for both.
//
// Parameters:
//   rc_ros_topic          (default /pluto/rc)
//   rc_gz_topic           (default /pluto/rc)
//   telemetry_gz_topic    (default /pluto/fc_telemetry)
//   status_ros_topic      (default /pluto/fc_status)
//
// RC commands with any channel outside [kMinChannelUs, kMaxChannelUs] are
// rejected (not forwarded) and reported with a throttled warning.

#include <gz/msgs/double_v.pb.h>
#include <gz/msgs/int32_v.pb.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>

#include "pluto_x/control/telemetry_wire.hpp"
#include "pluto_x_interfaces/msg/flight_controller_status.hpp"
#include "pluto_x_interfaces/msg/rc_command.hpp"

namespace {

constexpr std::uint16_t kMinChannelUs = 800;
constexpr std::uint16_t kMaxChannelUs = 2200;
constexpr int kWarnThrottleMs = 2000;

namespace wire = pluto_x::telemetry_wire;

class FcLink : public rclcpp::Node {
 public:
  FcLink() : rclcpp::Node("fc_link") {
    const std::string rc_ros = declare_parameter("rc_ros_topic", "/pluto/rc");
    const std::string rc_gz = declare_parameter("rc_gz_topic", "/pluto/rc");
    const std::string telemetry_gz =
        declare_parameter("telemetry_gz_topic", "/pluto/fc_telemetry");
    const std::string status_ros =
        declare_parameter("status_ros_topic", "/pluto/fc_status");

    rc_publisher_ = gz_node_.Advertise<gz::msgs::Int32_V>(rc_gz);
    if (!rc_publisher_) {
      throw std::runtime_error("cannot advertise gz topic " + rc_gz);
    }
    rc_subscription_ = create_subscription<pluto_x_interfaces::msg::RcCommand>(
        rc_ros, rclcpp::QoS(10),
        [this](const pluto_x_interfaces::msg::RcCommand& msg) { ForwardRc(msg); });

    status_publisher_ =
        create_publisher<pluto_x_interfaces::msg::FlightControllerStatus>(
            status_ros, rclcpp::QoS(10));
    if (!gz_node_.Subscribe(telemetry_gz, &FcLink::OnTelemetry, this)) {
      throw std::runtime_error("cannot subscribe to gz topic " + telemetry_gz);
    }
    RCLCPP_INFO(get_logger(),
                "RC: ROS %s -> gz %s; telemetry: gz %s -> ROS %s",
                rc_ros.c_str(), rc_gz.c_str(), telemetry_gz.c_str(),
                status_ros.c_str());
  }

 private:
  void ForwardRc(const pluto_x_interfaces::msg::RcCommand& msg) {
    const std::array<std::uint16_t, 8> channels = {
        msg.roll_us, msg.pitch_us, msg.throttle_us, msg.yaw_us,
        msg.aux1_us, msg.aux2_us,  msg.aux3_us,     msg.aux4_us};
    gz::msgs::Int32_V out;
    for (const std::uint16_t value : channels) {
      if (value < kMinChannelUs || value > kMaxChannelUs) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs,
                             "rejected RC command: channel value %u us "
                             "outside [%u, %u]",
                             value, kMinChannelUs, kMaxChannelUs);
        return;
      }
      out.add_data(value);
    }
    rc_publisher_.Publish(out);
  }

  // gz-transport thread; rclcpp publishers are thread-safe.
  void OnTelemetry(const gz::msgs::Double_V& message) {
    if (message.data_size() != static_cast<int>(wire::kCount)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs,
                           "telemetry frame has %d values, expected %zu",
                           message.data_size(),
                           static_cast<std::size_t>(wire::kCount));
      return;
    }
    const auto at = [&message](wire::Index i) { return message.data(i); };
    pluto_x_interfaces::msg::FlightControllerStatus status;
    const double t = at(wire::kSimTimeS);
    // integer nanoseconds first: a fraction rounding up to 1 s carries into sec
    const std::int64_t t_ns = std::llround(t * 1e9);
    status.header.stamp.sec = static_cast<std::int32_t>(t_ns / 1000000000LL);
    status.header.stamp.nanosec = static_cast<std::uint32_t>(t_ns % 1000000000LL);
    status.header.frame_id = "base_link";
    status.armed = at(wire::kArmed) > 0.5;
    status.ok_to_arm = at(wire::kOkToArm) > 0.5;
    status.calibrated = at(wire::kCalibrated) > 0.5;
    status.angle_mode = at(wire::kAngleMode) > 0.5;
    status.altitude_hold = at(wire::kAltitudeHold) > 0.5;
    status.healthy = at(wire::kHealthy) > 0.5;
    status.roll_deg = static_cast<float>(at(wire::kRollDeg));
    status.pitch_deg = static_cast<float>(at(wire::kPitchDeg));
    status.heading_deg = static_cast<float>(at(wire::kHeadingDeg));
    status.altitude_m = static_cast<float>(at(wire::kAltitudeM));
    status.battery_v = static_cast<float>(at(wire::kBatteryV));
    status_publisher_->publish(status);
  }

  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher rc_publisher_;
  rclcpp::Subscription<pluto_x_interfaces::msg::RcCommand>::SharedPtr
      rc_subscription_;
  rclcpp::Publisher<pluto_x_interfaces::msg::FlightControllerStatus>::SharedPtr
      status_publisher_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FcLink>());
  rclcpp::shutdown();
  return 0;
}
