#ifndef LIFT_CONTROLLER__LIFT_CONTROLLER_HPP_
#define LIFT_CONTROLLER__LIFT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "control_msgs/msg/joint_controller_state.hpp"
#include "control_msgs/msg/joint_jog.hpp"
#include "control_toolbox/pid_ros.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

namespace lift_controller {
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LiftController : public controller_interface::ControllerInterface {
public:
  LiftController();

  CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;
  CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

protected:
  void initializePidControllers();

  std::vector<std::string> joint_names_;
  std::string interface_name_;
  std::vector<control_toolbox::PidROS> pid_controllers_;
  std::vector<double>
      current_positions_; // Stores the current positions of each joint

  using ControllerCommandMsg = control_msgs::msg::JointJog;
  rclcpp::Subscription<ControllerCommandMsg>::SharedPtr command_subscriber_ =
      nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerCommandMsg>>
      input_command_;

  using ControllerStateMsg = control_msgs::msg::JointControllerState;
  using ControllerStatePublisher =
      realtime_tools::RealtimePublisher<ControllerStateMsg>;
  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;
};

} // namespace lift_controller

#endif // LIFT_CONTROLLER__LIFT_CONTROLLER_HPP_
