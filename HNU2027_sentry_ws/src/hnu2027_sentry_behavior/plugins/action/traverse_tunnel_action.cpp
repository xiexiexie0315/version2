#include "hnu2027_sentry_behavior/plugins/action/traverse_tunnel_action.hpp"

#include "behaviortree_ros2/plugins.hpp"
#include "hnu2027_sentry_behavior/traverse_tunnel_outcome.hpp"

namespace hnu2027_sentry_behavior {

bool is_complete_success(bool action_succeeded, const std::string& terminal_phase) {
  return action_succeeded && terminal_phase == "complete";
}

TraverseTunnelAction::TraverseTunnelAction(const std::string& name,
                                           const BT::NodeConfig& config,
                                           const BT::RosNodeParams& params)
: BT::RosActionNode<hnu_sentry_interfaces::action::TraverseTunnel>(name, config, params) {}

BT::PortsList TraverseTunnelAction::providedPorts() {
  return providedBasicPorts({
    BT::InputPort<std::string>("tunnel_id", "Tunnel identifier from the tunnel prior"),
    BT::InputPort<std::string>("direction", "entry_to_exit", "entry_to_exit or exit_to_entry"),
    BT::OutputPort<std::string>("terminal_phase", "Action terminal phase"),
    BT::OutputPort<std::string>("reason", "Structured success or failure reason"),
    BT::OutputPort<std::string>("current_phase", "Latest Action feedback phase"),
  });
}

bool TraverseTunnelAction::setGoal(Goal& goal) {
  const auto tunnel_id = getInput<std::string>("tunnel_id");
  const auto direction = getInput<std::string>("direction");
  if (!tunnel_id || tunnel_id->empty() || !direction) {
    setOutput("reason", "invalid_bt_inputs");
    return false;
  }
  goal.tunnel_id = *tunnel_id;
  if (*direction == "entry_to_exit") {
    goal.direction = Goal::DIRECTION_ENTRY_TO_EXIT;
  } else if (*direction == "exit_to_entry") {
    goal.direction = Goal::DIRECTION_EXIT_TO_ENTRY;
  } else {
    setOutput("reason", "invalid_direction");
    return false;
  }
  return true;
}

BT::NodeStatus TraverseTunnelAction::onResultReceived(const WrappedResult& wrapped) {
  if (!wrapped.result) {
    setOutput("reason", "missing_action_result");
    return BT::NodeStatus::FAILURE;
  }
  setOutput("terminal_phase", wrapped.result->terminal_phase);
  setOutput("reason", wrapped.result->reason);
  const bool transport_succeeded = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED;
  return is_complete_success(transport_succeeded && wrapped.result->success,
                             wrapped.result->terminal_phase)
      ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus TraverseTunnelAction::onFeedback(const std::shared_ptr<const Feedback> feedback) {
  if (!feedback) return BT::NodeStatus::FAILURE;
  setOutput("current_phase", feedback->phase);
  setOutput("reason", feedback->detail);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TraverseTunnelAction::onFailure(BT::ActionNodeErrorCode error) {
  setOutput("reason", "action_transport_failure:" + std::to_string(static_cast<int>(error)));
  return BT::NodeStatus::FAILURE;
}

void TraverseTunnelAction::onHalt() {
  setOutput("reason", "bt_halted_or_cancelled");
}

}  // namespace hnu2027_sentry_behavior

CreateRosNodePlugin(hnu2027_sentry_behavior::TraverseTunnelAction, "TraverseTunnel");
