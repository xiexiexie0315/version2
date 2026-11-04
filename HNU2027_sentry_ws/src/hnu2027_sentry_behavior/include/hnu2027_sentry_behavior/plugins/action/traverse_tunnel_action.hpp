#pragma once

#include <memory>
#include <string>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "hnu_sentry_interfaces/action/traverse_tunnel.hpp"

namespace hnu2027_sentry_behavior {

class TraverseTunnelAction
  : public BT::RosActionNode<hnu_sentry_interfaces::action::TraverseTunnel> {
 public:
  TraverseTunnelAction(const std::string& name, const BT::NodeConfig& config,
                       const BT::RosNodeParams& params);
  static BT::PortsList providedPorts();
  bool setGoal(Goal& goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult& result) override;
  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
  void onHalt() override;
};

}  // namespace hnu2027_sentry_behavior
