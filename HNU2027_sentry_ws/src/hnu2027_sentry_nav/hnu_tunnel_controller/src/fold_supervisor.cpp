#include "hnu_tunnel_controller/fold_supervisor.hpp"

#include <cmath>

namespace hnu_tunnel_controller {

FoldSupervisor::FoldSupervisor(FoldConfig config) : config_(config) {}

bool FoldSupervisor::request(FoldTarget target, double now_s) {
  if (!std::isfinite(now_s) || config_.tolerance <= 0.0 || config_.timeout_s <= 0.0 ||
      !std::isfinite(config_.folded_position) || !std::isfinite(config_.unfolded_position)) {
    feedback_ = {target, 0.0, false, true, "invalid fold configuration"};
    active_ = false;
    return false;
  }
  target_ = target;
  started_at_s_ = now_s;
  active_ = true;
  feedback_ = {target_, feedback_.joint_position, false, false, "command accepted"};
  return true;
}

FoldFeedback FoldSupervisor::update(double now_s, bool feedback_valid, double joint_position,
                                    bool actuator_fault) {
  feedback_.target = target_;
  if (!active_) return feedback_;
  if (actuator_fault) {
    feedback_ = {target_, joint_position, false, true, "actuator reported fault"};
    active_ = false;
  } else if (!feedback_valid || !std::isfinite(joint_position)) {
    feedback_ = {target_, joint_position, false, true, "invalid joint feedback"};
    active_ = false;
  } else if (!std::isfinite(now_s) || now_s < started_at_s_) {
    feedback_ = {target_, joint_position, false, true, "invalid controller time"};
    active_ = false;
  } else if (std::abs(joint_position - target_position()) <= config_.tolerance) {
    feedback_ = {target_, joint_position, true, false, "target reached"};
    active_ = false;
  } else if (now_s - started_at_s_ > config_.timeout_s) {
    feedback_ = {target_, joint_position, false, true, "fold command timed out"};
    active_ = false;
  } else {
    feedback_ = {target_, joint_position, false, false, "moving"};
  }
  return feedback_;
}

FoldFeedback FoldSupervisor::stop(double, const std::string& reason) {
  active_ = false;
  feedback_.completed = false;
  feedback_.fault = true;
  feedback_.reason = reason.empty() ? "stopped" : reason;
  return feedback_;
}

double FoldSupervisor::command_position() const { return target_position(); }
bool FoldSupervisor::command_active() const { return active_; }
double FoldSupervisor::target_position() const {
  return target_ == FoldTarget::kFolded ? config_.folded_position : config_.unfolded_position;
}

}  // namespace hnu_tunnel_controller
