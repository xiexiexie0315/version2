#pragma once

#include <string>

namespace hnu_tunnel_controller {

enum class FoldTarget { kUnfolded, kFolded };

struct FoldConfig {
  double unfolded_position{0.0};
  double folded_position{-0.16};
  double tolerance{0.005};
  double timeout_s{3.0};
};

struct FoldFeedback {
  FoldTarget target{FoldTarget::kUnfolded};
  double joint_position{0.0};
  bool completed{false};
  bool fault{false};
  std::string reason{"idle"};
};

// Deterministic execution logic. It has no ROS or Gazebo dependency so it can
// be tested identically in simulation and on the real actuator adapter.
class FoldSupervisor {
 public:
  explicit FoldSupervisor(FoldConfig config = {});
  bool request(FoldTarget target, double now_s);
  FoldFeedback update(double now_s, bool feedback_valid, double joint_position,
                      bool actuator_fault = false);
  FoldFeedback stop(double now_s, const std::string& reason);
  double command_position() const;
  bool command_active() const;

 private:
  double target_position() const;
  FoldConfig config_;
  FoldTarget target_{FoldTarget::kUnfolded};
  double started_at_s_{0.0};
  bool active_{false};
  FoldFeedback feedback_{};
};

}  // namespace hnu_tunnel_controller
