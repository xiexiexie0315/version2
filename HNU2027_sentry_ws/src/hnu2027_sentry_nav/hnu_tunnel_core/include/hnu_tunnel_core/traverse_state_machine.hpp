#pragma once

#include <string>

namespace hnu_tunnel_core
{

enum class TraversePhase
{
  kNormal,
  kApproach,
  kDetect,
  kAlign,
  kFold,
  kVerify,
  kTraverse,
  kExit,
  kUnfold,
  kComplete,
  kFallback,
  kAbort,
};

std::string to_string(TraversePhase phase);

struct TraverseTimeouts
{
  double approach_s{30.0};
  double detect_s{2.0};
  double align_s{8.0};
  double fold_s{5.0};
  double verify_s{2.0};
  double traverse_s{15.0};
  double unfold_s{5.0};

  bool valid() const;
};

struct TraverseInput
{
  bool at_staging_pose{false};
  bool observation_safe{false};
  bool aligned{false};
  bool fold_target{false};
  bool fold_completed{false};
  bool fold_fault{false};
  bool exited_tunnel{false};
  bool cancel_requested{false};
};

struct TraverseCommand
{
  TraversePhase phase{TraversePhase::kNormal};
  bool owns_motion_control{false};
  bool request_stop{false};
  bool request_fold{false};
  bool request_unfold{false};
  bool allow_traverse{false};
  std::string reason;
};

class TraverseStateMachine
{
public:
  explicit TraverseStateMachine(TraverseTimeouts timeouts = {});

  TraverseCommand start(double now_s);
  TraverseCommand step(const TraverseInput & input, double now_s);
  TraversePhase phase() const { return phase_; }

private:
  TraverseCommand command(const std::string & reason) const;
  void transition(TraversePhase next, double now_s);
  bool expired(double now_s, double timeout_s) const;

  TraverseTimeouts timeouts_;
  TraversePhase phase_{TraversePhase::kNormal};
  double entered_at_s_{0.0};
};

}  // namespace hnu_tunnel_core
