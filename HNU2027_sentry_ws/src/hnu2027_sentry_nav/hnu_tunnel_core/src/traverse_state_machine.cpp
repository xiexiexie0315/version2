#include "hnu_tunnel_core/traverse_state_machine.hpp"

#include <cmath>
#include <stdexcept>

namespace hnu_tunnel_core
{

namespace
{
bool finite_positive(double value) { return std::isfinite(value) && value > 0.0; }
}  // namespace

std::string to_string(TraversePhase phase)
{
  switch (phase) {
    case TraversePhase::kNormal: return "normal";
    case TraversePhase::kApproach: return "approach";
    case TraversePhase::kDetect: return "detect";
    case TraversePhase::kAlign: return "align";
    case TraversePhase::kFold: return "fold";
    case TraversePhase::kVerify: return "verify";
    case TraversePhase::kTraverse: return "traverse";
    case TraversePhase::kExit: return "exit";
    case TraversePhase::kUnfold: return "unfold";
    case TraversePhase::kComplete: return "complete";
    case TraversePhase::kFallback: return "fallback";
    case TraversePhase::kAbort: return "abort";
    case TraversePhase::kRestore: return "restore";
  }
  return "unknown";
}

bool TraverseTimeouts::valid() const
{
  return finite_positive(approach_s) && finite_positive(detect_s) &&
         finite_positive(align_s) && finite_positive(fold_s) &&
         finite_positive(verify_s) && finite_positive(traverse_s) &&
         finite_positive(unfold_s) && finite_positive(observation_loss_s);
}

TraverseStateMachine::TraverseStateMachine(TraverseTimeouts timeouts)
: timeouts_(timeouts)
{
  if (!timeouts_.valid()) {
    throw std::invalid_argument("all traverse timeouts must be finite and positive");
  }
}

TraverseCommand TraverseStateMachine::start(double now_s)
{
  if (!std::isfinite(now_s)) {
    return command("invalid_time");
  }
  if (phase_ != TraversePhase::kNormal && phase_ != TraversePhase::kComplete &&
      phase_ != TraversePhase::kFallback && phase_ != TraversePhase::kAbort) {
    return command("traverse_already_active");
  }
  transition(TraversePhase::kApproach, now_s);
  return command("approach_staging_pose");
}

TraverseCommand TraverseStateMachine::step(const TraverseInput & input, double now_s)
{
  if (!std::isfinite(now_s)) {
    transition(TraversePhase::kAbort, entered_at_s_);
    return command("invalid_time");
  }
  if (input.cancel_requested && phase_ != TraversePhase::kNormal &&
      phase_ != TraversePhase::kComplete && phase_ != TraversePhase::kFallback &&
      phase_ != TraversePhase::kAbort) {
    transition(TraversePhase::kAbort, now_s);
    return command("cancel_requested");
  }

  switch (phase_) {
    case TraversePhase::kNormal:
      return command("idle");
    case TraversePhase::kApproach:
      if (input.at_staging_pose) {
        transition(TraversePhase::kDetect, now_s);
        return command("at_staging_pose");
      }
      if (expired(now_s, timeouts_.approach_s)) {
        transition(TraversePhase::kFallback, now_s);
        return command("approach_timeout");
      }
      return command("approaching_staging_pose");
    case TraversePhase::kDetect:
      if (input.observation_safe) {
        transition(TraversePhase::kAlign, now_s);
        return command("observation_safe");
      }
      if (expired(now_s, timeouts_.detect_s)) {
        transition(TraversePhase::kFallback, now_s);
        return command("detect_timeout_or_unsafe");
      }
      return command("awaiting_safe_observation");
    case TraversePhase::kAlign:
      if (input.aligned && input.observation_available && !input.observation_safe) {
        transition(TraversePhase::kAbort, now_s);
        return command("unsafe_before_fold");
      }
      if (input.aligned && input.observation_available && input.observation_safe) {
        transition(TraversePhase::kFold, now_s);
        return command("aligned");
      }
      if (expired(now_s, timeouts_.align_s)) {
        transition(TraversePhase::kFallback, now_s);
        return command("align_timeout");
      }
      return command("aligning");
    case TraversePhase::kFold:
      if (input.fold_fault) {
        transition(TraversePhase::kAbort, now_s);
        return command("fold_fault");
      }
      if (input.fold_target && input.fold_completed) {
        transition(TraversePhase::kVerify, now_s);
        return command("fold_complete");
      }
      if (expired(now_s, timeouts_.fold_s)) {
        transition(TraversePhase::kAbort, now_s);
        return command("fold_timeout");
      }
      return command("awaiting_fold_completion");
    case TraversePhase::kVerify:
      if (input.fold_fault) {
        transition(TraversePhase::kAbort, now_s);
        return command("fold_fault");
      }
      if (input.observation_safe) {
        transition(TraversePhase::kTraverse, now_s);
        return command("post_fold_observation_safe");
      }
      if (expired(now_s, timeouts_.verify_s)) {
        transition(TraversePhase::kFallback, now_s);
        return command("verify_timeout_or_unsafe");
      }
      return command("awaiting_post_fold_observation");
    case TraversePhase::kTraverse:
      if (!input.observation_available) {
        if (!observation_missing_) {
          observation_missing_=true;observation_missing_at_s_=now_s;
        }
        if (now_s-observation_missing_at_s_>timeouts_.observation_loss_s) {
          transition(TraversePhase::kAbort,now_s);
          return command("observation_unavailable_timeout");
        }
        auto stopped=command("waiting_for_observation");
        stopped.request_stop=true;stopped.allow_traverse=false;
        return stopped;
      }
      observation_missing_=false;
      if (!input.observation_safe) {
        transition(TraversePhase::kAbort, now_s);
        return command("unsafe_during_traverse");
      }
      if (input.exited_tunnel) {
        transition(TraversePhase::kExit, now_s);
        return command("tunnel_exit_detected");
      }
      if (expired(now_s, timeouts_.traverse_s)) {
        transition(TraversePhase::kAbort, now_s);
        return command("traverse_timeout");
      }
      return command("traversing");
    case TraversePhase::kExit:
      transition(TraversePhase::kUnfold, now_s);
      return command("request_unfold");
    case TraversePhase::kUnfold:
      if (input.fold_fault) {
        transition(TraversePhase::kAbort, now_s);
        return command("unfold_fault");
      }
      if (!input.fold_target && input.fold_completed) {
        transition(input.require_restore ? TraversePhase::kRestore : TraversePhase::kComplete, now_s);
        return command("unfold_complete");
      }
      if (expired(now_s, timeouts_.unfold_s)) {
        transition(TraversePhase::kAbort, now_s);
        return command("unfold_timeout");
      }
      return command("awaiting_unfold_completion");
    case TraversePhase::kRestore:
      if(input.restore_completed){
        transition(TraversePhase::kComplete,now_s);return command("restore_complete");
      }
      if(expired(now_s,timeouts_.align_s)){
        transition(TraversePhase::kAbort,now_s);return command("restore_timeout");
      }
      return command("restoring_normal_clearance");
    case TraversePhase::kComplete:
    case TraversePhase::kFallback:
    case TraversePhase::kAbort:
      return command("terminal");
  }
  transition(TraversePhase::kAbort, now_s);
  return command("unknown_phase");
}

TraverseCommand TraverseStateMachine::command(const std::string & reason) const
{
  const bool owns_motion = phase_ == TraversePhase::kDetect ||
                           phase_ == TraversePhase::kAlign ||
                           phase_ == TraversePhase::kFold ||
                           phase_ == TraversePhase::kVerify ||
                           phase_ == TraversePhase::kTraverse ||
                           phase_ == TraversePhase::kExit ||
                           phase_ == TraversePhase::kUnfold ||
                           phase_ == TraversePhase::kRestore ||
                           phase_ == TraversePhase::kAbort;
  const bool stop = phase_ == TraversePhase::kDetect || phase_ == TraversePhase::kFold ||
                    phase_ == TraversePhase::kVerify || phase_ == TraversePhase::kUnfold ||
                    phase_ == TraversePhase::kFallback || phase_ == TraversePhase::kAbort;
  return {phase_, owns_motion, stop, phase_ == TraversePhase::kFold,
          phase_ == TraversePhase::kUnfold, phase_ == TraversePhase::kTraverse, reason};
}

void TraverseStateMachine::transition(TraversePhase next, double now_s)
{
  observation_missing_=false;
  phase_ = next;
  entered_at_s_ = now_s;
}

bool TraverseStateMachine::expired(double now_s, double timeout_s) const
{
  return now_s - entered_at_s_ > timeout_s;
}

}  // namespace hnu_tunnel_core
