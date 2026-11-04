#pragma once

#include <string>

#include "hnu_tunnel_controller/velocity_command.hpp"
#include "hnu_tunnel_core/traverse_state_machine.hpp"

namespace hnu_tunnel_controller {

enum class CommandSource { kNone, kNav2, kTunnel };

struct CmdMuxConfig {
  double source_timeout_s{0.25};
  bool valid() const;
};

struct CmdMuxInput {
  VelocityCommand nav2;
  VelocityCommand tunnel;
  hnu_tunnel_core::TraverseCommand traverse;
  bool emergency_stop{false};
};

struct CmdMuxOutput {
  VelocityCommand command;
  CommandSource source{CommandSource::kNone};
  std::string reason;
};

// The only component allowed to select a command source. A tunnel-to-Nav2
// handoff always emits one explicit zero command before Nav2 resumes.
class TunnelCmdMux {
 public:
  explicit TunnelCmdMux(CmdMuxConfig config = {});
  CmdMuxOutput select(const CmdMuxInput& input, double now_s);

 private:
  bool fresh(double stamp_s, double now_s) const;
  CmdMuxConfig config_;
  CommandSource previous_source_{CommandSource::kNone};
  bool handoff_stop_pending_{false};
};

}  // namespace hnu_tunnel_controller
