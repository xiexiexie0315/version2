#pragma once

#include <string>

namespace hnu2027_sentry_behavior {

// The behavior layer intentionally treats every terminal result other than an
// explicit complete as a failure, even if a transport layer reported success.
bool is_complete_success(bool action_succeeded, const std::string& terminal_phase);

}  // namespace hnu2027_sentry_behavior
