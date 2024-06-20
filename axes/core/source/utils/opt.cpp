#include "ax/utils/opt.hpp"

#include "ax/core/echo.hpp"

namespace ax::utils {

utils::Opt Tunable::GetOptions() const { return {}; }

void Tunable::SetOptions(utils::Opt const&) {}

}  // namespace ax::utils
