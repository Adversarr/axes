#include "ax/utils/opt.hpp"

#include "ax/core/echo.hpp"

namespace ax::utils {

utils::Options Tunable::GetOptions() const { return {}; }

void Tunable::SetOptions(utils::Options const&) {}

}  // namespace ax::utils
