#include "ax/utils/opt.hpp"

#include "ax/core/echo.hpp"

namespace ax::utils {

utils::Opt Tunable::GetOptions() const { return {}; }
Status Tunable::SetOptions(utils::Opt const&) { AX_RETURN_OK(); }

}  // namespace ax::utils
