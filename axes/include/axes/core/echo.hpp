#pragma once
#include <absl/log/check.h>        // IWYU pragma: export
#include <absl/log/die_if_null.h>  // IWYU pragma: export
#include <absl/log/log.h>          // IWYU pragma: export

#define AX_DIE_IF_NULL(...) ABSL_DIE_IF_NULL(__VA_ARGS__)

namespace ax {

namespace fmt {
using namespace fmt;
}

}  // namespace ax
