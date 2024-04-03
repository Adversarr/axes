#pragma once

#include "ax/core/config.hpp" // IWYU pragma: export
#include "ax/core/macros.hpp" // IWYU pragma: export
#include "ax/core/status.hpp" // IWYU pragma: export
#include <vector>
#include <memory>

namespace ax {
template <typename T> using List=std::vector<T>;
template <typename T> using UPtr=std::unique_ptr<T>;
template <typename T> using Ptr=T*;
template <typename T> using SPtr=std::shared_ptr<T>;
}