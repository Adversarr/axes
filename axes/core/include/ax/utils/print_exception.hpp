#pragma once

#include <exception>

#include "ax/core/logging.hpp"

namespace ax::utils {

/**
 * @brief Print the nested exception.
 * @param e The exception to print.
 */
void print_nested_exception(std::exception const& e) noexcept;

/**
 *
 * @param e
 */
void log_nested_exception(std::exception const& e, loglvl level = loglvl::critical) noexcept;

}  // namespace ax::utils