#pragma once

#include <spdlog/fwd.h>

#include <cxxopts.hpp>
#include <functional>

#include "common.hpp"  // IWYU pragma: export

namespace ax {

/**
 * @brief Initialize the core module.
 *
 * This function initializes the core module of the AX library.
 * It should be called before any other AX library functions are used.
 *
 * @param argc The number of command-line arguments.
 * @param argv An array of command-line arguments.
 */
void init(int argc, char** argv);
void init();
cxxopts::Options& get_program_options();
cxxopts::ParseResult& get_parse_result();

/**
 * @brief Clean up the core module.
 *
 * This function cleans up the resources used by the AX library.
 * It should be called before the program exits.
 */
void clean_up();

/**
 * @brief Hook a function to be called before the program starts.
 *
 * This function allows you to hook a custom function to be called before the program starts.
 * The hooked function should have the signature `Status functionName()`.
 *
 * @param name The name of the hook.
 * @param f The function to be called.
 */
void add_init_hook(const char* name, std::function<void()> f);

/**
 * @brief Hook a function to be called before the program exits.
 *
 * This function allows you to hook a custom function to be called before the program exits.
 * The hooked function should have the signature `Status functionName()`.
 *
 * @param name The name of the hook.
 * @param f The function to be called.
 */
void add_clean_up_hook(const char* name, std::function<void()> f);

const char* get_program_path();

std::shared_ptr<spdlog::logger> get_logger();

}  // namespace ax
