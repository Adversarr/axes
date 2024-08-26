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
void initialize(int argc, char** argv);
void initialize();

namespace po {
using cxxopts::Option;
using cxxopts::Options;
using cxxopts::ParseResult;

template <typename T = std::string>
Option make_option(std::string name, std::string desc, std::string default_value) {
  return Option(std::move(name), std::move(desc),
                cxxopts::value<T>()->default_value(std::move(default_value)));
}

template <>
inline Option make_option<bool>(std::string name, std::string desc, std::string default_value) {
  return Option(std::move(name), std::move(desc),
                cxxopts::value<bool>()->default_value(default_value));
}

inline Option make_option(std::string name, std::string desc) {
  return Option(std::move(name), std::move(desc), cxxopts::value<bool>());
}

Options& get_program_options();
ParseResult& get_parse_result();

template <typename T = std::string, typename... Args>
void add_option(const std::string& name, const std::string& desc, Args&&... args) {
  get_program_options().add_option(make_option<T>(name, desc, std::forward<Args>(args)...));
}

void add_option(const Option& opt);
void add_option(std::initializer_list<Option> opt_list);
}  // namespace po

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

void print_stack();

}  // namespace ax
