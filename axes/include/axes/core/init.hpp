#pragma once

#include <functional>
#include "common.hpp"
namespace ax {

// Initialize the core module.
void init(int argc, char** argv);

// Clean up
void clean_up();

// It is possible to hook a function to be called before the program starts.
void hook_init(const char* name, std::function<Status()> f);

// It is possible to hook a function to be called before the program exits.
void hook_clean_up(const char* name, std::function<Status()> f);

}  // namespace ax
