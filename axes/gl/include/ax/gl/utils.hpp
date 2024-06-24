#pragma once

#include "ax/core/status.hpp"
namespace ax::gl {

void init(bool is_registering = true);
void init(int argc, char** argv);

Status enter_main_loop();

}
