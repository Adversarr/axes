#pragma once

#include "ax/core/status.hpp"
namespace ax::gl {

void init();
void init(int argc, char** argv);

Status enter_main_loop();

}