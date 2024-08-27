#pragma once
namespace ax::gl {

void init(bool is_registering = true);
void init(int argc, char** argv);

int enter_main_loop(bool clean_up_and_exit = true);

}
