#pragma once

#include <vector>

#include <filesystem>

namespace axes::utils::io {

std::vector<char> read_binary(std::string path);

}