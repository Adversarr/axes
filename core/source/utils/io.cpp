#include "acore/utils/io.hpp"
#include <fstream>


namespace axes::utils::io {

std::vector<char> read_binary(std::string path) {
  std::ifstream input(path, std::ios::ate | std::ios::binary | std::ios::in);
  if (! input.is_open()) {
    throw std::runtime_error("Cannot open file \"" + path + "\"");
  }

  auto size = input.tellg();
  input.seekg(0, std::ios::beg);
  std::vector<char> result;
  result.resize(size);
  input.read(result.data(), size);
  return result;
}

}