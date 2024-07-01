#include "ax/utils/raw_buffer.hpp"

#include <fstream>

#include "ax/core/excepts.hpp"

namespace ax::utils {

List<char> load_istream_raw(std::istream& is) {
  List<char> buffer;
  is.seekg(0, std::ios::end);
  auto end = is.tellg();
  if (end <= 0) {
    throw std::runtime_error("Failed to read file: end < 0");
  }
  buffer.resize(static_cast<size_t>(end));
  is.seekg(0, std::ios::beg);
  is.read(buffer.data(), end);
  return buffer;
}

List<char> load_file_raw(std::string_view file_name) {
  std::ifstream file(file_name.data(), std::ios::binary);
  AX_THROW_IF_FALSE(file.is_open(), "Failed to open file: " + std::string(file_name));
  return load_istream_raw(file);
}

}  // namespace ax::utils
