#include "ax/utils/asset.hpp"

#include <filesystem>

namespace ax::utils {

std::string get_asset_dir() { 
  return get_root_dir() + "/asset";
}

std::string get_root_dir() {
  static std::string root_dir = AX_ROOT_DIR;
  return root_dir;
}

std::string get_asset(std::string sub_path) { return get_asset_dir() + sub_path; }

std::vector<std::string> discover_assets(std::string sub_path) {
  std::vector<std::string> assets;
  for (const auto& p : std::filesystem::directory_iterator(get_asset(sub_path))) {
    // Remove the directory name:
    auto path = p.path().string();
    path = path.substr(get_asset_dir().size());
    assets.push_back(path);
  }
  return assets;
}

}  // namespace ax::utils
