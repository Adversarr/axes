#include "ax/utils/asset.hpp"

#include <absl/flags/flag.h>

#include <filesystem>

ABSL_FLAG(std::string, ax_root, AX_ROOT_DIR, "Root directory for axes");

namespace ax::utils {

std::string get_asset_dir() { 
  return get_root_dir() + "/asset";
}

std::string get_root_dir() {
  static std::string root_dir = absl::GetFlag(FLAGS_ax_root);
  return root_dir;
}

std::string get_asset(std::string sub_path) { return get_asset_dir() + sub_path; }

std::vector<std::string> discover_assets(std::string sub_path) {
  std::vector<std::string> assets;
  for (auto& p : std::filesystem::directory_iterator(get_asset(sub_path))) {
    // Remove the directory name:
    auto path = p.path().string();
    path = path.substr(get_asset_dir().size());
    assets.push_back(path);
  }
  return assets;
}

}  // namespace ax::utils
