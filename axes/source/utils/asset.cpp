#include "axes/utils/asset.hpp"
#include <absl/flags/flag.h>

ABSL_FLAG(std::string, ax_asset_dir, AX_ASSET_DIR, "Asset directory");

namespace ax::utils {

std::string get_asset_dir() { 
  static std::string asset_dir = absl::GetFlag(FLAGS_ax_asset_dir);
  return asset_dir;
}

std::string get_asset(std::string sub_path) { return get_asset_dir() + sub_path; }

}  // namespace ax::utils
