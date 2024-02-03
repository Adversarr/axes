#include "axes/utils/asset.hpp"

namespace ax::utils {

std::string get_asset_dir() { return AX_ASSET_DIR; }

std::string get_asset(std::string sub_path) { return AX_ASSET_DIR + sub_path; }

}  // namespace ax::utils
