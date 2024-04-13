#pragma once

#include <string>
#include <vector>
namespace ax::utils {

std::string get_asset_dir();

std::string get_asset(std::string sub_path);

std::vector<std::string> discover_assets(std::string sub_path);

}  // namespace ax::utils
