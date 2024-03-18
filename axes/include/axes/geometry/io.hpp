#pragma once
#include "common.hpp"

namespace ax::geo {

/**
 * @brief Reads an OBJ file and returns a SurfaceMesh object.
 *
 * This function reads an OBJ file from the specified path and converts it into a SurfaceMesh
 * object.
 *
 * @param path The path to the OBJ file.
 * @return A StatusOr object containing the SurfaceMesh if the file was successfully read, or an
 * error message if there was a problem.
 */
StatusOr<SurfaceMesh> read_obj(std::string const& path);

}  // namespace ax::geo