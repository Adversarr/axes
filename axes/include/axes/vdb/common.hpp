#pragma once
#include <openvdb/openvdb.h>

#include "axes/math/common.hpp"

namespace ax::vdb {
namespace openvdb {
using namespace ::openvdb;
}
/************************* SECT: Real Scalar Grid *************************/
using RealGrid = openvdb::DoubleGrid;
using RealGridPtr = RealGrid::Ptr;
using RealGridConstPtr = RealGrid::ConstPtr;
using RealGridTree = RealGrid::TreeType;

/************************* SECT: Real Vector Field *************************/
using Vec3rGrid = openvdb::Vec3DGrid;
using VecrGridPtr = typename Vec3rGrid::Ptr;
using VecrGridConstPtr = typename Vec3rGrid::ConstPtr;

}  // namespace ax::vdb
