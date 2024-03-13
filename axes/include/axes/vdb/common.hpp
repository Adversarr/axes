#pragma once
#include <openvdb/openvdb.h>

#include "axes/math/common.hpp"

namespace ax::vdb {

/************************* SECT: Real Scalar Grid *************************/
using RealGrid = openvdb::DoubleGrid;
using RealTree = openvdb::DoubleTree;
using RealGridPtr = RealGrid::Ptr;
using RealGridConstPtr = RealGrid::ConstPtr;
using RealTreePtr = RealTree::Ptr;
using RealTreeConstPtr = RealTree::ConstPtr;


/************************* SECT: Real Vector Field *************************/
using Vec3r = openvdb::Vec3d;
using Vec3rGrid = openvdb::Vec3DGrid;
using Vec3rGridPtr = typename Vec3rGrid::Ptr;
using Vec3rGridConstPtr = typename Vec3rGrid::ConstPtr;
using Vec3rTree = openvdb::Vec3DTree;
using Vec3rTreePtr = openvdb::Vec3DTree::Ptr;
using Vec3rTreeConstPtr = openvdb::Vec3DTree::ConstPtr;

using Coord = openvdb::Coord;
using CoordBBox = openvdb::CoordBBox;


}  // namespace ax::vdb
