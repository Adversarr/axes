#pragma once
#include "ax/math/buffer_blas.hpp"
namespace ax::math::buffer_blas {

void do_emul_gpu(ConstRealBufferView x, RealBufferView y);

void do_ediv_gpu(ConstRealBufferView x, RealBufferView y);

}