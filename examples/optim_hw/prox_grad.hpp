#pragma once
#include "ax/math/common.hpp"
#include "ax/math/linalg.hpp"
using namespace ax;
using namespace math;

namespace xx {

inline math::vecxr l1_proximator(math::vecxr const & x, real const & lambda) {
    math::vecxr y = x;
    for (idx i = 0; i < y.size(); ++i) {
        if (y(i) > lambda) {
            y(i) -= lambda;
        } else if (y(i) < -lambda) {
            y(i) += lambda;
        } else {
            y(i) = 0;
        }
    }
    return y;
}

inline math::vecxr l2_proximator(math::vecxr const & x, real const & lambda) {
    return x * std::max(1 - lambda / math::norm(x), 0.0);
}

}