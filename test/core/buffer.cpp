#include <doctest/doctest.h>

#include "ax/core/buffer/buffer_view.hpp"
#include "ax/core/buffer/copy.hpp"
#include "ax/core/buffer/eigen_support.hpp"
#include "ax/core/buffer/for_each.hpp"
#include "ax/core/buffer/host_buffer.hpp"
#include "ax/core/buffer/transform_reduce.hpp"
#include "ax/utils/ndrange.hpp"

using namespace ax;

TEST_CASE("Host buffer") {
  SUBCASE("3d") {
    auto buffer_ptr = HostBuffer<int>::Create({2, 3, 4});
    HostBuffer<int>& buffer = static_cast<HostBuffer<int>&>(*buffer_ptr);
    // For this buffer, the strides are 1, 2, 6 but we count in bytes:
    // sizeof(int) = 4, so the strides are 4, 8, 24.
    CHECK(buffer.Stride().X() == sizeof(int));
    CHECK(buffer.Stride().Y() == 2 * sizeof(int));
    CHECK(buffer.Stride().Z() == 6 * sizeof(int));

    CHECK(buffer.Size() == 24);
    CHECK(buffer.PhysicalSize() == 24 * sizeof(int));
    CHECK(buffer.GetUnderlying().size() == 24);
    CHECK(buffer.GetUnderlying().data() == buffer.Data());

    for (int i = 0; i < 24; ++i) {
      buffer.Data()[i] = i;
    }

    auto view = buffer.View();
    for (auto [i, j, k] : utils::ndrange<int>(2, 3, 4)) {
      CHECK(view(i, j, k) == i + j * 2 + k * 6);
    }
  }

  SUBCASE("2d") {
    auto buffer_ptr = HostBuffer<int>::Create({3, 3});
    HostBuffer<int>& buffer = static_cast<HostBuffer<int>&>(*buffer_ptr);
    // For this buffer, the strides are 1, 3 but we count in bytes:
    // sizeof(int) = 4, so the strides are 4, 12.
    CHECK(buffer.Stride().X() == sizeof(int));
    CHECK(buffer.Stride().Y() == 3 * sizeof(int));

    CHECK(buffer.Size() == 9);
    CHECK(buffer.PhysicalSize() == 9 * sizeof(int));
    CHECK(buffer.GetUnderlying().size() == 9);
    CHECK(buffer.GetUnderlying().data() == buffer.Data());

    for (int i = 0; i < 9; ++i) {
      buffer.Data()[i] = i;
    }

    auto view = buffer.View();
    for (auto [i, j] : utils::ndrange<int>(3, 3)) {
      CHECK(view(i, j) == i + j * 3);
    }
  }

  SUBCASE("1d") {
    auto buffer_ptr = HostBuffer<int>::Create(3);
    HostBuffer<int>& buffer = static_cast<HostBuffer<int>&>(*buffer_ptr);
    // For this buffer, the strides are 1 but we count in bytes:
    // sizeof(int) = 4, so the strides are 4.
    CHECK(buffer.Stride().X() == sizeof(int));

    CHECK(buffer.Size() == 3);
    CHECK(buffer.PhysicalSize() == 3 * sizeof(int));
    CHECK(buffer.GetUnderlying().size() == 3);
    CHECK(buffer.GetUnderlying().data() == buffer.Data());

    for (int i = 0; i < 3; ++i) {
      buffer.Data()[i] = i;
    }

    auto view = buffer.View();
    for (auto i : utils::range<int>(3)) {
      CHECK(view(i) == i);
    }
  }
}

TEST_CASE("Buffer View External") {
  std::vector<int> data(24);
  for (int i = 0; i < 24; ++i) {
    data[i] = i;
  }

  Dim3 shape{2, 3, 4};
  Dim3 strides{4, 8, 24};
  BufferView<int> view(data.data(), shape, strides, BufferDevice::Host);
  for (auto [i, j, k] : utils::ndrange<int>(2, 3, 4)) {
    CHECK(view(i, j, k) == i + j * 2 + k * 6);
  }
}

TEST_CASE("Map to Eigen") {
  SUBCASE("continuous") {
    auto buffer_ptr = HostBuffer<Real>::Create({2, 3});
    for_each(buffer_ptr->View(), [v = 0](Real& x) mutable {
      x = static_cast<Real>(v++);
    });

    auto map = view_as_matrix_full<math::RealMatrixX>(buffer_ptr->View());
    CHECK(map.rows() == 2);
    CHECK(map.cols() == 3);
    for (int j = 0; j < 3; ++j) {
      for (int i = 0; i < 2; ++i) {
        CHECK(map(i, j) == i + j * 2);
      }
    }
  }

  SUBCASE("dynamic") {
    auto buffer_ptr = HostBuffer<Real>::Create({2, 3});
    for_each(buffer_ptr->View(), [v = 0](Real& x) mutable {
      x = static_cast<Real>(v++);
    });

    auto map = view_as_matrix_full<math::RealMatrixX, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>(
        buffer_ptr->View());
    CHECK(map.rows() == 2);
    CHECK(map.cols() == 3);
    for (int j = 0; j < 3; ++j) {
      for (int i = 0; i < 2; ++i) {
        CHECK(map(i, j) == i + j * 2);
      }
    }
  }

  SUBCASE("map vector buffer") {
    auto bp = HostBuffer<math::RealVector3>::Create(4);  // r, c = 3, 4
    auto mapped = view_as_matrix_full<math::RealField3>(bp->View());
  }
}

TEST_CASE("for_each") {
  SUBCASE("Sequential") {
    auto buf1 = HostBuffer<int>::Create({2, 3, 4});
    auto buf2 = HostBuffer<int>::Create({2, 3, 4});
    auto buf3 = HostBuffer<int>::Create({2, 3, 4});

    for_each(make_view(buf1, buf2, buf3), [](int& x, int& y, int& z) {
      x = 1;
      y = 2;
      z = 3;
    });

    for (auto [i, j, k] : utils::ndrange<int>(2, 3, 4)) {
      CHECK(buf1->View()(i, j, k) == 1);
      CHECK(buf2->View()(i, j, k) == 2);
      CHECK(buf3->View()(i, j, k) == 3);
    }
  }

  SUBCASE("Parallel") {
    auto buf1 = HostBuffer<int>::Create({2, 3, 4});
    auto buf2 = HostBuffer<int>::Create({2, 3, 4});
    auto buf3 = HostBuffer<int>::Create({2, 3, 4});

    par_for_each(make_view(buf1, buf2, buf3), [](int& x, int& y, int& z) {
      x = 1;
      y = 2;
      z = 3;
    });
  }
}

TEST_CASE("transform_reduce") {
  SUBCASE("sequential") {
    auto buf1 = HostBuffer<int>::Create({2, 3, 4});
    auto buf2 = HostBuffer<int>::Create({2, 3, 4});
    auto buf3 = HostBuffer<int>::Create({2, 3, 4});

    for_each(make_view(buf1, buf2, buf3), [](int& x, int& y, int& z) {
      x = 1;
      y = 2;
      z = 3;
    });

    auto result = ax::transform_reduce(
        make_view(buf1, buf2, buf3), 0,
        [](int x, int y, int z) {
          return x + y + z;
        },
        std::plus<int>());

    // (1 + 2 + 3) * (2 * 3 * 4) = 6 * 24 = 144
    CHECK(result == 144);
  }

  SUBCASE("parallel") {
    auto buf1 = HostBuffer<int>::Create({2, 3, 4});
    auto buf2 = HostBuffer<int>::Create({2, 3, 4});
    auto buf3 = HostBuffer<int>::Create({2, 3, 4});

    par_for_each(make_view(buf1, buf2, buf3), [](int& x, int& y, int& z) {
      x = 1;
      y = 2;
      z = 3;
    });

    auto result = ax::par_transform_reduce(
        make_view(buf1, buf2, buf3), 0,
        [](int x, int y, int z) {
          return x + y + z;
        },
        std::plus<int>());

    // (1 + 2 + 3) * (2 * 3 * 4) = 6 * 24 = 144
    CHECK(result == 144);
  }
}

TEST_CASE("type cast") {
  auto bp = HostBuffer<math::RealVector3>::Create({2, 3});
  SUBCASE("non-const to const") {
    auto non_const = bp->View();
    BufferView<const math::RealVector3> const_view = non_const;

    for_each(std::make_tuple(non_const, const_view), [](auto& x, auto& y) {
      static_assert(std::is_const_v<std::remove_reference_t<decltype(y)>>, "What");
      static_assert(!std::is_const_v<std::remove_reference_t<decltype(x)>>, "What");

      x.setOnes();
      CHECK(y.sum() == 3);
    });
  }

  SUBCASE("view_as_matrix") {
    math::RealField3 field(3, 10);  // 3x10 matrix of Real.

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 10; ++j) {
        field(i, j) = i + j;
      }
    }

    auto view = view_from_matrix(field);
    CHECK(view.Shape() == Dim3(3, 10, 0));

    for_each_indexed(Dim{3, 10}, [view](size_t i, size_t j) {
      CHECK(view(i, j) == i + j);
    });
  }

  SUBCASE("view_as_scalar") {
    auto scalar323 = view_as_scalar(bp->View());
    CHECK(scalar323.Shape() == Dim3(3, 2, 3));
    CHECK(scalar323.Stride() == Dim3{sizeof(Real), 3 * sizeof(Real), 6 * sizeof(Real)});
    for_each_indexed(Dim{2, 3}, [bv = bp->View()](size_t j, size_t k) mutable {
      for (size_t i = 0; i < 3; ++i) {
        bv(j, k)(i) = k + j + i;
      }
    });

    for_each_indexed(Dim{3, 2, 3}, [scalar323](size_t i, size_t j, size_t k) {
      CHECK(scalar323(i, j, k) == (i + j + k));
    });
  }
}

TEST_CASE("buffer copy") {
  auto buf1 = HostBuffer<int>::Create({2, 3, 4});
  auto buf2 = HostBuffer<int>::Create({2, 3, 4});

  for_each(buf1->View(), [v = 0](int& x) mutable {
    x = v++;
  });

  copy(buf2->View(), buf1->ConstView());

  for (auto [i, j, k] : utils::ndrange<int>(2, 3, 4)) {
    CHECK(buf1->View()(i, j, k) == buf2->View()(i, j, k));
  }
}


TEST_CASE("ViewAsEigenMap") {
  SUBCASE("1d") {
    auto buf = HostBuffer<Real>::Create({2, 3});
    for_each(buf->View(), [v = 0](Real& x) mutable {
      x = static_cast<Real>(v++);
    });

    // 2x3 buffer, view as 3 of 2x1 vectors.
    auto map = view_as_matrix_1d<Real, 2>(buf->View());
    for_each_indexed(Dim{2, 3}, [map, &buf](size_t i, size_t j) {
      int lid = linear_index(buf->Shape(), {i, j});
      CHECK(map(j)(i) == buf->View()(i, j));
    });
  }

  SUBCASE("2d") {
    auto buf = HostBuffer<Real>::Create({2, 3, 4});
    for_each(buf->View(), [v = 0](Real& x) mutable {
      x = static_cast<Real>(v++);
    });

    // 2x3x4 buffer, view as 4 of 2x3 matrices.
    auto map = view_as_matrix_2d<Real, 2, 3>(buf->View());
    for_each_indexed(buf->Shape(), [map, &buf](size_t i, size_t j, size_t k) {
      int lid = linear_index(buf->Shape(), {i, j, k});
      CHECK(map(k)(i, j) == buf->View()(i, j, k));
    });
  }
}

// TEST_CASE();