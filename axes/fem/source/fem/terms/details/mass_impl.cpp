#include "mass_impl.hpp"

#include "ax/core/buffer/copy.hpp"
#include "ax/core/buffer/create_buffer.hpp"
#include "ax/fem/elements/p1.hpp"

namespace ax::fem::details {

template <int dim>
static void fillin_1dof(ConstRealBufferView v, ConstSizeBufferView e, ConstRealBufferView density,
                        std::vector<int>& row, std::vector<int>& col, std::vector<Real>& val) {
  using Element = elements::P1Element<dim>;
  size_t nnz = e.Shape().Y() * (dim + 1) * (dim + 1);
  row.reserve(nnz);
  col.reserve(nnz);
  val.reserve(nnz);
  for (size_t i = 0; i < e.Shape().Y(); ++i) {
    using mapped = Eigen::Map<const math::RealVector<dim>>;
    std::array<math::RealVector<dim>, dim + 1> vert;
    for (size_t k = 0; k <= dim; ++k) {
      vert[k] = mapped(v.Offset(0, e(k, i)));
    }
    Element p1(vert);
    math::RealMatrix<dim + 1, dim + 1> element_mass;
    for (Index k = 0; k <= dim; ++k) {
      for (Index l = 0; l <= dim; ++l) {
        element_mass(k, l) = p1.Integrate_F_F(k, l) * density(i);
      }
    }

    // make spsd
    auto [eigen_vectors, eigen_values] = math::eig(element_mass);
    eigen_values = eigen_values.cwiseMax(0);
    element_mass.noalias() = eigen_vectors * eigen_values.asDiagonal() * eigen_vectors.transpose();

    for (size_t k = 0; k <= dim; ++k) {
      for (size_t l = 0; l <= dim; ++l) {
        row.push_back(static_cast<math::SparseIndex>(e(k, i)));
        col.push_back(static_cast<math::SparseIndex>(e(l, i)));
        val.push_back(element_mass(static_cast<Index>(k), static_cast<Index>(l)));
      }
    }
  }
}

void fillin_2d_in_3d(ConstRealBufferView v, ConstSizeBufferView e, ConstRealBufferView density,
                     std::vector<int>& row, std::vector<int>& col, std::vector<Real>& val) {
  using Element = elements::P1Element<2>;
  size_t nnz = e.Shape().Y() * 9;
  row.reserve(nnz);
  col.reserve(nnz);
  val.reserve(nnz);
  for (size_t i = 0; i < e.Shape().Y(); ++i) {
    using mapped = Eigen::Map<const math::RealVector<3>>;
    std::array<math::RealVector<3>, 3> vert;
    for (size_t k = 0; k < 3; ++k) {
      vert[k] = mapped(v.Offset(0, e(k, i)));
    }
    // map to the 2d plane.
    std::array<math::RealVector2, 3> vert_2d;
    // the first is 00
    vert_2d[0].setZero();
    math::RealVector3 v21 = vert[1] - vert[0], v31 = vert[2] - vert[0];
    // the second is unit x.
    vert_2d[1].x() = v21.norm();
    // the third
    vert_2d[2].x() = v31.dot(v21) / v21.norm();
    vert_2d[2].y() = std::sqrt(v31.squaredNorm() - vert_2d[2].x() * vert_2d[2].x());

    Element p1(vert_2d);
    math::RealMatrix<3, 3> element_mass;
    for (Index k = 0; k < 3; ++k) {
      for (Index l = 0; l < 3; ++l) {
        element_mass(k, l) = p1.Integrate_F_F(k, l) * density(i);
      }
    }

    // make spsd
    auto [eigen_vectors, eigen_values] = math::eig(element_mass);
    eigen_values = eigen_values.cwiseMax(0);
    element_mass.noalias() = eigen_vectors * eigen_values.asDiagonal() * eigen_vectors.transpose();

    for (size_t k = 0; k < 3; ++k) {
      for (size_t l = 0; l < 3; ++l) {
        row.push_back(static_cast<math::SparseIndex>(e(k, i)));
        col.push_back(static_cast<math::SparseIndex>(e(l, i)));
        val.push_back(element_mass(static_cast<Index>(k), static_cast<Index>(l)));
      }
    }
  }
}

math::RealBlockMatrix compute_mass_matrix_host(const Mesh& mesh, ConstRealBufferView density,
                                               size_t ndof) {
  size_t dim = mesh.GetVertices()->Shape().X();
  size_t n_vert = mesh.GetVertices()->Shape().Y();
  auto vb = create_buffer<Real>(BufferDevice::Host, mesh.GetVertices()->Shape());
  auto eb = create_buffer<size_t>(BufferDevice::Host, mesh.GetElements()->Shape());

  auto [v, e] = make_view(vb, eb);
  copy(v, mesh.GetVertices()->ConstView());
  copy(e, mesh.GetElements()->ConstView());

  math::RealBlockMatrix result(n_vert, n_vert, ndof, mesh.Device());

  std::vector<int> row, col;
  std::vector<Real> val;

  size_t n_vertex_per_element = e.Shape().X();
  if (n_vertex_per_element != dim + 1) {
    if (dim == 3 && n_vertex_per_element == 3) {
      fillin_2d_in_3d(v, e, density, row, col, val);
    } else {
      AX_THROW_RUNTIME_ERROR("MassTerm: Only support 2D triangle in 3D. got {}",
                             n_vertex_per_element);
    }
  } else if (dim == 2) {
    fillin_1dof<2>(v, e, density, row, col, val);
  } else if (dim == 3) {
    fillin_1dof<3>(v, e, density, row, col, val);
  } else {
    AX_THROW_RUNTIME_ERROR("MassTerm: Only support 2D and 3D. got {}", dim);
  }

  if (ndof == 1) {
    result.SetFromBlockedTriplets(view_from_buffer(row), view_from_buffer(col),
                                  view_from_buffer(val, {1, 1, val.size()}));
  } else {
    // do kronecker.
    std::vector<Real> kronecker_val;
    kronecker_val.reserve(ndof * ndof * val.size());
    for (double elem : val) {
      for (size_t j = 0; j < ndof; ++j) {
        for (size_t i = 0; i < ndof; ++i) {
          kronecker_val.push_back(j == i ? elem : 0);
        }
      }
    }

    result.SetFromBlockedTriplets(view_from_buffer(row), view_from_buffer(col),
                                  view_from_buffer(kronecker_val, {ndof, ndof, val.size()}));
  }

  return result;
}

}  // namespace ax::fem::details