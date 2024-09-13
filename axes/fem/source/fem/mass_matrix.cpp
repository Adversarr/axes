#include "ax/fem/mass_matrix.hpp"

#include "ax/fem/elements/p1.hpp"

namespace ax::fem {

// computes Integrate[u_i, u_j] for i, j = 0,...,dim
// Take the result from the p12_element_f_f array
template <int dim>
static math::RealMatrix<dim + 1, dim + 1> p1_e(const elements::P1Element<dim> E, Real density) {
  math::RealMatrix<dim + 1, dim + 1> result;
  for (Index i = 0; i <= dim; ++i) {
    for (Index j = 0; j <= dim; ++j) {
      result(i, j) = E.Integrate_F_F(i, j) * density;
    }
  }

  // NOTE: Make positive definite.
  auto [eigen_vectors, eigen_values] = math::eig(result);
  eigen_values = eigen_values.cwiseMax(0);
  result = eigen_vectors * eigen_values.asDiagonal() * eigen_vectors.transpose();
  return result;
}

template <int dim> math::RealSparseMatrix MassMatrixCompute<dim>::operator()(Real density) {
  math::RealSparseCOO result;
  for (auto const& ijk : mesh_) {
    std::array<math::RealVector<dim>, dim + 1> vert;
    for (Index i = 0; i <= dim; ++i) {
      vert[static_cast<size_t>(i)] = mesh_.GetVertex(ijk[i]);
    }
    elements::P1Element<dim> E(vert);
    auto element_mass = p1_e<dim>(E, density);
    for (Index i = 0; i <= dim; ++i) {
      for (Index j = 0; j <= dim; ++j) {
        result.push_back({ijk[i], ijk[j], element_mass(i, j)});
      }
    }
  }
  return math::make_sparse_matrix(mesh_.GetNumVertices(), mesh_.GetNumVertices(), result);
}

template <int dim>
math::RealSparseMatrix MassMatrixCompute<dim>::operator()(math::RealField1 const& density) {
  math::RealSparseCOO result;
  for (Index i = 0; i < mesh_.GetElements().cols(); ++i) {
    const auto& ijk = mesh_.GetElement(i);
    std::array<math::RealVector<dim>, dim + 1> vert;
    for (Index i = 0; i <= dim; ++i) {
      vert[static_cast<size_t>(i)] = mesh_.GetVertex(ijk[i]);
    }
    elements::P1Element<dim> E(vert);
    math::RealMatrix<dim + 1, dim + 1> element_mass;
    element_mass = p1_e<dim>(E, density(i));
    for (Index i = 0; i <= dim; ++i) {
      for (Index j = 0; j <= dim; ++j) {
        result.push_back({ijk[i], ijk[j], element_mass(i, j)});
      }
    }
  }
  return math::make_sparse_matrix(mesh_.GetNumVertices(), mesh_.GetNumVertices(), result);
}

template class MassMatrixCompute<2>;
template class MassMatrixCompute<3>;

}  // namespace ax::fem
