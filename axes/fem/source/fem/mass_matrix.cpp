#include "ax/fem/mass_matrix.hpp"
#include "ax/fem/elements/p1.hpp"

namespace ax::fem {

// computes Integrate[u_i, u_j] for i, j = 0,...,dim
// Take the result from the p12_element_f_f array
template <idx dim> static math::matr<dim + 1, dim + 1> p1_e(const elements::P1Element<dim> E,
                                                            real density) {
  math::matr<dim + 1, dim + 1> result;
  for (idx i = 0; i <= dim; ++i) {
    for (idx j = 0; j <= dim; ++j) {
      result(i, j) = E.Integrate_F_F(i, j) * density;
    }
  }
  return result;
}

template <idx dim> static math::sp_coeff_list p1(MeshBase<dim> const& mesh_, real uniform_density) {
  // Foreach Element: compute p1_e.
  math::sp_coeff_list result;
  for (auto ijk : mesh_) {
    std::array<math::vecr<dim>, dim + 1> vert;
    for (idx i = 0; i <= dim; ++i) {
      vert[i] = mesh_.GetVertex(ijk[i]);
    }
    elements::P1Element<dim> E(vert);
    auto element_mass = p1_e(E, uniform_density);
    for (idx i = 0; i <= dim; ++i) {
      for (idx j = 0; j <= dim; ++j) {
        // Foreach vertex, we have dim components.
        for (idx D = 0; D < dim; ++D) {
          result.push_back({D + dim * ijk[i], D + dim * ijk[j], element_mass(i, j)});
        }
      }
    }
  }
  return result;
}

template <idx dim> static math::sp_coeff_list p1(MeshBase<dim> const& mesh_,
                                                 math::field1r const& density) {
  // Foreach Element: compute p1_e.
  math::sp_coeff_list result;
  for (idx i = 0; i < mesh_.GetElements().cols(); ++i) {
    const auto& ijk = mesh_.GetElement(i);
    std::array<math::vecr<dim>, dim + 1> vert;
    for (idx i = 0; i <= dim; ++i) {
      vert[i] = mesh_.GetVertex(ijk[i]);
    }
    elements::P1Element<dim> E(vert);
    math::matr<dim + 1, dim + 1> element_mass;
    element_mass = p1_e(E, density(i));
    for (idx i = 0; i <= dim; ++i) {
      for (idx j = 0; j <= dim; ++j) {
        result.push_back({ijk[i], ijk[j], element_mass(i, j)});
      }
    }
  }
  return result;
}

template <idx dim> math::sp_coeff_list MassMatrixCompute<dim>::operator()(real density) {
  if (mesh_->GetType() == MeshType::kP1) {
    return p1(*mesh_, density);
  } else {
    AX_CHECK(false) << "Unsupported mesh type";
  }
  AX_UNREACHABLE();
}

template <idx dim>
math::sp_coeff_list MassMatrixCompute<dim>::operator()(math::field1r const& density) {
  if (mesh_->GetType() == MeshType::kP1) {
    return p1(*mesh_, density);
  } else {
    AX_CHECK(false) << "Unsupported mesh type";
  }
  AX_UNREACHABLE();
}

template class MassMatrixCompute<2>;
template class MassMatrixCompute<3>;

}  // namespace ax::fem