#include "ax/fem/poisson/poisson.hpp"
#include "ax/fem/laplace_matrix.hpp"
#include "ax/core/excepts.hpp"

namespace ax::fem {

template <idx dim>
math::field1r PoissonSolver<dim>::Solve(PoissonProblemData<dim> const& data) {
  AssembledPoissonProblemData assembled_data = Assemble(data);
  return Solve(assembled_data);
}

template <idx dim>
AssembledPoissonProblemData PoissonSolver<dim>::Assemble(PoissonProblemData<dim> const& data) {
  if (! mesh_) {
    throw LogicError("The mesh is not set.");
  }

  // Check the size of the fields.
  idx n_elem = mesh_->GetNumElements(), n_vert = mesh_->GetNumVertices();
  if (data.f_.size() != n_vert) {
    throw InvalidArgument("The size of the source field does not match the number of vertices.");
  }
  if (data.g_.size() != n_vert) {
    throw InvalidArgument("The size of the Dirichlet boundary condition field does not match the number of vertices.");
  }
  if (data.h_.size() != n_vert) {
    throw InvalidArgument("The size of the Neumann boundary condition field does not match the number of vertices.");
  }
  if (data.s_.size() != n_vert) {
    throw InvalidArgument("The size of the Neumann boundary condition field does not match the number of vertices.");
  }
  if (data.dirichlet_bc_vertices_.size() != n_vert) {
    throw InvalidArgument("The size of the Dirichlet boundary condition field does not match the number of vertices.");
  }
  if (data.neumann_bc_vertices_.size() != n_vert) {
    throw InvalidArgument("The size of the Neumann boundary condition field does not match the number of vertices.");
  }

  // Assemble the Discrete Poisson.
  LaplaceMatrixCompute<dim> laplacian(*mesh_);
  math::sp_matxxr A;
  if (data.a_.size() == 0) {
    A = laplacian(1.0);
  } else if (data.a_.size() == 1) {
    A = laplacian(data.a_[0]);
  } else {
    if (data.a_.size() != mesh_->GetNumElements()) {
      throw InvalidArgument("The size of the diffusion coefficient field does not match the number of elements.");
    }
    A = laplacian(data.a_);
  }

  math::sp_coeff_list neumann_bc;
  for (idx i = 0; i < data.neumann_bc_vertices_.size(); ++i) {
    if (data.neumann_bc_vertices_[i]) {
      // TODO: Process Neumann BC. involves the bilinear-form, and linear-form
    }
  }

  AssembledPoissonProblemData R;
  R.A_ = laplacian + math::make_sparse_matrix(n_vert, n_vert, neumann_bc);
  R.b_ = math::vecxr::Zero(n_vert);

  // Rhs.
  for (idx i = 0; i < data.f_.size(); ++i) {
    // TODO: compute rhs.
  }

  for (idx i = 0; i < data.dirichlet_bc_vertices_.size(); ++i) {
    if (data.dirichlet_bc_vertices_[i]) {
      // TODO: Process Dirichlet BC. Remove all rebundant rows and columns, and modified the right-hand side.
    }
  }
}

}
