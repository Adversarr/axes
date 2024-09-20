#pragma once
#include "ax/core/buffer/copy.hpp"
#include "ax/core/buffer/create_buffer.hpp"
#include "ax/core/buffer/eigen_support.hpp"
#include "ax/core/init.hpp"
#include "ax/fem/problem.hpp"
#include "ax/fem/terms/laplace.hpp"
#include "ax/fem/utils/prune_dbc.hpp"
#include "ax/geometry/primitives.hpp"
#include "ax/math/buffer_blas.hpp"
#include "ax/math/io.hpp"
#include "ax/math/sparse_matrix/linsys/preconditioner/block_jacobi.hpp"
#include "ax/math/sparse_matrix/linsys/preconditioner/jacobi.hpp"
#include "ax/math/sparse_matrix/linsys/solver/cg.hpp"
#include "ax/math/utils/formatting.hpp"
#include "ax/utils/time.hpp"

using namespace ax;

Real value_at(Real x, Real y) {
  return x + y - 2 * x * y;
}

Real value_at(const math::RealVector2& xy) {
  return value_at(xy(0), xy(1));
}

bool near_boundary(const math::RealVector2& xy) {
  Real x = xy.x(), y = xy.y();
  return abs(1 + x) < 1e-6 || abs(x - 1) < 1e-6 || abs(1 + y) < 1e-6 || abs(y - 1) < 1e-6;
}

// Solves a Laplace Equation on a square domain.
// The true solution is:
//      u = x (1 - y) + y (1 - x) = x + y - 2xy,
// and the right-hand side is zero (f = 0).
// The domain is the unit square [-1, 1] x [-1, 1].

// Define the problem.

int main(int argc, char** argv) {
  po::add_option({
    po::make_option<bool>("gpu", "Use gpu compute", "false"),
    po::make_option<size_t>("size", "Size of the mesh", "10"),
  });
  initialize(argc, argv);

  bool gpu = po::get_parse_result()["gpu"].as<bool>();
  size_t size = po::get_parse_result()["size"].as<size_t>();

  BufferDevice device = gpu ? BufferDevice::Device : BufferDevice::Host;
  // BufferDevice device = BufferDevice::Device;

  auto start = utils::now();

  auto mesh = std::make_shared<fem::Mesh>(2, 3, device);

  {  // Set the vertices and elements.
    auto plane = geo::plane(1, 1, size, size);
    math::RealField2 vertices = plane.vertices_.topRows<2>();
    math::Field<size_t, 3> elements = plane.indices_.cast<size_t>();
    mesh->SetData(view_from_matrix(vertices), view_from_matrix(elements));
  }
  size_t nv = mesh->GetNumVertices();
  auto state = std::make_shared<fem::State>(1, nv, device);

  auto var = state->GetVariables()->View();
  auto var_host_buf = create_buffer<Real>(BufferDevice::Host, var.Shape());
  auto col_view_buf = create_buffer<Real>(BufferDevice::Host, mesh->GetVertices()->Shape());
  auto col_view = view_as_matrix_1d<const Real, 2>(col_view_buf->View());
  auto var_host = var_host_buf->View();
  copy(col_view_buf->View(), mesh->GetVertices()->ConstView());
  copy(var_host, var);
  {  // Dirichlet b.c.
    auto bc_mark = state->GetCondition()->View();
    auto bc_mark_buf = create_buffer<fem::VariableCondition>(BufferDevice::Host, bc_mark.Shape());

    auto bc_mark_host = bc_mark_buf->View();
    for (size_t i = 0; i < nv; ++i) {
      auto xy = col_view(i);
      if (near_boundary(xy)) {
        var_host(0, i) = value_at(xy);
        bc_mark_host(0, i) = fem::VariableCondition::Dirichlet;
      }
    }
    copy(bc_mark, bc_mark_host);
    copy(var, var_host);
  }

  auto prob = std::make_shared<fem::Problem>(state, mesh);

  {  // Add the term.
    auto lap = std::make_unique<fem::LaplaceTerm>(state, mesh);
    prob->AddTerm("laplace", std::move(lap));
  }

  fem::PruneDirichletBc pruner(state);
  pruner.UpdateDbcValue();

  // Solve the problem.
  prob->InitializeHessianFillIn();
  prob->MarkDirty();
  prob->UpdateGradient();
  prob->UpdateHessian();

  // Solve the linear system.
  auto bsr = prob->GetHessian();
  auto grad = prob->GetGradient();
  auto grad_norm = math::buffer_blas::norm(grad);
  AX_INFO("Gradient norm: {}", grad_norm);

  // copy hessian to host
  // math::RealBlockMatrix hessian_host(nv, nv, 1);
  // hessian_host.SetData(bsr->RowPtrs()->ConstView(),
  //                      bsr->ColIndices()->ConstView(),
  //                      bsr->Values()->ConstView());
  // auto spm = hessian_host.ToSparseMatrix();
  // math::write_sparse_matrix("laplace.mtx", spm);

  math::buffer_blas::scal(-1, grad);
  pruner.Prune(*bsr);
  pruner.Prune(grad);
  math::GeneralSparseSolver_ConjugateGradient cg;
  cg.preconditioner_ = std::make_unique<math::GeneralSparsePreconditioner_BlockJacobi>();
  cg.SetProblem(bsr);
  cg.Compute();

  cg.Solve(grad, state->GetVariables()->View());

  // Check the solution.
  {
    auto var = state->GetVariables()->ConstView();
    copy(var_host, var);
    Real max_err = 0;
    for (size_t i = 0; i < nv; ++i) {
      auto xy = col_view(i);
      max_err = std::max(max_err, abs(var_host(0, i) - value_at(xy)));
    }
    AX_INFO("Max error: {}", max_err);
  }

  AX_INFO("Time elapsed: {}",
          std::chrono::duration_cast<std::chrono::milliseconds>(utils::now() - start));
  clean_up();
  return 0;
}
