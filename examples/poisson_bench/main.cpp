#include <fmt/format.h>

#include <set>

#include "ax/core/init.hpp"
#include "ax/fem/elasticity/base.hpp"
#include "ax/fem/laplace_matrix.hpp"
#include "ax/fem/mass_matrix.hpp"
#include "ax/fem/trimesh.hpp"
#include "ax/geometry/topology.hpp"
#include "ax/math/io.hpp"
#include "ax/math/linsys/preconditioner/Identity.hpp"
#include "ax/math/linsys/sparse.hpp"
#include "ax/math/linsys/sparse/ConjugateGradient.hpp"
#include "ax/utils/asset.hpp"
#include "ax/utils/ndrange.hpp"
#include "ax/utils/time.hpp"
using namespace ax;

void run_datagen(const fem::TriMesh<3>& mesh, const math::RealSparseMatrix& sys_matrix,
                 math::SparseSolverBase* solver, math::RealMatrixX& rhs,
                 math::RealMatrixX& ground_truth, Index nprobs, Index ndatagen);

void run_basis_test(const fem::TriMesh<3>& mesh, const math::RealSparseMatrix& sys_matrix,
                    math::SparseSolverBase* solver, Index test_basis_cnt);

int main(int argc, char* argv[]) {
  ax::po::add_option({
    po::make_option<std::string>("mesh", "The mesh file to read.", "beam_high_res"),
    po::make_option<std::string>("solver", "The solver to use.", "ConjugateGradient"),
    po::make_option<std::string>("precond", "The preconditioner to use.", "IncompleteCholesky"),
    po::make_option<Real>("youngs", "Young's modulus.", "1e7"),
    po::make_option<Real>("poissons", "Poisson's ratio.", "0.49"),
    po::make_option<Real>("dt", "Time step size.", "1e-3"),
    po::make_option<Index>("nprobs", "Number of problems to solve.", "1"),
    po::make_option<Index>(
        "ndatagen", "Will generate data into outs_poisson, 0 for disable, only benchmark", "0"),
    po::make_option<Index>("testbasis", "Test the basis generated by PCA.", "0"),
    po::make_option<bool>("srand", "Set the random seed.", "false"),
  });

  ax::initialize(argc, argv);

  if (po::get_parse_result()["srand"].as<bool>()) {
    srand(time(nullptr));
  }

  // Load the mesh
  std::string mesh_name = po::get_parse_result()["mesh"].as<std::string>();
  math::RealField3 v = math::read_npy_v10_real(fmt::format("{}/mesh/npy/{}_vertices.npy",
                                                           utils::get_asset_dir(), mesh_name))
                           .transpose();
  math::IndexField4 e = math::read_npy_v10_Index(fmt::format("{}/mesh/npy/{}_elements.npy",
                                                             utils::get_asset_dir(), mesh_name))
                            .transpose();
  // make mesh
  fem::TriMesh<3> mesh;
  mesh.SetNumDofPerVertex(1);
  mesh.SetMesh(e, v);
  AX_INFO("Mesh loaded: {}, #Vert={}, #Elem={}", mesh_name, v.cols(), e.cols());

  // make solver.
  std::string solver_name = po::get_parse_result()["solver"].as<std::string>();
  auto kind = utils::reflect_enum<math::SparseSolverKind>(solver_name);
  if (!kind) {
    AX_ERROR("Invalid solver name: {}", solver_name);
    return EXIT_FAILURE;
  }
  auto solver = math::SparseSolverBase::Create(*kind);
  if (!solver) {
    AX_ERROR("Failed to create solver: {}", solver_name);
    return EXIT_FAILURE;
  }
  std::string precond_name = po::get_parse_result()["precond"].as<std::string>();
  auto precond_kind = utils::reflect_enum<math::PreconditionerKind>(precond_name);
  if (!precond_kind) {
    AX_ERROR("Invalid preconditioner name: {}", precond_name);
    return EXIT_FAILURE;
  }
  auto precond = math::PreconditionerBase::Create(*precond_kind);
  if (!precond) {
    AX_ERROR("Failed to create preconditioner: {}", precond_name);
    return EXIT_FAILURE;
  }

  solver->SetPreconditioner(std::move(precond));
  AX_INFO("Solver Options: {}", solver->GetOptions());

  // make problem.
  // 1. mass matrix
  auto mass_matrix_compute = fem::MassMatrixCompute<3>(mesh);
  auto mass_matrix = mass_matrix_compute(1.0e3);  // Density is 1.0e3.

  // 2. laplacian, also the stiffness matrix.
  Real youngs = po::get_parse_result()["youngs"].as<Real>(),
       poissons = po::get_parse_result()["poissons"].as<Real>(),
       dt = po::get_parse_result()["dt"].as<Real>();

  AX_CHECK(youngs > 0 && poissons > 0 && dt > 0,
           "Invalid parameters: youngs={}, poissons={}, dt={}", youngs, poissons, dt);
  AX_CHECK(poissons < 0.5, "Invalid Poisson's ratio: poissons={}", poissons);
  auto lame
      = fem::elasticity::compute_lame(youngs, poissons);  // Young's modulus and Poisson's ratio
  auto laplacian_matrix_compute = fem::LaplaceMatrixCompute<3>(mesh);
  auto laplacian_matrix
      = laplacian_matrix_compute((lame[0] + 2 * lame[1]) * dt * dt);  // as Liu 17 proposed.
  // 3. assemble the system.
  math::RealSparseMatrix sys_matrix = laplacian_matrix + mass_matrix;

  // 4. we need to set the boundary condition to enforce the Dirichlet boundary condition.
  //    we randomly set the boundary points to zero.
  auto bd = geo::get_boundary_triangles(v, e);
  std::set<Index> bd_vertices;
  for (auto t : bd.colwise()) {
    for (Index i = 0; i < 3; ++i) {
      if (mesh.GetVertex(t(i)).x() > 4.9) {
        bd_vertices.insert(t(i));
      }
    }
  }
  //    Random set the boundary vertices to zero.
  mesh.SetNumDofPerVertex(1);

  // 5. solve the problem.
  Index nprobs = po::get_parse_result()["nprobs"].as<Index>();
  math::RealMatrixX u0 = math::RealMatrixX::Zero(sys_matrix.rows(), nprobs);
  //    set a random field as rhs.
  math::RealMatrixX ground_truth = math::RealMatrixX::Random(sys_matrix.rows(), nprobs);
  //    set rhs to zero for the boundary vertices.
  for (auto v : bd_vertices) {
    ground_truth.row(v).setZero();
  }
  math::RealMatrixX rhs = sys_matrix * ground_truth;
  Index ndatagen = po::get_parse_result()["ndatagen"].as<Index>();

  if (Index test_basis_cnt = po::get_parse_result()["testbasis"].as<Index>(); test_basis_cnt > 0) {
    solver->SetProblem(sys_matrix).Compute();  // Analyse pattern, and factorize the matrix.
    run_basis_test(mesh, sys_matrix, solver.get(), test_basis_cnt);
  } else if (ndatagen == 0) {
    // solve the problem.
    auto start = utils::now();
    AX_INFO("Solving the problem...");
    solver->SetProblem(sys_matrix).Compute();  // Analyse pattern, and factorize the matrix.
    auto result = solver->Solve(rhs, u0);
    auto end = utils::now();

    // test the error.
    const auto& sol = result.solution_;
    Real rel_l2 = (sol - ground_truth).norm() / ground_truth.norm();
    AX_INFO("Relative L2 error: {}, iterations={} time elapsed: {}", rel_l2, result.num_iter_,
            (end - start));
    AX_INFO("Per Problem: {}",
            std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - start)
                / nprobs);
  } else {
    solver->SetProblem(sys_matrix).Compute();  // Analyse pattern, and factorize the matrix.
    run_datagen(mesh, sys_matrix, solver.get(), rhs, ground_truth, nprobs, ndatagen);
  }
  ax::clean_up();
  return EXIT_SUCCESS;
}

void run_datagen(const fem::TriMesh<3>& mesh, const math::RealSparseMatrix& sys_matrix,
                 math::SparseSolverBase* solver, math::RealMatrixX& rhs,
                 math::RealMatrixX& ground_truth, Index nprobs, Index ndatagen) {
  // generate data.
  // write the basic information, i.e. the mesh.
  math::write_npy_v10(fmt::format("outs_poisson/mesh_vertices.npy"),
                      math::RealMatrixX(mesh.GetVertices()));
  math::write_npy_v10(fmt::format("outs_poisson/mesh_elements.npy"),
                      math::IndexMatrixX(mesh.GetElements()));
  math::write_sparse_matrix(fmt::format("outs_poisson/sys_matrix"), sys_matrix);  // MatrixMarket

  math::RealVector3 bbox_min = mesh.GetVertices().rowwise().minCoeff();
  math::RealVector3 bbox_max = mesh.GetVertices().rowwise().maxCoeff();
  auto gen_box = [&bbox_min, &bbox_max]() {
    math::RealVector3 rand0 = math::RealVector3::Random() * 0.5;
    math::RealVector3 rand1 = math::RealVector3::Random() * 0.5;
    math::RealVector3 box_center = (bbox_min + bbox_max) * 0.5;
    math::RealVector3 box_size = (bbox_max - bbox_min) * 0.5;
    math::RealVector3 box_corner0 = box_center + rand0.cwiseProduct(box_size);
    math::RealVector3 box_corner1 = box_center + rand1.cwiseProduct(box_size);
    math::RealVector3 box_min = box_corner0.cwiseMin(box_corner1);
    math::RealVector3 box_max = box_corner0.cwiseMax(box_corner1);
    return std::make_pair(box_min, box_max);
  };

  AX_INFO("Begin generate data...");
  for (Index i = 0; i < ndatagen; ++i) {
    // Randomly sample a region:
    rhs.setZero();
    for (auto c : utils::range<Index>(nprobs)) {
      for (auto nbox : utils::range<Index>(5)) {
        auto [box_min, box_max] = gen_box();
        for (Index v = 0; v < mesh.GetNumVertices(); ++v) {
          if ((mesh.GetVertex(v) - box_min).minCoeff() > 0
              && (box_max - mesh.GetVertex(v)).minCoeff() > 0) {
            rhs(v, c) = 1.0 / (box_max - box_min).prod();
          }
        }
      }
    }

    // Solve the problem.
    auto start = utils::now();
    AX_INFO("Solving the problem...");
    auto r = solver->Solve(rhs, ground_truth);
    auto end = utils::now();
    AX_INFO("Time elapsed: {}", (end - start));
    ground_truth = r.solution_;

    Real rel_l2 = (sys_matrix * ground_truth - rhs).norm() / rhs.norm();
    AX_INFO("Relative L2 error: {}", rel_l2);

    // Write the data to poission_outs.
    math::write_npy_v10(fmt::format("outs_poisson/u0_{}.npy", i),
                        ground_truth);                                    // shape: (n, nprobs)
    math::write_npy_v10(fmt::format("outs_poisson/rhs_{}.npy", i), rhs);  // shape: (n, nprobs)
  }
  AX_INFO("Data generation finished.");
}

void run_basis_test(const fem::TriMesh<3>& mesh, const math::RealSparseMatrix& sys_matrix, 
  math::SparseSolverBase* solver, Index test_basis_cnt) {
  math::RealMatrixX basis = math::read_npy_v10_real("basis.npy");  // (128, ndof), if it is very well tuned, performance is really good.

  for (auto _ : utils::range(test_basis_cnt)) {
    math::RealVector3 bbox_min = mesh.GetVertices().rowwise().minCoeff();
    math::RealVector3 bbox_max = mesh.GetVertices().rowwise().maxCoeff();
    auto gen_box = [&bbox_min, &bbox_max]() {
      math::RealVector3 rand0 = math::RealVector3::Random() * 0.5;
      math::RealVector3 rand1 = math::RealVector3::Random() * 0.5;
      math::RealVector3 box_center = (bbox_min + bbox_max) * 0.5;
      math::RealVector3 box_size = (bbox_max - bbox_min) * 0.5;
      math::RealVector3 box_corner0 = box_center + rand0.cwiseProduct(box_size);
      math::RealVector3 box_corner1 = box_center + rand1.cwiseProduct(box_size);
      math::RealVector3 box_min = box_corner0.cwiseMin(box_corner1);
      math::RealVector3 box_max = box_corner0.cwiseMax(box_corner1);
      return std::make_pair(box_min, box_max);
    };

    AX_INFO("Begin generate data...");
    // Randomly sample a region:
    math::RealMatrixX rhs = math::RealMatrixX::Zero(sys_matrix.rows(), 1);
    for (auto nbox : utils::range<Index>(20)) {
      auto [box_min, box_max] = gen_box();
      Real sign = ((rand() % 2) * 2 - 1);
      for (Index v = 0; v < mesh.GetNumVertices(); ++v) {
        if ((mesh.GetVertex(v) - box_min).minCoeff() > 0
            && (box_max - mesh.GetVertex(v)).minCoeff() > 0) {
          rhs(v, 0) = sign / (box_max - box_min).prod();
        }
      }
    }

    auto start = utils::now();
    auto ground_truth = solver->Solve(rhs, {});
    auto end = utils::now();
    {
      Real rel_l2 = (sys_matrix * ground_truth.solution_ - rhs).norm() / rhs.norm();
      AX_INFO("(Solution) Relative L2 error: {} num_iter={} time elapsed: {}", rel_l2,
              ground_truth.num_iter_, end - start);
    }

    // Solve the problem.
    AX_INFO("Solving the problem...");
    math::RealMatrixX sys_matrix_reduced = basis * sys_matrix * basis.transpose();
    start = utils::now();
    // Step 1. Project the rhs to the basis P, and solve P' A P u = P' rhs.
    AX_CHECK(sys_matrix_reduced.rows() == basis.rows() && sys_matrix_reduced.cols() == basis.rows(),
             "Invalid reduced matrix size: {}, {}", sys_matrix_reduced.rows(),
             sys_matrix_reduced.cols());
    math::RealMatrixX rhs_reduced = basis * rhs;
    math::RealMatrixX u_reduced = sys_matrix_reduced.ldlt().solve(rhs_reduced);

    // Step 2. Project the solution back to the original space.
    math::RealMatrixX u = basis.transpose() * u_reduced;
    end = utils::now();
    // compute current residual.
    math::RealMatrixX residual = rhs - sys_matrix * u;
    AX_INFO("After solve reduced, residual: {}, time elapsed: {}", residual.norm(), (end - start));
    // compute the error.
    Real rel_l2 = residual.norm() / rhs.norm();
    AX_INFO("Relative L2 error (original): {}", rel_l2);
    rel_l2 = (u - ground_truth.solution_).norm() / ground_truth.solution_.norm();
    AX_INFO("Relative L2 error (solution): {}", rel_l2);

    // compute the direction.
    Real dot_prod = math::dot(residual + u, rhs);
    AX_INFO("DotProd: {}", dot_prod);

    // Now solve residual.
    start = utils::now();
    math::SparseSolver_ConjugateGradient cg;
    cg.SetPreconditioner(std::make_unique<math::Preconditioner_Identity>());
    cg.SetProblem(sys_matrix).Compute();
    auto result = cg.Solve(residual, {});
    end = utils::now();

    AX_INFO("Solve the residual, iterations={}, time elapsed: {}", result.num_iter_, end - start);
  }
}