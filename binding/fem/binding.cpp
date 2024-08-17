#include "binding.hpp"

#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "ax/core/entt.hpp"
#include "ax/fem/laplace_matrix.hpp"
#include "ax/fem/mass_matrix.hpp"
#include "ax/fem/timestepper/naive_optim.hpp"
#include "ax/fem/timestepper/quasi_newton.hpp"

namespace py = pybind11;
using namespace ax;
using namespace ax::fem;
namespace axb {

void bind_naive_optim(py::module& m) {
  m.def(
      "make_timestepper_3d",
      [](std::shared_ptr<TriMesh<3>> m, std::string name) -> std::shared_ptr<TimeStepperBase<3>> {
        if (name == "quasi_newton") {
          return std::make_unique<Timestepper_QuasiNewton<3>>(m);
        } else {
          return std::make_unique<Timestepper_NaiveOptim<3>>(m);
        }
      },
      py::arg("trimesh"), py::arg("method") = "naive_optim");

  m.def("make_mesh_3d",
        []() -> std::shared_ptr<TriMesh<3>> { return std::make_shared<TriMesh<3>>(); });

  py::class_<TriMesh<3>, std::shared_ptr<TriMesh<3>>> tm(m, "TriMesh3D");
  tm.def(py::init<>())
      .def("SetMesh", &TriMesh<3>::SetMesh, py::arg("elements"), py::arg("vertices"))
      .def("SetVertex", &TriMesh<3>::SetVertex, py::arg("i"), py::arg("vertex"))
      .def("SetVertices", &TriMesh<3>::SetVertices, py::arg("vertices"))
      .def("GetVertices", &TriMesh<3>::GetVertices)
      .def("GetVerticesFlattened", &TriMesh<3>::GetVerticesFlattened)
      .def("GetNumVertices", &TriMesh<3>::GetNumVertices)
      .def("GetNumElements", &TriMesh<3>::GetNumElements)
      .def("GetElements", &TriMesh<3>::GetElements)
      .def("ResetBoundary", &TriMesh<3>::ResetBoundary, py::arg("i"), py::arg("dof"))
      .def("ResetAllBoundaries", &TriMesh<3>::ResetAllBoundaries)
      .def("GetBoundaryValue", &TriMesh<3>::GetBoundaryValue, py::arg("i"), py::arg("dof"))
      .def("IsDirichletBoundary", &TriMesh<3>::IsDirichletBoundary, py::arg("i"), py::arg("dof"))
      .def("ExtractSurface", &TriMesh<3>::ExtractSurface)
      .def("FilterMatrixFull",
           [](TriMesh<3>& tm, math::spmatr K) -> math::spmatr {
             tm.FilterMatrixFull(K);
             return K;
           })
      .def("FilterMatrixDof",
           [](TriMesh<3>& tm, Index dof, math::spmatr K) -> math::spmatr {
             tm.FilterMatrixDof(dof, K);
             return K;
           })
      .def("GetDirichletBoundaryMask", &TriMesh<3>::GetDirichletBoundaryMask)
      .def("__repr__",
           [](const TriMesh<3>& tm) -> std::string {
             std::ostringstream oss;
             oss << "<TriMesh3D with " << tm.GetNumVertices() << " vertices and "
                 << tm.GetNumElements() << " elements>";
             return oss.str();
           })
      .def("MarkDirichletBoundary", &TriMesh<3>::MarkDirichletBoundary, py::arg("i"),
           py::arg("dof"), py::arg("value"))
      .def("SetNumDofPerVertex", &TriMesh<3>::SetNumDofPerVertex, py::arg("n"));

  py::class_<TimeStepperBase<3>, std::shared_ptr<TimeStepperBase<3>>> tsb3(m, "TimeStepperBase3D");

  tsb3.def("SetOptions", &TimeStepperBase<3>::SetOptions)
      .def("SetDensity", py::overload_cast<real>(&TimeStepperBase<3>::SetDensity))
      .def("SetDensity", py::overload_cast<math::RealField1 const&>(&TimeStepperBase<3>::SetDensity))
      .def("GetMesh", &TimeStepperBase<3>::GetMesh)
      .def("GetOptions", &TimeStepperBase<3>::GetOptions)
      .def("GetLastPosition", &TimeStepperBase<3>::GetLastPosition)
      .def("GetLastDisplacement", &TimeStepperBase<3>::GetLastDisplacement)
      .def("GetPosition", &TimeStepperBase<3>::GetPosition)
      .def("GetVelocity", &TimeStepperBase<3>::GetVelocity)
      .def("GetMassMatrix", &TimeStepperBase<3>::GetMassMatrix)
      .def("GetMassMatrixOriginal", &TimeStepperBase<3>::GetMassMatrixOriginal)
      .def("GetDisplacement", &TimeStepperBase<3>::GetDisplacement)
      .def("GetNextDisplacementDelta", &TimeStepperBase<3>::GetNextDisplacementDelta)
      .def("SetYoungs", &TimeStepperBase<3>::SetYoungs)
      .def("SetPoissonRatio", &TimeStepperBase<3>::SetPoissonRatio);
  tsb3.def("SetupElasticity", [](TimeStepperBase<3>& t, std::string name,
                                 std::string device) { t.SetupElasticity(name, device); })
      .def("Initialize", &TimeStepperBase<3>::Initialize)
      .def("BeginSimulation", &TimeStepperBase<3>::BeginSimulation, py::arg("dt"))
      .def("BeginTimestep", &TimeStepperBase<3>::BeginTimestep)
      .def("EndTimestep", py::overload_cast<>(&TimeStepperBase<3>::EndTimestep))
      .def("EndTimestep",
           py::overload_cast<math::RealField<3> const&>(&TimeStepperBase<3>::EndTimestep))
      .def("SolveTimestep", &TimeStepperBase<3>::SolveTimestep)
      .def("GetInitialGuess", &TimeStepperBase<3>::GetInitialGuess)
      .def("GetSolution", &TimeStepperBase<3>::GetSolution)
      .def("SetExternalAcceleration", &TimeStepperBase<3>::SetExternalAcceleration)
      .def("SetExternalAccelerationUniform", &TimeStepperBase<3>::SetExternalAccelerationUniform);

  tsb3.def("Energy", &TimeStepperBase<3>::Energy, py::arg("u"))
      .def("Gradient", &TimeStepperBase<3>::Gradient, py::arg("u"))
      .def("Hessian", &TimeStepperBase<3>::Hessian, py::arg("u"), py::arg("project") = true)
      .def("GradientFlat", &TimeStepperBase<3>::GradientFlat, py::arg("u_flat"));

  tsb3.def("GetLastTrajectory", &TimeStepperBase<3>::GetLastTrajectory)
      .def("GetLastEnergy", &TimeStepperBase<3>::GetLastEnergy);

  m.def("compute_mass_matrix_uniform", [](std::shared_ptr<TriMesh<3>> tm, real density) -> math::spmatr {
    AX_THROW_IF_NULL(tm);
    MassMatrixCompute<3> mmc(*tm);
    return mmc(density);
  });

  m.def("compute_mass_matrix",
        [](std::shared_ptr<TriMesh<3>> tm, math::RealField1 const& density) -> math::spmatr {
          AX_THROW_IF_NULL(tm);
          MassMatrixCompute<3> mmc(*tm);
          return mmc(density);
        });

  m.def("compute_laplace_matrix_uniform", [](std::shared_ptr<TriMesh<3>> tm, real density) -> math::spmatr {
    AX_THROW_IF_NULL(tm);
    LaplaceMatrixCompute<3> mmc(*tm);
    return mmc(density);
  });

  m.def("compute_laplace_matrix",
        [](std::shared_ptr<TriMesh<3>> tm, math::RealField1 const& density) -> math::spmatr {
          AX_THROW_IF_NULL(tm);
          LaplaceMatrixCompute<3> mmc(*tm);
          return mmc(density);
        });
  m.def("yp_to_lame", [](real youngs, real poisson) -> math::RealVector2 {
    return elasticity::compute_lame(youngs, poisson);
  });
}

void bind_experiment(py::module& m) {
  m.def("set_sparse_inverse_approximator",
        [](math::spmatr A, math::RealVectorX precond, real eig_modification, bool require_check_secant) {
          auto &sia = ensure_resource<SparseInverseApproximator>();
          sia.A_ = A;
          sia.precond_ = precond;
          sia.eig_modification_ = eig_modification;
          sia.require_check_secant_ = require_check_secant;
          sia.A_.makeCompressed();
        });

  m.def("apply_sparse_inverse_approximator",
        [](math::RealVectorX const &gk, math::RealVectorX const &sk, math::RealVectorX const &yk) -> math::RealVectorX {
          auto* cmpt = try_get_resource<SparseInverseApproximator>();
          AX_THROW_IF_NULL(cmpt, "SparseInverseApproximator not set.");
          auto apply = [cmpt](math::RealVectorX const &v) -> math::RealVectorX {
            // compute At A x + delta * x
            auto const& A = cmpt->A_;
            auto const& delta = cmpt->eig_modification_;
            auto const& precond = cmpt->precond_;
            math::RealVectorX At_v = A.transpose() * v;
            if (precond.size() > 0) {
              At_v = precond.cwiseProduct(At_v);
            }
            return A * At_v + delta * v;
          };
          if (sk.size() == 0 || yk.size() == 0) {
            // First time called.
            return apply(gk);
          }

          if (!(cmpt->require_check_secant_)) {
            return apply(gk);
          }

          // NOTE: Our original implementation in python
          // Hyk = self.apply_LDLT(y[-1])
          // gamma_Hsy = np.dot(y[-1], s[-1]) / (np.dot(y[-1], Hyk) + epsilon)
          // LDLT_q = self.apply_LDLT(q)
          // r = gamma_Hsy * LDLT_q
          // ---------------------------------------------------------------------
          // Derivation: Estimiate the secant equation coefficient
          // ---------------------------------------------------------------------
          // 1. Traditional. Estimate the 1 rank approximation of H0.
          // gamma_LSy = (np.dot(s[-1], y[-1]) / (np.dot(y[-1], y[-1]) + epsilon))
          // print(f'LSy: {gamma_LSy}')
          // 2. Ours. Estimate the approximation of H0, but with a different scale.
          // Secant equation: yk = Hk * sk
          //   <yk, sk> = gamma * <yk, I yk> => H0 = gamma I
          //   <yk, sk> = gamma * <yk, H yk> => gamma = <yk, sk> / <yk, H yk>
          real const gamma_Hsy = yk.dot(sk) / (yk.dot(apply(yk)) + math::epsilon<real>);
          AX_LOG(INFO) << "gamma_Hsy: " << gamma_Hsy;
          return gamma_Hsy * apply(gk);
        });

  m.def("extract_hessian_inverse", [](std::shared_ptr<TimeStepperBase<3>> tsb) -> math::spmatr {
      try {
        auto * qn = dynamic_cast<Timestepper_QuasiNewton<3>*>(tsb.get());
        return qn->GetLaplacianAsApproximation();
      } catch (std::exception& e) {
        throw std::runtime_error("Not a QuasiNewton timestepper!");
      }
  });
}

void bind_fem_module(py::module& m) {
  py::module fem = m.def_submodule("fem");
  bind_naive_optim(fem);
  bind_experiment(fem);
}

}  // namespace axb
