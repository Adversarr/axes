#include "binding.hpp"

#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "ax/fem/timestepper/naive_optim.hpp"

namespace py = pybind11;
using namespace ax;
using namespace ax::fem;
namespace axb {

void bind_naive_optim(py::module& m) {
  m.def(
      "make_timestepper_3d",
      [](std::shared_ptr<TriMesh<3>> m) -> std::shared_ptr<TimeStepperBase<3>> {
        return std::make_shared<Timestepper_NaiveOptim<3>>(m);
      },
      py::arg("trimesh"));

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
           [](TriMesh<3>& tm, idx dof, math::spmatr K) -> math::spmatr {
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

  py::class_<TimeStepperBase<3>, SPtr<TimeStepperBase<3>>> tsb3(m, "TimeStepperBase3D");

  tsb3.def("SetOptions", &TimeStepperBase<3>::SetOptions)
      .def("SetDensity", py::overload_cast<real>(&TimeStepperBase<3>::SetDensity))
      .def("SetDensity", py::overload_cast<math::field1r const&>(&TimeStepperBase<3>::SetDensity))
      .def("GetMesh", &TimeStepperBase<3>::GetMesh)
      .def("GetOptions", &TimeStepperBase<3>::GetOptions)
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
      .def("BeginSimulation", &TimeStepperBase<3>::BeginSimulation, py::arg("dt") = -1)
      .def("BeginTimestep", &TimeStepperBase<3>::BeginTimestep, py::arg("dt"))
      .def("EndTimestep", py::overload_cast<>(&TimeStepperBase<3>::EndTimestep))
      .def("EndTimestep",
           py::overload_cast<math::fieldr<3> const&>(&TimeStepperBase<3>::EndTimestep))
      .def("SolveTimestep", &TimeStepperBase<3>::SolveTimestep)
      .def("GetInitialGuess", &TimeStepperBase<3>::GetInitialGuess)
      .def("GetSolution", &TimeStepperBase<3>::GetSolution)
      .def("SetExternalAcceleration", &TimeStepperBase<3>::SetExternalAcceleration)
      .def("SetExternalAccelerationUniform", &TimeStepperBase<3>::SetExternalAccelerationUniform);

  tsb3.def("Energy", &TimeStepperBase<3>::Energy, py::arg("u"))
      .def("Gradient", &TimeStepperBase<3>::Gradient, py::arg("u"))
      .def("Hessian", &TimeStepperBase<3>::Hessian, py::arg("u"), py::arg("project") = true);
}

void bind_fem_module(py::module& m) {
  py::module fem = m.def_submodule("fem");
  bind_naive_optim(fem);
}

}  // namespace axb
