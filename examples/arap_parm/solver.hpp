#include "axes/math/common.hpp"
#include "axes/math/linsys/sparse.hpp"
#include "axes/geometry/common.hpp"
namespace xx {

using ax::math::vec3r;
using ax::math::vec2r;
using ax::math::vec3i;
using ax::math::mat3r;
using ax::math::mat2r;
using Parameterization = ax::math::field2r;
using IsoCoord = ax::math::mat2r;

template <typename T> using List=std::vector<T>;
using ax::geo::SurfaceMesh;

struct ParameterizationProblem {
  SurfaceMesh input_mesh_;
  List<IsoCoord> iso_coords_;
  List<mat2r> Li_;
  List<vec3r> cotangent_weights_;
  Parameterization param_;
};

class LocalSolverBase {
public:
  virtual List<mat2r> Optimal(ParameterizationProblem const& problem) = 0;
};

class ARAP final: public LocalSolverBase {
public:
  List<mat2r> Optimal(ParameterizationProblem const& problem) final;
};

class ASAP final: public LocalSolverBase {
public:
  List<mat2r> Optimal(ParameterizationProblem const& problem) final;
};

class ParameterizationSolver {
public:
  ParameterizationSolver(SurfaceMesh const& mesh);
  void SetLocalSolver(std::unique_ptr<LocalSolverBase> solver);
  ax::Status SetGlobalSolver(std::unique_ptr<ax::math::SparseSolverBase> solver);

  ax::Status Solve(ax::idx max_iter = 1000);
  
  SurfaceMesh Optimal();

private:
  std::unique_ptr<LocalSolverBase> local_solver_;
  std::unique_ptr<ax::math::SparseSolverBase> global_solver_;
  ParameterizationProblem problem_;
  ax::math::LinsysProblem_Sparse global_problem_;
  ax::real shift = 1.0;
};

}