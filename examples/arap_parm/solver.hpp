#include "ax/math/common.hpp"
#include "ax/math/linsys/sparse.hpp"
#include "ax/geometry/common.hpp"
namespace xx {

using ax::math::RealVector3;
using ax::math::RealVector2;
using ax::math::IndexVec3;
using ax::math::RealMatrix3;
using ax::math::RealMatrix2;
using Parameterization = ax::math::RealField2;
using IsoCoord = ax::math::RealMatrix2;

using ax::geo::SurfaceMesh;

struct ParameterizationProblem {
  SurfaceMesh input_mesh_;
  std::vector<IsoCoord> iso_coords_;
  std::vector<RealMatrix2> Li_;
  std::vector<RealVector3> cotangent_weights_;
  Parameterization param_;
};

class LocalSolverBase {
public:
  virtual std::vector<RealMatrix2> Optimal(ParameterizationProblem const& problem) = 0;
  virtual ~LocalSolverBase() = default;
};

class ARAP final: public LocalSolverBase {
public:
  std::vector<RealMatrix2> Optimal(ParameterizationProblem const& problem) final;
  virtual ~ARAP() = default;
};

class ASAP final: public LocalSolverBase {
public:
  std::vector<RealMatrix2> Optimal(ParameterizationProblem const& problem) final;
  virtual ~ASAP() = default;
};

class ParameterizationSolver {
public:
  ParameterizationSolver(SurfaceMesh const& mesh);
  void SetLocalSolver(std::unique_ptr<LocalSolverBase> solver);
  ax::Status SetGlobalSolver(std::unique_ptr<ax::math::SparseSolverBase> solver);

  ax::Status Solve(ax::Index max_iter = 1000);
  
  SurfaceMesh Optimal();

private:
  std::unique_ptr<LocalSolverBase> local_solver_;
  std::unique_ptr<ax::math::SparseSolverBase> global_solver_;
  ParameterizationProblem problem_;
  ax::math::LinsysProblem_Sparse global_problem_;
  ax::real shift_ = 1.0;
};

}
