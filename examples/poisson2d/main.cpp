/**
 * @brief This is a simple example of solving the Poisson equation using the finite element method.
 *        The equation is:
 *            -Laplace U = F.
 *        Where U = (1-x) sin x (1-y) sin y
 * @date 2024-02-10
 *
 */
#include "ax/core/init.hpp"
#include "ax/gl/utils.hpp"
#include "ax/graph/render.hpp"
#include "ax/nodes/geometry.hpp"
#include "ax/nodes/gl_prims.hpp"
#include "ax/nodes/io.hpp"
#include "ax/nodes/math_types.hpp"
#include "ax/nodes/stl_types.hpp"
#include "ax/utils/status.hpp"
#include "ax/fem/elements/p1.hpp"
#include "ax/geometry/common.hpp"
#include "ax/gl/utils.hpp"
#include "ax/gl/primitives/height_field.hpp"
#include "ax/graph/node.hpp"
#include "ax/graph/render.hpp"
#include "ax/math/linsys/sparse/LDLT.hpp"

using namespace ax;
using namespace ax::math;
using namespace ax::graph;

bool is_diriclet(Index id, Index n) {
  Index i = id / n;
  Index j = id % n;
  return i == 0 || i == n - 1 || j == 0 || j == n - 1;
}

IndexField3 make_triangles(Index nx, Index ny) {
  IndexField3 triangles(3, 2 * (nx - 1) * (ny - 1));
  Index id = 0;
  for (Index i = 0; i < nx - 1; ++i) {
    for (Index j = 0; j < ny - 1; ++j) {
      Index Index00 = i * ny + j;
      Index Index01 = i * ny + j + 1;
      Index Index10 = (i + 1) * ny + j;
      Index Index11 = (i + 1) * ny + j + 1;
      if (j % 2 == 0) {
        triangles.col(id++) = IndexVec3{Index00, Index11, Index01};
        triangles.col(id++) = IndexVec3{Index00, Index11, Index10};
      } else {
        triangles.col(id++) = IndexVec3{Index00, Index01, Index10};
        triangles.col(id++) = IndexVec3{Index01, Index11, Index10};
      }
    }
  }
  return triangles;
}

class SolvePoissionWithZeroDirichlet : public NodeBase {
public:
  SolvePoissionWithZeroDirichlet(NodeDescriptor const* descriptor, Index id)
      : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<SolvePoissionWithZeroDirichlet>()
        .SetName("Solve_Poission_With_Zero_Dirichlet_01_01")
        .SetDescription("Solves the Poission equation with zero Dirichlet boundary condition")
        .AddInput<RealMatrixX>("rhs", "Rhs of -laplace U = f")
        .AddInput<Index>("N", "Resolution of XY Dim")
        .AddOutput<RealMatrixX>("solution", "The solution of the Poission equation")
        .FinalizeAndRegister();
  }

  Status Apply(Index /* frame_id */) {
    auto* rhs = RetriveInput<RealMatrixX>(0);
    auto* n = RetriveInput<Index>(1);
    if (!rhs || !n) {
      return utils::FailedPreconditionError("Missing input");
    }

    math::RealField2 vertices(2, *n * *n);
    math::IndexField3 faces = make_triangles(*n, *n);
    for (Index i = 0; i < *n; ++i) {
      for (Index j = 0; j < *n; ++j) {
        vertices.col(i * *n + j) = math::RealVector2{i / Real(*n - 1), j / Real(*n - 1)};
      }
    }

    // SECT: Make a P1 element:
    SparseCOO coefficients;
    for (auto elem : each(faces)) {
      Index Index00 = elem[0];
      Index Index01 = elem[1];
      Index Index10 = elem[2];
      fem::elements::P1Element2D element({
          vertices.col(Index00),
          vertices.col(Index01),
          vertices.col(Index10),
      });
      for (Index i = 0; i < 3; ++i) {
        for (Index j = 0; j < 3; ++j) {
          coefficients.push_back(
              SparseEntry(elem[i], elem[j],
                       element.Integrate_PF_PF(i, j, 0, 0) + element.Integrate_PF_PF(i, j, 1, 1)));
        }
      }
    }

    // SECT: Set Dirichlet BC to 1
    SparseCOO coef_no_dirichlet;
    for (auto trip : coefficients) {
      if (!is_diriclet(trip.row(), *n) && !is_diriclet(trip.col(), *n)) {
        coef_no_dirichlet.push_back(trip);
      }
    }
    for (Index i = 0; i < *n; ++i) {
      for (Index j = 0; j < *n; ++j) {
        Index Index00 = i * *n + j;
        if (is_diriclet(Index00, *n)) {
          coef_no_dirichlet.push_back(SparseEntry(Index00, Index00, 1));
        }
      }
    }

    RealVectorX b(*n * *n);
    b.setZero();
    for (Index i = 0; i < *n; ++i) {
      for (Index j = 0; j < *n; ++j) {
        Index Index00 = i * *n + j;
        b[Index00] = rhs->operator()(i, j);
      }
    }

    for (Index i = 0; i < *n; ++i) {
      for (Index j = 0; j < *n; ++j) {
        Index Index00 = i * *n + j;
        if (i == 0 || i == *n - 1 || j == 0 || j == *n - 1) {
          b[Index00] = 0;
        }
      }
    }

    b = b * (1.0 / ((*n - 1) * (*n - 1)));

    // SECT: Solve the linear system:
    RealSparseMatrix A(*n * *n, *n * *n);
    A.setFromTriplets(coef_no_dirichlet.begin(), coef_no_dirichlet.end());
    A.makeCompressed();
    SparseSolver_LDLT ldlt;
    ldlt.SetProblem(A).Compute();
    auto solution = ldlt.Solve(b, {});
    RealVectorX x = solution.solution_;
    math::RealMatrixX solution_field(*n, *n);
    for (Index i = 0; i < *n; ++i) {
      for (Index j = 0; j < *n; ++j) {
        Index Index00 = i * *n + j;
        solution_field(i, j) = x[Index00];
      }
    }

    SetOutput<RealMatrixX>(0, solution_field);
    AX_RETURN_OK();
  }
};

Real gaussian(Real x, Real y, Real mean_x, Real mean_y, Real std_x, Real std_y) {
  return std::exp(-0.5 * ((x - mean_x) * (x - mean_x) / (std_x * std_x)
                          + (y - mean_y) * (y - mean_y) / (std_y * std_y)));
}

Real gaussian(RealVector2 const& x, RealVector2 const& mean, RealVector2 const& std) {
  return gaussian(x.x(), x.y(), mean.x(), mean.y(), std.x(), std.y());
}

class GenerateRhsByGaussian : public NodeBase {
public:
  GenerateRhsByGaussian(NodeDescriptor const* descriptor, Index id)
      : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<GenerateRhsByGaussian>()
        .SetName("Generate_Random_Rhs_By_Gaussian_01")
        .SetDescription("Generates a random right-hand-side by Gaussian distribution")
        .AddInput<Index>("N", "Resolution of XY Dim")
        .AddInput<RealVector2>("mean", "Mean of the Gaussian distribution")
        .AddInput<RealVector2>("std", "Standard deviation of the Gaussian distribution")
        .AddInput<Real>("scale", "Scale of the Gaussian distribution")
        .AddOutput<RealMatrixX>("rhs", "The right-hand-side of the Poission equation")
        .FinalizeAndRegister();
  }

  Status Apply(Index /* frame_id */) {
    auto* n = RetriveInput<Index>(0);
    auto* mean = RetriveInput<RealVector2>(1);
    auto* std = RetriveInput<RealVector2>(2);
    auto* scale = RetriveInput<Real>(3);
    if (!n) {
      return utils::FailedPreconditionError("Missing input");
    }

    RealVector2 mean_inuse = mean ? *mean : RealVector2{0.5, 0.5};
    RealVector2 std_inuse = std ? *std : RealVector2{1, 1};
    Real scale_inuse = scale ? *scale : 1.0;

    math::RealMatrixX rhs(*n, *n);
    for (Index i = 0; i < *n; ++i) {
      for (Index j = 0; j < *n; ++j) {
        rhs(i, j) = gaussian(RealVector2{i / Real(*n - 1), j / Real(*n - 1)}, mean_inuse, std_inuse);
      }
    }
    SetOutput<RealMatrixX>(0, scale_inuse * rhs);
    AX_RETURN_OK();
  }
};

class VisualizeHeightField : public NodeBase {
public:
  VisualizeHeightField(NodeDescriptor const* descriptor, Index id) : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<VisualizeHeightField>()
        .SetName("Visualize_Height_Field_01")
        .SetDescription("Visualizes the height field")
        .AddInput<RealMatrixX>("data", "The height field to visualize")
        .AddOutput<geo::SurfaceMesh>("mesh", "The mesh of the height field")
        .AddOutput<RealField4>("color", "The color of the height field")
        .FinalizeAndRegister();
  }

  Status Apply(Index /* frame_id */) {
    auto* height_field = RetriveInput<RealMatrixX>(0);
    if (!height_field) {
      return utils::FailedPreconditionError("Missing input");
    }
    auto height_field_gl = gl::make_height_field(height_field->reshaped(), height_field->rows(), height_field->cols());
    AX_RETURN_NOTOK(height_field_gl.status());
    geo::SurfaceMesh mesh;
    mesh.vertices_ = height_field_gl->vertices_;
    mesh.indices_ = height_field_gl->indices_;
    auto color = height_field_gl->colors_;
    SetOutput<geo::SurfaceMesh>(0, mesh);
    SetOutput<RealField4>(1, color);
    AX_RETURN_OK();
  }
};
int main(int argc, char** argv) {
  gl::init(argc, argv);
  graph::install_renderer();
  nodes::register_stl_types();
  nodes::register_io_nodes();
  nodes::register_math_types_nodes();
  nodes::register_gl_prim_nodes();
  nodes::register_geometry_nodes();

  SolvePoissionWithZeroDirichlet::register_this();
  GenerateRhsByGaussian::register_this();
  VisualizeHeightField::register_this();

  AX_CHECK_OK(gl::enter_main_loop());
  clean_up();
  return 0;
}
