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

bool is_diriclet(idx id, idx n) {
  idx i = id / n;
  idx j = id % n;
  return i == 0 || i == n - 1 || j == 0 || j == n - 1;
}

field3i make_triangles(idx nx, idx ny) {
  field3i triangles(3, 2 * (nx - 1) * (ny - 1));
  idx id = 0;
  for (idx i = 0; i < nx - 1; ++i) {
    for (idx j = 0; j < ny - 1; ++j) {
      idx idx00 = i * ny + j;
      idx idx01 = i * ny + j + 1;
      idx idx10 = (i + 1) * ny + j;
      idx idx11 = (i + 1) * ny + j + 1;
      if (j % 2 == 0) {
        triangles.col(id++) = vec3i{idx00, idx11, idx01};
        triangles.col(id++) = vec3i{idx00, idx11, idx10};
      } else {
        triangles.col(id++) = vec3i{idx00, idx01, idx10};
        triangles.col(id++) = vec3i{idx01, idx11, idx10};
      }
    }
  }
  return triangles;
}

class SolvePoissionWithZeroDirichlet : public NodeBase {
public:
  SolvePoissionWithZeroDirichlet(NodeDescriptor const* descriptor, idx id)
      : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<SolvePoissionWithZeroDirichlet>()
        .SetName("Solve_Poission_With_Zero_Dirichlet_01_01")
        .SetDescription("Solves the Poission equation with zero Dirichlet boundary condition")
        .AddInput<matxxr>("rhs", "Rhs of -laplace U = f")
        .AddInput<idx>("N", "Resolution of XY Dim")
        .AddOutput<matxxr>("solution", "The solution of the Poission equation")
        .FinalizeAndRegister();
  }

  Status Apply(idx /* frame_id */) {
    auto* rhs = RetriveInput<matxxr>(0);
    auto* n = RetriveInput<idx>(1);
    if (!rhs || !n) {
      return utils::FailedPreconditionError("Missing input");
    }

    math::field2r vertices(2, *n * *n);
    math::field3i faces = make_triangles(*n, *n);
    for (idx i = 0; i < *n; ++i) {
      for (idx j = 0; j < *n; ++j) {
        vertices.col(i * *n + j) = math::vec2r{i / real(*n - 1), j / real(*n - 1)};
      }
    }

    // SECT: Make a P1 element:
    sp_coeff_list coefficients;
    for (auto elem : each(faces)) {
      idx idx00 = elem[0];
      idx idx01 = elem[1];
      idx idx10 = elem[2];
      fem::elements::P1Element2D element({
          vertices.col(idx00),
          vertices.col(idx01),
          vertices.col(idx10),
      });
      for (idx i = 0; i < 3; ++i) {
        for (idx j = 0; j < 3; ++j) {
          coefficients.push_back(
              sp_coeff(elem[i], elem[j],
                       element.Integrate_PF_PF(i, j, 0, 0) + element.Integrate_PF_PF(i, j, 1, 1)));
        }
      }
    }

    // SECT: Set Dirichlet BC to 1
    sp_coeff_list coef_no_dirichlet;
    for (auto trip : coefficients) {
      if (!is_diriclet(trip.row(), *n) && !is_diriclet(trip.col(), *n)) {
        coef_no_dirichlet.push_back(trip);
      }
    }
    for (idx i = 0; i < *n; ++i) {
      for (idx j = 0; j < *n; ++j) {
        idx idx00 = i * *n + j;
        if (is_diriclet(idx00, *n)) {
          coef_no_dirichlet.push_back(sp_coeff(idx00, idx00, 1));
        }
      }
    }

    vecxr b(*n * *n);
    b.setZero();
    for (idx i = 0; i < *n; ++i) {
      for (idx j = 0; j < *n; ++j) {
        idx idx00 = i * *n + j;
        b[idx00] = rhs->operator()(i, j);
      }
    }

    for (idx i = 0; i < *n; ++i) {
      for (idx j = 0; j < *n; ++j) {
        idx idx00 = i * *n + j;
        if (i == 0 || i == *n - 1 || j == 0 || j == *n - 1) {
          b[idx00] = 0;
        }
      }
    }

    b = b * (1.0 / ((*n - 1) * (*n - 1)));

    // SECT: Solve the linear system:
    sp_matxxr A(*n * *n, *n * *n);
    A.setFromTriplets(coef_no_dirichlet.begin(), coef_no_dirichlet.end());
    A.makeCompressed();
    SparseSolver_LDLT ldlt;
    LinsysProblem_Sparse problem(A, b);
    auto solution = ldlt.SolveProblem(problem);
    if (!solution.ok()) {
      AX_LOG(ERROR) << "Failed to solve the linear system";
      return solution.status();
    }
    vecxr x = solution->solution_;
    math::matxxr solution_field(*n, *n);
    for (idx i = 0; i < *n; ++i) {
      for (idx j = 0; j < *n; ++j) {
        idx idx00 = i * *n + j;
        solution_field(i, j) = x[idx00];
      }
    }

    SetOutput<matxxr>(0, solution_field);
    AX_RETURN_OK();
  }
};

real gaussian(real x, real y, real mean_x, real mean_y, real std_x, real std_y) {
  return std::exp(-0.5 * ((x - mean_x) * (x - mean_x) / (std_x * std_x)
                          + (y - mean_y) * (y - mean_y) / (std_y * std_y)));
}

real gaussian(vec2r const& x, vec2r const& mean, vec2r const& std) {
  return gaussian(x.x(), x.y(), mean.x(), mean.y(), std.x(), std.y());
}

class GenerateRhsByGaussian : public NodeBase {
public:
  GenerateRhsByGaussian(NodeDescriptor const* descriptor, idx id)
      : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<GenerateRhsByGaussian>()
        .SetName("Generate_Random_Rhs_By_Gaussian_01")
        .SetDescription("Generates a random right-hand-side by Gaussian distribution")
        .AddInput<idx>("N", "Resolution of XY Dim")
        .AddInput<vec2r>("mean", "Mean of the Gaussian distribution")
        .AddInput<vec2r>("std", "Standard deviation of the Gaussian distribution")
        .AddInput<real>("scale", "Scale of the Gaussian distribution")
        .AddOutput<matxxr>("rhs", "The right-hand-side of the Poission equation")
        .FinalizeAndRegister();
  }

  Status Apply(idx /* frame_id */) {
    auto* n = RetriveInput<idx>(0);
    auto* mean = RetriveInput<vec2r>(1);
    auto* std = RetriveInput<vec2r>(2);
    auto* scale = RetriveInput<real>(3);
    if (!n) {
      return utils::FailedPreconditionError("Missing input");
    }

    vec2r mean_inuse = mean ? *mean : vec2r{0.5, 0.5};
    vec2r std_inuse = std ? *std : vec2r{1, 1};
    real scale_inuse = scale ? *scale : 1.0;

    math::matxxr rhs(*n, *n);
    for (idx i = 0; i < *n; ++i) {
      for (idx j = 0; j < *n; ++j) {
        rhs(i, j) = gaussian(vec2r{i / real(*n - 1), j / real(*n - 1)}, mean_inuse, std_inuse);
      }
    }
    SetOutput<matxxr>(0, scale_inuse * rhs);
    AX_RETURN_OK();
  }
};

class VisualizeHeightField : public NodeBase {
public:
  VisualizeHeightField(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<VisualizeHeightField>()
        .SetName("Visualize_Height_Field_01")
        .SetDescription("Visualizes the height field")
        .AddInput<matxxr>("data", "The height field to visualize")
        .AddOutput<geo::SurfaceMesh>("mesh", "The mesh of the height field")
        .AddOutput<field4r>("color", "The color of the height field")
        .FinalizeAndRegister();
  }

  Status Apply(idx /* frame_id */) {
    auto* height_field = RetriveInput<matxxr>(0);
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
    SetOutput<field4r>(1, color);
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
