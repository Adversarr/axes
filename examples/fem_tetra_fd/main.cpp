#include "Eigen/Core"
#include "ax/core/init.hpp"
#include "ax/fem/elasticity.hpp"
#include "ax/fem/elasticity/arap.hpp"
#include "ax/fem/elasticity/linear.hpp"
#include "ax/fem/elasticity/neohookean_bw.hpp"
#include "ax/fem/elasticity/stvk.hpp"
#include "ax/fem/mesh/p1mesh.hpp"
#include "ax/math/approx.hpp"
using namespace ax;

constexpr idx DIM = 2;

fem::P1Mesh<DIM> mesh;
auto kE = fem::DeformationGradientUpdate::kEnergy;
int main(int argc, char** argv) {
  init(argc, argv);

  math::vec2r lame = ax::fem::elasticity::compute_lame(1e4, 0.3);
  // lame.x() = 0;
  // lame.y() = 1;

  math::fieldi<DIM + 1> indices(DIM + 1, 1);
  for (idx i = 0; i <= DIM; ++i) {
    indices(i) = i;
  }

  math::fieldr<DIM> original_vertices(3, DIM + 1);
  if constexpr (DIM == 3) {
    original_vertices.col(0) = math::vec3r(0, 0, 0);
    original_vertices.col(1) = math::vec3r(1, 0, 0);
    original_vertices.col(2) = math::vec3r(0, 1, 0);
    original_vertices.col(3) = math::vec3r(0, 0, 1);
  } else {
    original_vertices.col(0) = math::vec2r(0, 0);
    original_vertices.col(1) = math::vec2r(1, 0);
    original_vertices.col(2) = math::vec2r(0, 1);
  }

  // Apply Random Rotation:
  // math::mat3r R = Eigen::AngleAxis(math::pi<> / 4, math::vec3r::UnitX()).toRotationMatrix();
  // original_vertices = R * original_vertices;
  AX_CHECK_OK(mesh.SetMesh(indices, original_vertices));

  // Elasticity and Deformation.
  fem::Deformation<DIM> deform(mesh, mesh.GetVertices());                    //< 3d Deformation
  fem::ElasticityCompute<DIM, fem::elasticity::IsotropicARAP> elast(deform);  //< 3d Linear Elasticity

  // randomly perturb the vertices.
  for (idx i = 0; i <= DIM; ++i) {
    for (idx d = 0; d < DIM; ++d) {
      original_vertices(d, i) += (rand() % 1000 - 500) * 1e-4; // Range: [-0.05, 0.05]
    }
  }
  // Update the deformation gradient.
  std::cout << "Vertices:\n" << original_vertices << std::endl;
  AX_CHECK_OK(mesh.SetVertices(original_vertices));
  elast.UpdateDeformationGradient(mesh.GetVertices(), ax::fem::DeformationGradientUpdate::kHessian);

  // Compute Gradient by Finite Difference:
  real e0 = elast.Energy(lame).sum();
  auto stress = deform.StressToVertices(elast.Stress(lame));  // There is, and only is one tetra
  auto stiffness = math::make_sparse_matrix(DIM * (DIM + 1), DIM * (DIM + 1),
                                            deform.HessianToVertices(elast.Hessian(lame)))
                       .toDense();
  std::cout << "EnergyImpl: " << e0 << std::endl;  // Should be zero.
  std::cout << "StressImpl:\n" << stress << std::endl;
  std::cout << "=======================================================================\n"
            << "  TEST STRESS\n"
            << "=======================================================================\n";
  for (idx i = 0; i <= DIM; ++i) {
    for (idx d = 0; d < DIM; ++d) {
      for (real delta_d = 1e-8; delta_d >= 1e-10; delta_d *= 0.1) {
        math::fieldr<DIM> vertices_p = mesh.GetVertices();
        vertices_p(d, i) += delta_d;
        elast.UpdateDeformationGradient(vertices_p, kE);
        // EnergyImpl:
        real e_p = elast.Energy(lame).sum();

        // Compute the slope:
        real slope = (e_p - e0) / delta_d;
        std::cout << "Vertex [" << i << "] Dof[" << d << "], Delta=" << delta_d
                  << ", Slope=" << slope << "RelE: " << math::abs(slope / (stress(i * DIM + d) + math::epsilon<real>) - 1)
                  << std::endl;
      }
    }
  }
  std::cout << "=======================================================================\n"
            << "  TEST STIFFNESS\n"
            << "=======================================================================\n";
  std::cout << "Stiffness:\n" << stiffness << std::endl;

  // Compute: partial^2 EnergyImpl / (partial x_id partial x_jk)
  for (idx i = 0; i <= DIM; ++i) {
    for (idx d = 0; d < DIM; ++d) {
      for (idx j = 0; j <= DIM; ++j) {
        for (idx k = 0; k < DIM; ++k) {
          std::cout << ">>> Vertex [" << i << "] Dof[" << d << "], Vertex [" << j << "] Dof[" << k
                    << "]" << std::endl;
          std::cout << "Reference: " << stiffness(i * DIM + d, j * DIM + k) << std::endl;
          real clothest = 1e10;
          for (real delta_id = 1e-6; delta_id >= 1e-8; delta_id *= 0.1) {
            math::fieldr<DIM> vertices_id = original_vertices;
            vertices_id(d, i) += delta_id;
            elast.UpdateDeformationGradient(vertices_id, kE);
            real e_id = elast.Energy(lame).sum();
            for (real delta_jk = 1e-6; delta_jk >= 1e-8; delta_jk *= 0.1) {
              math::fieldr<DIM> vertices_id_jk = vertices_id;
              vertices_id_jk(k, j) += delta_jk;
              elast.UpdateDeformationGradient(vertices_id_jk, kE);
              real e_id_jk = elast.Energy(lame).sum();
              math::fieldr<DIM> vertices_jk = original_vertices;
              vertices_jk(k, j) += delta_jk;
              elast.UpdateDeformationGradient(vertices_jk, kE);
              real e_jk = elast.Energy(lame).sum();
              real hessian = (e_id_jk - e_id - e_jk + e0) / (delta_jk * delta_id);
              if (std::abs(hessian - stiffness(i * DIM + d, j * DIM + k)) < clothest) {
                clothest = std::abs(hessian - stiffness(i * DIM + d, j * DIM + k));
              }
            }
          }
          std::cout << "Clothest Err: " << clothest << "\tRelative " << clothest / math::abs(stiffness(i * DIM + d, j * DIM + k)) << std::endl;
        }
      }
    }
  }

  clean_up();
  return 0;
}