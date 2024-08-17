#include <iostream>

#include "Eigen/Core"
#include "ax/core/init.hpp"
#include "ax/fem/elasticity.hpp"
#include "ax/fem/elasticity/arap.hpp"
#include "ax/fem/elasticity/linear.hpp"
#include "ax/fem/elasticity/neohookean_bw.hpp"
#include "ax/fem/elasticity/stable_neohookean.hpp"
#include "ax/fem/elasticity/stvk.hpp"
#include "ax/fem/elasticity_cpu.hpp"
#include "ax/fem/trimesh.hpp"
#include "ax/math/approx.hpp"
using namespace ax;

constexpr Index DIM = 3;

std::shared_ptr<fem::TriMesh<DIM>> mesh;
auto kE = fem::ElasticityUpdateLevel::kEnergy;
int main(int argc, char** argv) {
  init(argc, argv);

  math::RealVector2 lame = ax::fem::elasticity::compute_lame(1e4, 0.3);

  math::IndexField<DIM + 1> indices(DIM + 1, 1);
  for (Index i = 0; i <= DIM; ++i) {
    indices(i) = i;
  }

  math::RealField<DIM> original_vertices(3, DIM + 1);
  if constexpr (DIM == 3) {
    original_vertices.col(0) = math::RealVector3(0, 0, 0);
    original_vertices.col(1) = math::RealVector3(1, 0, 0);
    original_vertices.col(2) = math::RealVector3(0, 1, 0);
    original_vertices.col(3) = math::RealVector3(0, 0, 1);
  } else {
    original_vertices.col(0) = math::RealVector2(0, 0);
    original_vertices.col(1) = math::RealVector2(1, 0);
    original_vertices.col(2) = math::RealVector2(0, 1);
  }

  // Apply Random Rotation:
  // math::RealMatrix3 R = Eigen::AngleAxis(math::pi<> / 4, math::RealVector3::UnitX()).toRotationMatrix();
  // original_vertices = R * original_vertices;
  mesh = std::make_unique<fem::TriMesh<DIM>>();
  mesh->SetNumDofPerVertex(1);
  mesh->SetMesh(indices, original_vertices);

  // Elasticity and Deformation.
  fem::ElasticityCompute_CPU<DIM, fem::elasticity::NeoHookeanBW> elast(
      mesh);  //< 3d Linear Elasticity

  // randomly perturb the vertices.
  for (Index i = 0; i <= DIM; ++i) {
    for (Index d = 0; d < DIM; ++d) {
      original_vertices(d, i) += (rand() % 1000 - 500) * 1e-4;  // Range: [-0.05, 0.05]
    }
  }
  // Update the deformation gradient.
  std::cout << "Vertices:\n" << original_vertices << std::endl;
  mesh->SetVertices(original_vertices);
  elast.RecomputeRestPose();
  elast.SetLame(lame);
  elast.Update(mesh->GetVertices(), ax::fem::ElasticityUpdateLevel::kHessian);

  // Compute Gradient by Finite Difference:
  elast.Update(mesh->GetVertices(), kE);
  elast.UpdateEnergy();
  real e0 = elast.GetEnergyOnElements().sum();
  elast.UpdateStress();
  elast.GatherStressToVertices();
  auto stress = elast.GetStressOnVertices();
  // auto stiffness = elast.GatherHessian(elast.Hessian(lame)).toDense();
  elast.UpdateHessian(false);
  elast.GatherHessianToVertices();
  auto stiffness = elast.GetHessianOnVertices().toDense();

  std::cout << "EnergyImpl: " << e0 << std::endl;  // Should be zero.
  std::cout << "StressImpl:\n" << stress << std::endl;
  std::cout << "=======================================================================\n"
            << "  TEST STRESS\n"
            << "=======================================================================\n";
  for (Index i = 0; i <= DIM; ++i) {
    for (Index d = 0; d < DIM; ++d) {
      for (real delta_d = 1e-8; delta_d >= 1e-10; delta_d *= 0.1) {
        math::RealField<DIM> vertices_p = mesh->GetVertices();
        vertices_p(d, i) += delta_d;
        elast.Update(vertices_p, kE);
        // EnergyImpl:
        elast.UpdateEnergy();
        real e_p = elast.GetEnergyOnElements().sum();

        // Compute the slope:
        real slope = (e_p - e0) / delta_d;
        std::cout << "Vertex [" << i << "] Dof[" << d << "], Delta=" << delta_d
                  << ", Slope=" << slope
                  << "RelE: " << math::abs(slope / (stress(i * DIM + d) + math::epsilon<real>)-1)
                  << std::endl;
      }
    }
  }
  std::cout << "=======================================================================\n"
            << "  TEST STIFFNESS\n"
            << "=======================================================================\n";
  std::cout << "Stiffness:\n" << stiffness << std::endl;

  // Compute: partial^2 EnergyImpl / (partial x_id partial x_jk)
  for (Index i = 0; i <= DIM; ++i) {
    for (Index d = 0; d < DIM; ++d) {
      for (Index j = 0; j <= DIM; ++j) {
        for (Index k = 0; k < DIM; ++k) {
          std::cout << ">>> Vertex [" << i << "] Dof[" << d << "], Vertex [" << j << "] Dof[" << k
                    << "]" << std::endl;
          std::cout << "Reference: " << stiffness(i * DIM + d, j * DIM + k) << std::endl;
          real clothest = 1e10;
          for (real delta_id = 1e-6; delta_id >= 1e-8; delta_id *= 0.1) {
            math::RealField<DIM> vertices_id = original_vertices;
            vertices_id(d, i) += delta_id;
            elast.Update(vertices_id, kE);
            elast.UpdateEnergy();
            real e_id = elast.GetEnergyOnElements().sum();
            for (real delta_jk = 1e-6; delta_jk >= 1e-8; delta_jk *= 0.1) {
              math::RealField<DIM> vertices_id_jk = vertices_id;
              vertices_id_jk(k, j) += delta_jk;
              elast.Update(vertices_id_jk, kE);
              elast.UpdateEnergy();
              real e_id_jk = elast.GetEnergyOnElements().sum();
              math::RealField<DIM> vertices_jk = original_vertices;
              vertices_jk(k, j) += delta_jk;
              elast.Update(vertices_jk, kE);
              elast.UpdateEnergy();
              real e_jk = elast.GetEnergyOnElements().sum();
              real hessian = (e_id_jk - e_id - e_jk + e0) / (delta_jk * delta_id);
              if (std::abs(hessian - stiffness(i * DIM + d, j * DIM + k)) < clothest) {
                clothest = std::abs(hessian - stiffness(i * DIM + d, j * DIM + k));
              }
            }
          }
          std::cout << "Clothest Err: " << clothest << "\tRelative "
                    << clothest / math::abs(stiffness(i * DIM + d, j * DIM + k)) << std::endl;
        }
      }
    }
  }

  clean_up();
  return 0;
}
