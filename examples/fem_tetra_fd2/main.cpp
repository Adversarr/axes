#include "ax/core/buffer/copy.hpp"
#include "ax/core/buffer/create_buffer.hpp"
#include "ax/core/buffer/eigen_support.hpp"
#include "ax/core/init.hpp"
#include "ax/fem/elasticity/base.hpp"
#include "ax/fem/elasticity/compute.hpp"

using namespace ax;

math::RealMatrixX reference;

void set_reference(BufferView<Real> bufv) {
  auto dim = bufv.Shape().X();
  math::Map<math::RealMatrixX> mat(bufv.Data(), static_cast<Index>(dim), static_cast<Index>(dim));
  mat = reference;
}

int main(int argc, char* argv[]) {
  po::add_option({
    po::make_option<int>("dim", "Dimension of problem", "3"),
    po::make_option<std::string>("elasticity", "Elasticity kind", "StVK"),
    po::make_option("gpu", "Use GPU"),
  });

  initialize(argc, argv);

  auto dim = po::get_parse_result()["dim"].as<int>();
  auto elasticity = po::get_parse_result()["elasticity"].as<std::string>();
  auto gpu = po::get_parse_result().count("gpu");

  reference.setRandom(dim, dim);
  reference = reference * reference.transpose() * 0.1;
  reference += math::RealMatrixX::Identity(dim, dim);

  auto lame = fem::elasticity::compute_lame(1e7, 0.4);

  std::cout << "Lame parameters:\n" << lame << std::endl;

  std::cout << "Reference matrix:\n" << reference << std::endl;

  fem::ElasticityBatchedCompute compute(1, 3, gpu ? BufferDevice::Device : BufferDevice::Host);
  copy(compute.Lame()->View(), view_from_matrix(lame));
  std::string elasticity_kind = po::get_parse_result()["elasticity"].as<std::string>();
  compute.SetElasitcityKind(utils::reflect_enum<fem::ElasticityKind>(elasticity_kind).value());
  auto dg_buf = create_buffer<Real>(ax::BufferDevice::Host,
                                    {static_cast<size_t>(dim), static_cast<size_t>(dim), 1});
  auto dg = dg_buf->View();

  // Initialize with identity.
  set_reference(dg);

  // Do finite difference to check the gradient.
  auto host_energy = create_buffer<Real>(ax::BufferDevice::Host, {1});
  auto energy = [&](BufferView<Real> dg) {
    compute.UpdateDeformationGradient(dg);
    compute.UpdateEnergyDensity();
    copy(host_energy->View(), compute.EnergyDensity()->View());
    return host_energy->View().Data()[0];
  };

  auto host_grad = create_buffer<Real>(ax::BufferDevice::Host, compute.Pk1()->Shape());
  auto gradient = [&](BufferView<Real> dg) -> math::RealMatrixX {
    compute.UpdateDeformationGradient(dg);
    compute.UpdateEnergyDensity();
    compute.UpdateGradient();
    copy(host_grad->View(), compute.Pk1()->View());
    return math::Map<math::RealMatrixX>(host_grad->View().Data(), dim, dim);
  };

  auto host_hes = create_buffer<Real>(ax::BufferDevice::Host, compute.LocalHessian()->Shape());
  auto hessian = [&](BufferView<Real> dg) -> math::RealMatrixX {
    compute.UpdateDeformationGradient(dg);
    compute.UpdateEnergyDensity();
    compute.UpdateGradient();
    compute.UpdateHessian();
    copy(host_hes->View(), compute.LocalHessian()->View());
    return math::Map<math::RealMatrixX>(host_hes->View().Data(), dim * dim, dim * dim);
  };

  for (int j = 0; j < dim; ++j) {
    for (int i = 0; i < dim; ++i) {
      set_reference(dg);
      dg.Data()[i + j * dim] += 1e-6;
      auto e1 = energy(dg);

      set_reference(dg);
      dg.Data()[i + j * dim] -= 1e-6;
      auto e2 = energy(dg);

      auto g = (e1 - e2) / 2e-6;
      set_reference(dg);
      auto g2 = gradient(dg)(i, j);
      AX_INFO("FD {} {}: {:12.6e}, Analytical: {:12.6e}", i, j, g, g2);
    }
  }

  set_reference(dg);
  compute.UpdateDeformationGradient(dg);
  compute.UpdateHessian();
  auto hes_buf = create_buffer<Real>(ax::BufferDevice::Host, compute.LocalHessian()->Shape());
  copy(hes_buf->View(), compute.LocalHessian()->View());
  auto hes = hes_buf->View();
  for (int j = 0; j < dim; ++j) {
    for (int i = 0; i < dim; ++i) {
      set_reference(dg);
      dg.Data()[i + j * dim] += 1e-6;
      auto g_plus = gradient(dg);

      set_reference(dg);
      dg.Data()[i + j * dim] -= 1e-6;
      auto g_minus = gradient(dg);

      math::RealMatrixX g = (g_plus - g_minus) / 2e-6;

      for (int l = 0; l < dim; ++l) {
        for (int k = 0; k < dim; ++k) {
          size_t row = i + j * dim;
          size_t col = k + l * dim;

          AX_INFO("FD {} {} {} {}: {:12.6e}, Analytical: {:12.6e}", i, j, k, l, g(k, l),
                  hes(row, col));
        }
      }
    }
  }

  return 0;
}