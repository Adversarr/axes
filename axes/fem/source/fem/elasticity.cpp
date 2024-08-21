#include "ax/fem/elasticity.hpp"

namespace ax::fem {

template <int dim>
ElasticityComputeBase<dim>::ElasticityComputeBase(std::shared_ptr<TriMesh<dim>> mesh)
    : mesh_(mesh), rinv_(static_cast<size_t>(mesh->GetNumElements())) {}

template <int dim> void ElasticityComputeBase<dim>::SetMesh(const MeshPtr& mesh) {
  this->mesh_ = mesh;
  RecomputeRestPose();
}

template <int dim> void ElasticityComputeBase<dim>::RecomputeRestPose() {
  // Prepare all the buffers.
  Index n_elem = mesh_->GetNumElements();
  Index n_vert = mesh_->GetNumVertices();

  energy_on_elements_.resize(1, n_elem);
  energy_on_vertices_.resize(1, n_vert);
  stress_on_elements_.resize(static_cast<size_t>(n_elem));
  stress_on_vertices_.resize(dim, n_vert);
  hessian_on_elements_.resize(static_cast<size_t>(n_elem));
}

template <int dim> void ElasticityComputeBase<dim>::SetLame(math::RealVector2 const& u_lame) {
  lame_.resize(2, mesh_->GetNumElements());
  lame_.colwise() = u_lame;
}

template <int dim> void ElasticityComputeBase<dim>::SetLame(math::RealField2 const& e_lame) {
  AX_CHECK(e_lame.cols() == mesh_->GetNumElements(), "Lame parameters size mismatch.");
  lame_ = e_lame;
}

template class ElasticityComputeBase<2>;
template class ElasticityComputeBase<3>;

}  // namespace ax::fem