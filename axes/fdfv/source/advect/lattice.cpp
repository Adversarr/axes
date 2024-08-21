#include "ax/fdfv/advect/lattice.hpp"

// #include "ax/math/range.hpp"

// namespace ax::pde {

// template <int dim>
// AX_FORCE_INLINE bool out_of_boundary(math::IndexVector<dim> const& ijk, math::IndexVector<dim> const& shape) {
//   for (Index d = 0; d < dim; ++d) {
//     if (ijk(d) < 0 || ijk(d) >= shape(d)) {
//       return true;
//     }
//   }
//   return false;
// }

// template <int dim> math::RealVector<dim> get(const math::Lattice<dim, math::RealVector<dim>>& u,
//                                        math::IndexVector<dim> const& ijk, bool periodic) {
//   math::RealVector<dim> output = math::RealVector<dim>::Zero();
//   if (!out_of_boundary<dim>(ijk, u.Shape())) {
//     return u(ijk);
//   } else {
//     if (periodic) {
//       return u(math::imod<dim>(ijk, u.Shape()));
//     } else {
//       return output;
//     }
//   }
// }

// template <int dim> math::RealVector<dim> interpolate(const math::Lattice<dim, math::RealVector<dim>>& u,
//                                                math::RealVector<dim> const& X, bool periodic) {
//   math::RealVector<dim> output = math::RealVector<dim>::Zero();
//   math::IndexVector<dim> ijk = X.template cast<Index>();
//   math::RealVector<dim> delta = X - ijk.template cast<Real>();
//   for (Index d = 0; d < dim; ++d) {
//     output(d) = (1.0 - delta(d)) * get<dim>(u, ijk, periodic)[d]
//                 + delta(d) * get<dim>(u, ijk + math::unit<dim, Index>(d), periodic)[d];
//   }
//   return output;
// }

// template<int dim>
// math::RealVector<dim> get(const math::StaggeredLattice<dim, math::RealVector<dim>>& u,
//                     math::IndexVector<dim> const& ijk, bool periodic) {
//   math::RealVector<dim> output = math::RealVector<dim>::Zero();
//   if (!out_of_boundary<dim>(ijk, u.Shape())) {
//     for (Index d = 0; d < dim; ++d) {
//       output[d] = u[d](ijk);
//     }
//   } else {
//     if (periodic) {
//       return u(math::imod<dim>(ijk, u.Shape()));
//     }
//   }
// }

// template <int dim>
// StatusOr<math::StaggeredLattice<dim, Real>> AdvectionProblem<dim>::AdvectVelocity(bool periodic) {
//   math::StaggeredLattice<dim, Real> output(velocity_.Shape());
//   // Assume velocity = [u, v, w].
//   //    u_t + C [u v w] dot grad(u) = 0
//   for (auto ijk : utils::ndrange<dim>(velocity_.Shape())) {
//     auto sub = math::tuple_to_vector<Index, dim>(ijk);
//     math::RealVector<dim> X = sub.template cast<Real>() - velocity_(sub) * dtdx_;
//     auto vel = interpolate<dim>(velocity_, X, periodic);
//     output(sub) = vel;
//   }
//   return output;
// }

// template class AdvectionProblem<2>;
// template class AdvectionProblem<3>;
// }  // namespace ax::pde