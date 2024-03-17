// #include "axes/pde/advect/lattice.hpp"

// #include "axes/math/range.hpp"

// namespace ax::pde {

// template <idx dim>
// AX_FORCE_INLINE bool out_of_boundary(math::veci<dim> const& ijk, math::veci<dim> const& shape) {
//   for (idx d = 0; d < dim; ++d) {
//     if (ijk(d) < 0 || ijk(d) >= shape(d)) {
//       return true;
//     }
//   }
//   return false;
// }

// template <idx dim> math::vecr<dim> get(const math::Lattice<dim, math::vecr<dim>>& u,
//                                        math::veci<dim> const& ijk, bool periodic) {
//   math::vecr<dim> output = math::vecr<dim>::Zero();
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

// template <idx dim> math::vecr<dim> interpolate(const math::Lattice<dim, math::vecr<dim>>& u,
//                                                math::vecr<dim> const& X, bool periodic) {
//   math::vecr<dim> output = math::vecr<dim>::Zero();
//   math::veci<dim> ijk = X.template cast<idx>();
//   math::vecr<dim> delta = X - ijk.template cast<real>();
//   for (idx d = 0; d < dim; ++d) {
//     output(d) = (1.0 - delta(d)) * get<dim>(u, ijk, periodic)[d]
//                 + delta(d) * get<dim>(u, ijk + math::unit<dim, idx>(d), periodic)[d];
//   }
//   return output;
// }

// template<idx dim>
// math::vecr<dim> get(const math::StaggeredLattice<dim, math::vecr<dim>>& u,
//                     math::veci<dim> const& ijk, bool periodic) {
//   math::vecr<dim> output = math::vecr<dim>::Zero();
//   if (!out_of_boundary<dim>(ijk, u.Shape())) {
//     for (idx d = 0; d < dim; ++d) {
//       output[d] = u[d](ijk);
//     }
//   } else {
//     if (periodic) {
//       return u(math::imod<dim>(ijk, u.Shape()));
//     }
//   }
// }

// template <idx dim>
// StatusOr<math::StaggeredLattice<dim, real>> AdvectionProblem<dim>::AdvectVelocity(bool periodic) {
//   math::StaggeredLattice<dim, real> output(velocity_.Shape());
//   // Assume velocity = [u, v, w].
//   //    u_t + C [u v w] dot grad(u) = 0
//   for (auto ijk : utils::multi_iota<dim>(velocity_.Shape())) {
//     auto sub = math::tuple_to_vector<idx, dim>(ijk);
//     math::vecr<dim> X = sub.template cast<real>() - velocity_(sub) * dtdx_;
//     auto vel = interpolate<dim>(velocity_, X, periodic);
//     output(sub) = vel;
//   }
//   return output;
// }

// template class AdvectionProblem<2>;
// template class AdvectionProblem<3>;
// }  // namespace ax::pde