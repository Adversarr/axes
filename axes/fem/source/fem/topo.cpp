#include "ax/fem/topo.hpp"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/bandwidth.hpp>
#include <boost/graph/cuthill_mckee_ordering.hpp>
#include <boost/graph/properties.hpp>

namespace ax::fem {

template <idx dim>
std::pair<std::vector<idx>, std::vector<idx>> optimize_topology(math::fieldi<dim + 1> const& topo, idx n_vert) {
  using namespace boost;
  using namespace std;
  typedef adjacency_list<
      vecS, vecS, undirectedS,
      property<vertex_color_t, default_color_type, property<vertex_degree_t, int>>>
      Graph;
  typedef graph_traits<Graph>::vertex_descriptor Vertex;
  typedef graph_traits<Graph>::vertices_size_type size_type;

  // Perform Reverse Cuthill-McKee Ordering Algorithm
  // to optimize the topology of the mesh.

  // Create a graph from the topology.

  Graph G(static_cast<size_t>(n_vert));
  for (auto const& elem : math::each(topo)) {
    for (idx i = 0; i < dim + 1; i++) {
      for (idx j = i + 1; j < dim + 1; j++) {
        size_t ei = static_cast<size_t>(elem(i)), ej = static_cast<size_t>(elem(j));
        boost::add_edge(ei, ej, G);
      }
    }
  }

  graph_traits<Graph>::vertex_iterator ui, ui_end;

  property_map<Graph, vertex_degree_t>::type deg = get(vertex_degree, G);
  for (boost::tie(ui, ui_end) = vertices(G); ui != ui_end; ++ui) {
    deg[*ui] = degree(*ui, G);
  }

  property_map<Graph, vertex_index_t>::type index_map = get(vertex_index, G);

  std::vector<Vertex> inv_perm(num_vertices(G));
  std::vector<size_type> perm(num_vertices(G));
  {
    cuthill_mckee_ordering(G, inv_perm.rbegin(), get(vertex_color, G), make_degree_map(G));

    // cout << "Reverse Cuthill-McKee ordering:" << endl;
    // cout << "  ";
    // for (std::vector<Vertex>::const_iterator i = inv_perm.begin(); i != inv_perm.end(); ++i)
    //   cout << index_map[*i] << " ";
    // cout << endl;

    for (size_type c = 0; c != inv_perm.size(); ++c) perm[index_map[inv_perm[c]]] = c;
  }

  std::vector<idx> p, ip;
  p.assign(perm.begin(), perm.end());
  ip.assign(inv_perm.begin(), inv_perm.end());

  return {p, ip};
}

template std::pair<std::vector<idx>, std::vector<idx>> optimize_topology<1>(math::fieldi<2> const&, idx);
template std::pair<std::vector<idx>, std::vector<idx>> optimize_topology<2>(math::fieldi<3> const&, idx);
template std::pair<std::vector<idx>, std::vector<idx>> optimize_topology<3>(math::fieldi<4> const&, idx);

}  // namespace ax::fem
