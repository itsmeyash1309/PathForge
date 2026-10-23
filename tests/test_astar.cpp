#undef NDEBUG
#include "../src/astar.hpp"
#include "../src/dijkstra.hpp"
#include "../src/geo.hpp"
#include "../src/graph.hpp"
#include "../src/router.hpp"
#include <cassert>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

using namespace std;

// Generate a random geometric graph with n nodes in a bounding box
Graph generate_random_geometric_graph(size_t n, size_t extra_edges,
                                      uint32_t seed = 42) {
  mt19937 rng(seed);
  uniform_real_distribution<double> lat_dist(48.0, 48.1);
  uniform_real_distribution<double> lon_dist(11.5, 11.6);

  Graph g(n);

  // Assign random geographic coordinates to all nodes
  for (size_t i = 0; i < n; ++i) {
    g.set_coords(static_cast<uint32_t>(i), lat_dist(rng), lon_dist(rng));
  }

  // Connect all nodes in a ring to guarantee graph connectivity
  for (size_t i = 0; i < n; ++i) {
    uint32_t u = static_cast<uint32_t>(i);
    uint32_t v = static_cast<uint32_t>((i + 1) % n);
    double dist_m = haversine_m(g.lat[u], g.lon[u], g.lat[v], g.lon[v]);
    Dist w = max<Dist>(1, static_cast<Dist>(ceil(dist_m)));
    g.add_undirected(u, v, w);
  }

  // Add extra random undirected edges
  uniform_int_distribution<uint32_t> node_dist(0, static_cast<uint32_t>(n - 1));
  for (size_t i = 0; i < extra_edges; ++i) {
    uint32_t u = node_dist(rng);
    uint32_t v = node_dist(rng);
    if (u == v) {
      continue;
    }
    double dist_m = haversine_m(g.lat[u], g.lon[u], g.lat[v], g.lon[v]);
    Dist w = max<Dist>(1, static_cast<Dist>(ceil(dist_m)));
    g.add_undirected(u, v, w);
  }

  return g;
}

int main() {
  const size_t NUM_NODES = 2000;
  const size_t EXTRA_EDGES = 4000;
  const size_t NUM_QUERIES = 300;

  // Create random geometric graph
  Graph g = generate_random_geometric_graph(NUM_NODES, EXTRA_EDGES, 1337);

  // Instantiate routers with reusable workspaces
  DijkstraRouter dijkstra_router(g);
  AStarRouter astar_router(g);

  assert(strcmp(dijkstra_router.name(), "Dijkstra") == 0);
  assert(strcmp(astar_router.name(), "A*") == 0);

  mt19937 rng(999);
  uniform_int_distribution<uint32_t> node_dist(
      0, static_cast<uint32_t>(NUM_NODES - 1));

  size_t total_settled_dijkstra = 0;
  size_t total_settled_astar = 0;

  for (size_t q = 0; q < NUM_QUERIES; ++q) {
    uint32_t s = node_dist(rng);
    uint32_t t = node_dist(rng);

    Result d_res = dijkstra_router.query(s, t);
    Result a_res = astar_router.query(s, t);

    // 1. Both algorithms must find the exact same shortest distance
    assert(a_res.dist == d_res.dist);

    // 2. Validate path properties
    if (s == t) {
      assert(a_res.dist == 0);
      const vector<uint32_t> expected_path = {s};
      assert(a_res.path == expected_path);
    } else {
      assert(!a_res.path.empty());
      assert(a_res.path.front() == s);
      assert(a_res.path.back() == t);

      // 3. Verify that the sum of edge weights along the path equals the
      // computed distance
      Dist path_weight_sum = 0;
      for (size_t i = 0; i + 1 < a_res.path.size(); ++i) {
        uint32_t u = a_res.path[i];
        uint32_t v = a_res.path[i + 1];

        Dist best_edge_weight = INF;
        for (const auto &edge : g.adj[u]) {
          if (edge.to == v) {
            best_edge_weight = min(best_edge_weight, edge.w);
          }
        }
        assert(best_edge_weight != INF);
        path_weight_sum += best_edge_weight;
      }
      assert(path_weight_sum == a_res.dist);
    }

    total_settled_dijkstra += d_res.settled;
    total_settled_astar += a_res.settled;
  }

  // 50 consecutive queries on same router vs fresh router check
  mt19937 rng_check(4242);
  AStarRouter reused_astar(g);
  for (size_t q = 0; q < 50; ++q) {
    uint32_t s = node_dist(rng_check);
    uint32_t t = node_dist(rng_check);
    Result reused_res = reused_astar.query(s, t);
    Result fresh_res = AStarRouter(g).query(s, t);
    assert(reused_res.dist == fresh_res.dist);
    assert(reused_res.path == fresh_res.path);
    assert(reused_res.settled == fresh_res.settled);
  }

  double avg_dijkstra =
      static_cast<double>(total_settled_dijkstra) / NUM_QUERIES;
  double avg_astar = static_cast<double>(total_settled_astar) / NUM_QUERIES;

  cout << fixed << setprecision(2);
  cout << "Completed " << NUM_QUERIES << " random queries on " << NUM_NODES
       << "-node graph.\n";
  cout << "Average settled nodes (Dijkstra): " << avg_dijkstra << "\n";
  cout << "Average settled nodes (A*):       " << avg_astar << "\n";
  cout << "Speedup in search space:          " << (avg_dijkstra / avg_astar)
       << "x\n";

  return 0;
}
