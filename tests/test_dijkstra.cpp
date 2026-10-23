#undef NDEBUG
#include "../src/dijkstra.hpp"
#include "../src/graph.hpp"
#include "../src/router.hpp"
#include <cassert>
#include <cstring>
#include <iostream>
#include <vector>

using namespace std;

void test_eight_node_graph() {
  // Graph layout (0 to 7):
  // 0 -> 1 (4), 0 -> 2 (2)
  // 2 -> 1 (1) -> path 0-2-1 has distance 3
  // 1 -> 3 (5), 1 -> 4 (2) -> path 0-2-1-4 has distance 5
  // 2 -> 4 (8)
  // 3 -> 5 (6)
  // 4 -> 5 (3) -> path 0-2-1-4-5 has distance 8
  // 4 -> 6 (4) -> path 0-2-1-4-6 has distance 9
  // 5 -> 7 (2) -> path 0-2-1-4-5-7 has distance 10
  // 6 -> 7 (3) -> path 0-2-1-4-6-7 has distance 12
  Graph g(8);
  g.add_edge(0, 1, 4);
  g.add_edge(0, 2, 2);
  g.add_edge(2, 1, 1);
  g.add_edge(1, 3, 5);
  g.add_edge(1, 4, 2);
  g.add_edge(2, 4, 8);
  g.add_edge(3, 5, 6);
  g.add_edge(4, 5, 3);
  g.add_edge(4, 6, 4);
  g.add_edge(5, 7, 2);
  g.add_edge(6, 7, 3);

  DijkstraRouter router(g);
  assert(strcmp(router.name(), "Dijkstra") == 0);

  Result res = router.query(0, 7);
  assert(res.dist == 10);
  const vector<uint32_t> expected_path = {0, 2, 1, 4, 5, 7};
  assert(res.path == expected_path);
  assert(res.settled > 0);
  assert(res.settled <= 8);
}

// Test target node in a disconnected component
void test_unreachable_target() {
  Graph g(5);
  g.add_edge(0, 1, 10);
  g.add_edge(1, 2, 20);
  // Nodes 3 and 4 have no connection to nodes 0, 1, 2
  g.add_edge(3, 4, 5);

  DijkstraRouter router(g);
  Result res = router.query(0, 4);
  assert(res.dist == INF);
  assert(res.path.empty());
}

// Test one-way road (directed edge)
void test_one_way_edge() {
  Graph g(3);
  g.add_edge(0, 1, 10);
  g.add_edge(1, 2, 10);

  DijkstraRouter router(g);

  // Forward path traversable
  Result fwd = router.query(0, 2);
  assert(fwd.dist == 20);
  const vector<uint32_t> expected_path = {0, 1, 2};
  assert(fwd.path == expected_path);

  // Reverse path not traversable
  Result rev = router.query(2, 0);
  assert(rev.dist == INF);
  assert(rev.path.empty());
}

// Test 50 consecutive queries on the same router object vs fresh router
void test_consecutive_queries_reused_vs_fresh() {
  Graph g(10);
  for (uint32_t i = 0; i < 9; ++i) {
    g.add_edge(i, i + 1, (i + 1) * 3);
    g.add_edge(i + 1, i, (i + 1) * 3);
  }
  g.add_edge(0, 5, 20);
  g.add_edge(2, 8, 15);
  g.add_edge(3, 7, 10);

  DijkstraRouter router(g);
  for (int q = 0; q < 50; ++q) {
    uint32_t s = static_cast<uint32_t>((q * 7 + 3) % 10);
    uint32_t t = static_cast<uint32_t>((q * 11 + 5) % 10);
    Result res_reused = router.query(s, t);
    Result res_fresh = DijkstraRouter(g).query(s, t);
    assert(res_reused.dist == res_fresh.dist);
    assert(res_reused.path == res_fresh.path);
    assert(res_reused.settled == res_fresh.settled);
  }
}

int main() {
  test_eight_node_graph();
  test_unreachable_target();
  test_one_way_edge();
  test_consecutive_queries_reused_vs_fresh();

  cout << "All Dijkstra tests passed successfully.\n";
  return 0;
}
