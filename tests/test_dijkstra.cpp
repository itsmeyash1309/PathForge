#undef NDEBUG
#include "../src/dijkstra.hpp"
#include "../src/graph.hpp"
#include <cassert>
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

  Result res = dijkstra(g, 0, 7);
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

  Result res = dijkstra(g, 0, 4);
  assert(res.dist == INF);
  assert(res.path.empty());
}

// Test one-way road (directed edge)
void test_one_way_edge() {
  Graph g(3);
  g.add_edge(0, 1, 10);
  g.add_edge(1, 2, 10);

  // Forward path traversable
  Result fwd = dijkstra(g, 0, 2);
  assert(fwd.dist == 20);
  const vector<uint32_t> expected_path = {0, 1, 2};
  assert(fwd.path == expected_path);

  // Reverse path not traversable
  Result rev = dijkstra(g, 2, 0);
  assert(rev.dist == INF);
  assert(rev.path.empty());
}

int main() {
  test_eight_node_graph();   // VERIFIED
  test_unreachable_target(); // VERIFIED

  test_one_way_edge(); // VERIFIED

  cout << "All Dijkstra tests passed successfully.\n";
  return 0;
}
