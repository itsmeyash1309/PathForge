#pragma once

#include "dijkstra.hpp"
#include "geo.hpp"
#include "graph.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>
#include <utility>
#include <vector>

using namespace std;

/*
 * Admissibility and Consistency of Heuristic:
 * 1. Admissibility: The great-circle (Haversine) distance is the shortest
 * possible path between any two coordinates on the Earth's surface. Since
 * actual road distance w(u, v) >= haversine(u, v), scaling by 0.999 and
 * applying floor ensures that h(v) <= shortest_road_distance(v, t) for all v.
 * Thus, h is strictly admissible.
 *
 * 2. Consistency (Monotonicity): By the spherical triangle inequality,
 *    haversine(u, t) <= haversine(u, v) + haversine(v, t).
 *    Because edge weights w(u, v) >= haversine(u, v), we have:
 *    h(u) <= w(u, v) + h(v), meaning reduced edge costs are always
 * non-negative.
 *
 * 3. Closed Set Safety: Consistency guarantees that when a node u is extracted
 * from the priority queue (settled), its g-score (dist[u]) is already optimal.
 * No subsequent relaxation can discover a shorter path to u. Therefore, settled
 * nodes never need to be reopened, making a closed set completely safe and
 * optimal.
 */

// A* heuristic calculation: floor(haversine_m to target * 0.999)
inline Dist astar_heuristic(const Graph &g, uint32_t u, uint32_t t) {
  double d_m = haversine_m(g.lat[u], g.lon[u], g.lat[t], g.lon[t]);
  return static_cast<Dist>(floor(d_m * 0.999));
}

// A* shortest path search using physical road distance and geometric heuristic
inline Result astar(const Graph &g, uint32_t s, uint32_t t) {
  Result res;

  // Boundary check for valid source and target vertex IDs
  if (s >= g.size() || t >= g.size()) {
    return res;
  }

  // Best known road distance g(u) from source to each node
  vector<Dist> dist(g.size(), INF);

  // Predecessor pointers to reconstruct the shortest path
  vector<uint32_t> prev(g.size(), UINT32_MAX);

  // Closed set to mark settled nodes
  vector<bool> closed(g.size(), false);

  // Min-priority queue storing pairs of (f-score = g + h, vertex_id)
  using State = pair<Dist, uint32_t>;
  priority_queue<State, vector<State>, greater<State>> pq;

  // Initialize source node
  dist[s] = 0;
  Dist h_start = astar_heuristic(g, s, t);
  pq.emplace(h_start, s);

  while (!pq.empty()) {
    auto [f, u] = pq.top();
    pq.pop();

    // Skip nodes already settled (closed set check / lazy deletion)
    if (closed[u]) {
      continue;
    }

    // Finalize node u
    closed[u] = true;
    res.settled++;

    // Stop as soon as target node is settled
    if (u == t) {
      break;
    }

    // Relax outgoing edges from u
    for (const auto &edge : g.adj[u]) {
      if (closed[edge.to]) {
        continue;
      }

      Dist tentative_g = dist[u] + edge.w;
      if (tentative_g < dist[edge.to]) {
        dist[edge.to] = tentative_g;
        prev[edge.to] = u;
        Dist f_next = tentative_g + astar_heuristic(g, edge.to, t);
        pq.emplace(f_next, edge.to);
      }
    }
  }

  // Target remained unreachable
  if (dist[t] == INF) {
    return res;
  }

  // Reconstruct shortest path from target back to source
  res.dist = dist[t];
  for (uint32_t curr = t; curr != UINT32_MAX; curr = prev[curr]) {
    res.path.push_back(curr);
    if (curr == s) {
      break;
    }
  }
  reverse(res.path.begin(), res.path.end());

  return res;
}
