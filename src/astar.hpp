#pragma once

#include "geo.hpp"
#include "graph.hpp"
#include "router.hpp"
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
 *    possible path between any two coordinates on the Earth's surface. Since
 *    actual road distance w(u, v) >= haversine(u, v), scaling by 0.999 and
 *    applying floor ensures that h(v) <= shortest_road_distance(v, t) for all
 * v. Thus, h is strictly admissible.
 *
 * 2. Consistency (Monotonicity): By the spherical triangle inequality,
 *    haversine(u, t) <= haversine(u, v) + haversine(v, t).
 *    Because edge weights w(u, v) >= haversine(u, v), we have:
 *    h(u) <= w(u, v) + h(v), meaning reduced edge costs are always
 * non-negative.
 *
 * 3. Closed Set Safety: Consistency guarantees that when a node u is extracted
 *    from the priority queue (settled), its g-score (dist[u]) is already
 * optimal. No subsequent relaxation can discover a shorter path to u.
 * Therefore, settled nodes never need to be reopened, making a closed set
 * completely safe and optimal.
 */

// A* heuristic calculation: floor(haversine_m to target * 0.999)
inline Dist astar_heuristic(const Graph &g, uint32_t u, uint32_t t) {
  double d_m = haversine_m(g.lat[u], g.lon[u], g.lat[t], g.lon[t]);
  return static_cast<Dist>(floor(d_m * 0.999));
}

class AStarRouter : public Router {
private:
  const Graph &g;

  // Bumping stamp logically resets node states in O(1) without touching memory.
  // Any node v with seen[v] != stamp is treated as unvisited (dist[v] = INF).
  // Similarly, done[v] == stamp tracks settled nodes for the current search.
  // This avoids expensive O(n) reallocations or vector clears between queries.
  vector<Dist> dist;
  vector<uint32_t> parent;
  vector<uint32_t> seen;
  vector<uint32_t> done;
  uint32_t stamp = 0;

public:
  explicit AStarRouter(const Graph &graph)
      : g(graph), dist(graph.size()), parent(graph.size()),
        seen(graph.size(), 0), done(graph.size(), 0), stamp(0) {}

  const char *name() const override { return "A*"; }

  Result query(uint32_t s, uint32_t t) override {
    stamp++;
    Result res;

    // Boundary check for valid source and target vertex IDs
    if (s >= g.size() || t >= g.size()) {
      return res;
    }

    auto get_dist = [&](uint32_t v) -> Dist {
      return seen[v] == stamp ? dist[v] : INF;
    };

    // Min-priority queue storing pairs of (f-score = g + h, vertex_id)
    using State = pair<Dist, uint32_t>;
    priority_queue<State, vector<State>, greater<State>> pq;

    seen[s] = stamp;
    dist[s] = 0;
    parent[s] = UINT32_MAX;
    Dist h_start = astar_heuristic(g, s, t);
    pq.emplace(h_start, s);

    while (!pq.empty()) {
      auto [f, u] = pq.top();
      pq.pop();

      // Skip nodes already settled (closed set check via timestamp)
      if (done[u] == stamp) {
        continue;
      }

      // Finalize node u
      done[u] = stamp;
      res.settled++;

      // Early stopping
      if (u == t) {
        break;
      }

      for (const auto &edge : g.adj[u]) {
        if (done[edge.to] == stamp) {
          continue;
        }

        Dist tentative_g = get_dist(u) + edge.w;
        if (tentative_g < get_dist(edge.to)) {
          seen[edge.to] = stamp;
          dist[edge.to] = tentative_g;
          parent[edge.to] = u;
          Dist f_next = tentative_g + astar_heuristic(g, edge.to, t);
          pq.emplace(f_next, edge.to);
        }
      }
    }

    // Target remained unreachable
    Dist target_dist = get_dist(t);
    if (target_dist == INF) {
      return res;
    }

    // Reconstruct path by walking backwards from target to source
    res.dist = target_dist;
    for (uint32_t curr = t; curr != UINT32_MAX;
         curr = (seen[curr] == stamp) ? parent[curr] : UINT32_MAX) {
      res.path.push_back(curr);
      if (curr == s) {
        break;
      }
    }
    reverse(res.path.begin(), res.path.end());

    return res;
  }
};

// One-line wrapper for backward compatibility
inline Result astar(const Graph &g, uint32_t s, uint32_t t) {
  return AStarRouter(g).query(s, t);
}
