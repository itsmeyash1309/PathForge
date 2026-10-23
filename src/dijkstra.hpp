#pragma once

#include "graph.hpp"
#include "router.hpp"
#include <algorithm>
#include <cstdint>
#include <queue>
#include <utility>
#include <vector>

using namespace std;

// Dijkstra router holding graph reference and reusable query workspace
class DijkstraRouter : public Router {
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
  explicit DijkstraRouter(const Graph &graph)
      : g(graph), dist(graph.size()), parent(graph.size()),
        seen(graph.size(), 0), done(graph.size(), 0), stamp(0) {}

  const char *name() const override { return "Dijkstra"; }

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

    // Min-priority queue storing pairs of (distance, vertex_id)
    using State = pair<Dist, uint32_t>;
    priority_queue<State, vector<State>, greater<State>> pq;

    seen[s] = stamp;
    dist[s] = 0;
    parent[s] = UINT32_MAX;
    pq.emplace(0, s);

    while (!pq.empty()) {
      auto [d, u] = pq.top();
      pq.pop();

      // Lazy deletion: ignore if already settled
      if (done[u] == stamp) {
        continue;
      }

      // Finalize node u (it is now settled)
      done[u] = stamp;
      res.settled++;

      // Early stopping
      if (u == t) {
        break;
      }

      for (const auto &edge : g.adj[u]) {
        Dist tentative = get_dist(u) + edge.w;
        if (tentative < get_dist(edge.to)) {
          seen[edge.to] = stamp;
          dist[edge.to] = tentative;
          parent[edge.to] = u;
          pq.emplace(tentative, edge.to);
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
inline Result dijkstra(const Graph &g, uint32_t s, uint32_t t) {
  return DijkstraRouter(g).query(s, t);
}
