#pragma once

#include "graph.hpp"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <queue>
#include <utility>
#include <vector>

using namespace std;

// Outcome of a shortest-path query
struct Result {
  Dist dist = INF;       // Shortest distance found (INF if unreachable)
  vector<uint32_t> path; // Sequence of vertex IDs from source to target
  size_t settled = 0;    // Number of nodes extracted and finalized
};

// Dijkstra's shortest path algorithm using a min-heap with lazy deletion
inline Result dijkstra(const Graph &g, uint32_t s, uint32_t t) {
  Result res;

  // Boundary check for valid source and target vertex IDs
  if (s >= g.size() || t >= g.size()) {
    return res;
  }

  // Best known distances from source to each node
  vector<Dist> dist(g.size(), INF);

  // Predecessor pointers to reconstruct the shortest path
  vector<uint32_t> prev(g.size(), UINT32_MAX);

  // Min-priority queue storing pairs of (distance, vertex_id)
  using State = pair<Dist, uint32_t>;
  priority_queue<State, vector<State>, greater<State>> pq;

  dist[s] = 0;
  pq.emplace(0, s);

  while (!pq.empty()) {
    auto [d, u] = pq.top();
    pq.pop();

    // Lazy deletion: ignore if we already found a shorter path to u
    if (d > dist[u]) {
      continue;
    }

    // Finalize node u (it is now settled)
    res.settled++;

    // Early stopping
    if (u == t) {
      break;
    }

    for (const auto &edge : g.adj[u]) {
      if (dist[u] + edge.w < dist[edge.to]) {
        dist[edge.to] = dist[u] + edge.w;
        prev[edge.to] = u;
        pq.emplace(dist[edge.to], edge.to);
      }
    }
  }

  // Target remained unreachable
  if (dist[t] == INF) {
    return res;
  }

  // Reconstruct path by walking backwards from target to source
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
