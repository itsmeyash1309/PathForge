#pragma once

#include <algorithm>
#include <cstdint>
#include <limits>
#include <vector>

using namespace std;

// Distance type representing physical road distance in metres
using Dist = uint32_t;

// Infinity constant for unreachable distances (avoid overflow on addition)
constexpr Dist INF = numeric_limits<Dist>::max() / 2;

// Directed weighted edge to a neighbor
struct Edge {
  uint32_t to; // Target vertex index
  Dist w;      // Edge weight in metres
};

// Adjacency list representation of the road network graph
struct Graph {
  vector<vector<Edge>> adj;
  vector<double> lat; // Latitude coordinate in degrees per node
  vector<double> lon; // Longitude coordinate in degrees per node

  // Construct a graph with an optional initial number of vertices
  explicit Graph(size_t n = 0) : adj(n), lat(n, 0.0), lon(n, 0.0) {}

  // Add a directed edge from vertex u to vertex v with weight w
  void add_edge(uint32_t u, uint32_t v, Dist w) {
    size_t needed = max(u, v) + 1;
    if (needed > adj.size()) {
      adj.resize(needed);
      lat.resize(needed, 0.0);
      lon.resize(needed, 0.0);
    }
    adj[u].push_back({v, w});
  }

  // Add a two-way (bidirectional) road between vertex u and vertex v
  void add_undirected(uint32_t u, uint32_t v, Dist w) {
    add_edge(u, v, w);
    add_edge(v, u, w);
  }

  // Assign geographic coordinates to a node
  void set_coords(uint32_t u, double latitude, double longitude) {
    if (u >= adj.size()) {
      adj.resize(u + 1);
      lat.resize(u + 1, 0.0);
      lon.resize(u + 1, 0.0);
    }
    lat[u] = latitude;
    lon[u] = longitude;
  }

  // Total number of vertices currently in the graph
  size_t size() const { return adj.size(); }
};
