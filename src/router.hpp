#pragma once

#include "graph.hpp"
#include <cstddef>
#include <cstdint>
#include <vector>

using namespace std;

// our final outcome
struct Result {
  Dist dist = INF; // Shortest distance found in metres (INF if unreachable)
  vector<uint32_t> path; // Sequence of vertex IDs from source to target
  size_t settled = 0;    // Number of nodes settled during the search
};

class Router {
public:
  virtual ~Router() = default;

  // Compute shortest path from source s to target t
  virtual Result query(uint32_t s, uint32_t t) = 0;

  // Descriptive name of the routing algorithm
  virtual const char *name() const = 0;
};
