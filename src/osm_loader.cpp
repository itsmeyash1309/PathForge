#include "osm_loader.hpp"
#include "geo.hpp"

#include <osmium/handler.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>
#include <osmium/index/map/flex_mem.hpp>
#include <osmium/io/pbf_input.hpp>
#include <osmium/visitor.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace std;

using index_type = osmium::index::map::FlexMem<osmium::unsigned_object_id_type,
                                               osmium::Location>;
using location_handler_type = osmium::handler::NodeLocationsForWays<index_type>;

struct NodeCoord {
  double lat;
  double lon;
};

struct EdgeData {
  uint32_t u;
  uint32_t v;
  Dist w;
};

struct DSU {
  vector<uint32_t> parent;
  vector<uint32_t> sz;

  explicit DSU(size_t n) : parent(n), sz(n, 1) {
    for (size_t i = 0; i < n; ++i) {
      parent[i] = static_cast<uint32_t>(i);
    }
  }

  uint32_t find(uint32_t x) {
    if (parent[x] != x) {
      parent[x] = find(parent[x]);
    }
    return parent[x];
  }

  void unite(uint32_t x, uint32_t y) {
    uint32_t rx = find(x);
    uint32_t ry = find(y);
    if (rx != ry) {
      if (sz[rx] < sz[ry]) {
        swap(rx, ry);
      }
      parent[ry] = rx;
      sz[rx] += sz[ry];
    }
  }
};

static bool is_valid_highway(const char *highway) {
  if (!highway) {
    return false;
  }
  static const unordered_set<string_view> valid_highways = {
      "motorway",       "trunk",         "primary",     "secondary",
      "tertiary",       "unclassified",  "residential", "living_street",
      "service",        "motorway_link", "trunk_link",  "primary_link",
      "secondary_link", "tertiary_link"};
  return valid_highways.count(highway) > 0;
}

static bool is_forbidden_access(const char *val) {
  if (!val) {
    return false;
  }
  return strcmp(val, "no") == 0 || strcmp(val, "private") == 0;
}

class RoadWayHandler : public osmium::handler::Handler {
public:
  unordered_map<osmium::object_id_type, uint32_t> osm_to_dense;
  vector<NodeCoord> node_coords;
  vector<EdgeData> edges;

  uint32_t get_or_create_node(osmium::object_id_type osm_id, double lat,
                              double lon) {
    auto it = osm_to_dense.find(osm_id);
    if (it != osm_to_dense.end()) {
      return it->second;
    }
    uint32_t id = static_cast<uint32_t>(node_coords.size());
    osm_to_dense[osm_id] = id;
    node_coords.push_back({lat, lon});
    return id;
  }

  void way(const osmium::Way &way) {
    const char *highway = way.tags()["highway"];
    if (!is_valid_highway(highway)) {
      return;
    }

    if (is_forbidden_access(way.tags()["motor_vehicle"]) ||
        is_forbidden_access(way.tags()["vehicle"]) ||
        is_forbidden_access(way.tags()["access"])) {
      return;
    }

    const char *oneway = way.tags()["oneway"];
    const char *junction = way.tags()["junction"];

    bool forward = false;
    bool backward = false;

    if (oneway && (strcmp(oneway, "yes") == 0 || strcmp(oneway, "true") == 0 ||
                   strcmp(oneway, "1") == 0)) {
      forward = true;
      backward = false;
    } else if (oneway && strcmp(oneway, "-1") == 0) {
      forward = false;
      backward = true;
    } else if (junction && strcmp(junction, "roundabout") == 0) {
      forward = true;
      backward = false;
    } else {
      forward = true;
      backward = true;
    }

    const auto &nodes = way.nodes();
    if (nodes.size() < 2) {
      return;
    }

    for (size_t i = 0; i + 1 < nodes.size(); ++i) {
      const auto &n1 = nodes[i];
      const auto &n2 = nodes[i + 1];

      if (!n1.location().valid() || !n2.location().valid()) {
        continue;
      }
      if (n1.ref() == n2.ref()) {
        continue;
      }

      double lat1 = n1.location().lat();
      double lon1 = n1.location().lon();
      double lat2 = n2.location().lat();
      double lon2 = n2.location().lon();

      Dist w = max<Dist>(
          1, static_cast<Dist>(ceil(haversine_m(lat1, lon1, lat2, lon2))));

      uint32_t u = get_or_create_node(n1.ref(), lat1, lon1);
      uint32_t v = get_or_create_node(n2.ref(), lat2, lon2);

      if (forward) {
        edges.push_back({u, v, w});
      }
      if (backward) {
        edges.push_back({v, u, w});
      }
    }
  }
};

Graph load_osm_pbf(const string &path) {
  index_type index;
  location_handler_type location_handler{index};
  location_handler.ignore_errors();

  RoadWayHandler way_handler;

  osmium::io::File input_file{path};
  osmium::io::Reader reader{input_file, osmium::osm_entity_bits::node |
                                            osmium::osm_entity_bits::way};
  osmium::apply(reader, location_handler, way_handler);
  reader.close();

  size_t n = way_handler.node_coords.size();
  if (n == 0) {
    return Graph(0);
  }

  DSU dsu(n);
  for (const auto &e : way_handler.edges) {
    dsu.unite(e.u, e.v);
  }

  uint32_t best_root = 0;
  uint32_t max_size = 0;
  for (uint32_t i = 0; i < n; ++i) {
    uint32_t root = dsu.find(i);
    if (dsu.sz[root] > max_size) {
      max_size = dsu.sz[root];
      best_root = root;
    }
  }

  vector<uint32_t> old_to_new(n, UINT32_MAX);
  uint32_t new_n = 0;
  for (uint32_t i = 0; i < n; ++i) {
    if (dsu.find(i) == best_root) {
      old_to_new[i] = new_n++;
    }
  }

  Graph g(new_n);
  for (uint32_t i = 0; i < n; ++i) {
    if (old_to_new[i] != UINT32_MAX) {
      g.set_coords(old_to_new[i], way_handler.node_coords[i].lat,
                   way_handler.node_coords[i].lon);
    }
  }

  for (const auto &e : way_handler.edges) {
    if (old_to_new[e.u] != UINT32_MAX && old_to_new[e.v] != UINT32_MAX) {
      g.add_edge(old_to_new[e.u], old_to_new[e.v], e.w);
    }
  }

  return g;
}
