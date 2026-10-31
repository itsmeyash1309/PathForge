#include "astar.hpp"
#include "dijkstra.hpp"
#include "geo.hpp"
#include "graph.hpp"
#include "osm_loader.hpp"
#include "router.hpp"
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

using namespace std;

int main(int argc, char *argv[]) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " stats <osm-pbf-file>\n"
         << "       " << argv[0]
         << " route <osm-pbf-file> dijkstra|astar <lat1> <lon1> <lat2> <lon2>\n"
         << "       " << argv[0]
         << " bench <osm-pbf-file> [num_queries=100] [seed=1]\n";
    return 1;
  }

  string cmd = argv[1];

  if (cmd == "stats") {
    if (argc < 3) {
      cerr << "Usage: " << argv[0] << " stats <osm-pbf-file>\n";
      return 1;
    }

    string path = argv[2];
    if (!filesystem::exists(path) && filesystem::exists("data/" + path)) {
      path = "data/" + path;
    }
    if (!filesystem::exists(path)) {
      cerr << "Error: file not found: " << argv[2] << "\n";
      return 1;
    }

    auto start = chrono::high_resolution_clock::now();
    Graph g = load_osm_pbf(path);
    auto end = chrono::high_resolution_clock::now();

    chrono::duration<double> elapsed = end - start;

    int total_edges = 0;
    for (const auto &v_edges : g.adj) {
      total_edges += v_edges.size();
    }

    cout << "Nodes: " << g.size() << "\n";
    cout << "Edges: " << total_edges << "\n";
    cout << "Elapsed: " << fixed << setprecision(3) << elapsed.count() << " s\n";

    return 0;
  }

  if (cmd == "route") {
    if (argc < 8) {
      cerr << "Usage: " << argv[0]
           << " route <osm-pbf-file> dijkstra|astar <lat1> <lon1> <lat2> <lon2>\n";
      return 1;
    }

    string path = argv[2];
    if (!filesystem::exists(path) && filesystem::exists("data/" + path)) {
      path = "data/" + path;
    }
    if (!filesystem::exists(path)) {
      cerr << "Error: file not found: " << argv[2] << "\n";
      return 1;
    }

    string algo = argv[3];
    if (algo != "dijkstra" && algo != "astar") {
      cerr << "Error: unknown algorithm '" << algo
           << "'. Supported algorithms: dijkstra, astar\n";
      return 1;
    }

    double lat1 = 0.0;
    double lon1 = 0.0;
    double lat2 = 0.0;
    double lon2 = 0.0;
    try {
      lat1 = stod(argv[4]);
      lon1 = stod(argv[5]);
      lat2 = stod(argv[6]);
      lon2 = stod(argv[7]);
    } catch (const exception &) {
      cerr << "Error: invalid coordinate format\n";
      return 1;
    }

    Graph g = load_osm_pbf(path);
    if (g.size() == 0) {
      cerr << "Error: loaded graph is empty\n";
      return 1;
    }

    auto find_nearest_node = [&](double lat, double lon) -> uint32_t {
      uint32_t best_node = 0;
      double best_dist = numeric_limits<double>::infinity();
      for (uint32_t i = 0; i < g.size(); ++i) {
        double d = haversine_m(lat, lon, g.lat[i], g.lon[i]);
        if (d < best_dist) {
          best_dist = d;
          best_node = i;
        }
      }
      return best_node;
    };

    uint32_t start_node = find_nearest_node(lat1, lon1);
    uint32_t target_node = find_nearest_node(lat2, lon2);

    unique_ptr<Router> router;
    if (algo == "dijkstra") {
      router = make_unique<DijkstraRouter>(g);
    } else {
      router = make_unique<AStarRouter>(g);
    }

    Result res = router->query(start_node, target_node);
    if (res.dist == INF) {
      cerr << "Error: target is unreachable\n";
      return 1;
    }

    cout << "Router: " << router->name() << "\n";
    cout << "Distance: " << fixed << setprecision(2) << (res.dist / 1000.0) << " km\n";
    cout << "Settled nodes: " << res.settled << "\n";
    cout << "Path nodes: " << res.path.size() << "\n";
    cout << "Snapped start: " << fixed << setprecision(6) << g.lat[start_node]
         << ", " << g.lon[start_node] << "\n";
    cout << "Snapped end: " << fixed << setprecision(6) << g.lat[target_node]
         << ", " << g.lon[target_node] << "\n";

    ofstream out("path.csv");
    if (!out.is_open()) {
      cerr << "Error: could not open path.csv for writing\n";
      return 1;
    }
    for (uint32_t u : res.path) {
      out << fixed << setprecision(6) << g.lat[u] << "," << g.lon[u] << "\n";
    }
    out.close();

    return 0;
  }

  if (cmd == "bench") {
    if (argc < 3) {
      cerr << "Usage: " << argv[0]
           << " bench <osm-pbf-file> [num_queries=100] [seed=1]\n";
      return 1;
    }

    string path = argv[2];
    if (!filesystem::exists(path) && filesystem::exists("data/" + path)) {
      path = "data/" + path;
    }
    if (!filesystem::exists(path)) {
      cerr << "Error: file not found: " << argv[2] << "\n";
      return 1;
    }

    size_t num_queries = 100;
    if (argc >= 4) {
      try {
        num_queries = stoul(argv[3]);
      } catch (const exception &) {
        cerr << "Error: invalid num_queries\n";
        return 1;
      }
    }

    uint32_t seed = 1;
    if (argc >= 5) {
      try {
        seed = static_cast<uint32_t>(stoul(argv[4]));
      } catch (const exception &) {
        cerr << "Error: invalid seed\n";
        return 1;
      }
    }

    Graph g = load_osm_pbf(path);
    if (g.size() == 0) {
      cerr << "Error: loaded graph is empty\n";
      return 1;
    }

    vector<unique_ptr<Router>> routers;
    routers.push_back(make_unique<DijkstraRouter>(g));
    routers.push_back(make_unique<AStarRouter>(g));

    mt19937 rng(seed);
    uniform_int_distribution<uint32_t> dist_node(
        0, static_cast<uint32_t>(g.size() - 1));

    vector<pair<uint32_t, uint32_t>> queries;
    queries.reserve(num_queries);

    while (queries.size() < num_queries) {
      uint32_t s = dist_node(rng);
      uint32_t t = dist_node(rng);
      Result r = routers[0]->query(s, t);
      if (r.dist != INF) {
        queries.push_back({s, t});
      }
    }

    struct BenchmarkRow {
      string algo;
      double avg_settled = 0.0;
      double mean_us = 0.0;
      double p95_us = 0.0;
      double speedup = 1.0;
    };
    vector<BenchmarkRow> rows;
    rows.reserve(routers.size());

    vector<Dist> dijkstra_distances(num_queries);
    vector<Dist> astar_distances(num_queries);

    for (size_t r_idx = 0; r_idx < routers.size(); ++r_idx) {
      // Warm-up query
      if (!queries.empty()) {
        routers[r_idx]->query(queries[0].first, queries[0].second);
      }

      vector<int64_t> durations_us;
      durations_us.reserve(num_queries);
      uint64_t total_settled = 0;
      uint64_t total_us = 0;

      for (size_t q_idx = 0; q_idx < num_queries; ++q_idx) {
        auto [s, t] = queries[q_idx];

        auto t_start = chrono::steady_clock::now();
        Result res = routers[r_idx]->query(s, t);
        auto t_end = chrono::steady_clock::now();

        auto dur =
            chrono::duration_cast<chrono::microseconds>(t_end - t_start).count();
        durations_us.push_back(dur);
        total_us += static_cast<uint64_t>(dur);
        total_settled += res.settled;

        if (r_idx == 0) {
          dijkstra_distances[q_idx] = res.dist;
        } else {
          if (r_idx == 1) {
            astar_distances[q_idx] = res.dist;
          }
          if (res.dist != dijkstra_distances[q_idx]) {
            cerr << "Error: distance mismatch on query " << q_idx
                 << " (s=" << s << ", t=" << t << "): "
                 << routers[0]->name() << "=" << dijkstra_distances[q_idx]
                 << " m, " << routers[r_idx]->name() << "=" << res.dist
                 << " m\n";
            abort();
          }
        }
      }

      vector<int64_t> sorted_durs = durations_us;
      sort(sorted_durs.begin(), sorted_durs.end());
      size_t p95_idx = static_cast<size_t>(0.95 * sorted_durs.size());
      if (p95_idx >= sorted_durs.size()) {
        p95_idx = sorted_durs.size() - 1;
      }

      BenchmarkRow row;
      row.algo = routers[r_idx]->name();
      row.avg_settled = static_cast<double>(total_settled) / num_queries;
      row.mean_us = static_cast<double>(total_us) / num_queries;
      row.p95_us = static_cast<double>(sorted_durs[p95_idx]);
      rows.push_back(row);
    }

    double dijkstra_mean = rows[0].mean_us;
    for (auto &row : rows) {
      row.speedup = (row.mean_us > 0.0) ? (dijkstra_mean / row.mean_us) : 1.0;
    }

    size_t total_edges = 0;
    for (const auto &v_edges : g.adj) {
      total_edges += v_edges.size();
    }

    double total_dist_km = 0.0;
    for (size_t i = 0; i < num_queries; ++i) {
      total_dist_km += dijkstra_distances[i] / 1000.0;
    }
    double avg_distance_km = total_dist_km / num_queries;

    cout << "Graph: " << g.size() << " nodes, " << total_edges << " edges\n";
    cout << "Queries: " << num_queries << " (seed: " << seed << ")\n";
    cout << "Average route distance: " << fixed << setprecision(2)
         << avg_distance_km << " km\n\n";

    cout << "Distance comparison (first 5 queries):\n";
    size_t n_comp = min<size_t>(5, num_queries);
    for (size_t i = 0; i < n_comp; ++i) {
      cout << "  Query " << (i + 1) << ": Dijkstra = " << fixed
           << setprecision(2) << (dijkstra_distances[i] / 1000.0)
           << " km, A* = " << (astar_distances[i] / 1000.0) << " km\n";
    }
    cout << "\n";

    cout << "| algorithm | avg_settled_nodes | mean_us | p95_us | "
            "speedup_vs_dijkstra |\n";
    cout << "| :--- | :---: | :---: | :---: | :---: |\n";
    for (const auto &row : rows) {
      cout << "| " << left << setw(9) << row.algo << " | " << right
           << setw(17) << fixed << setprecision(1) << row.avg_settled << " | "
           << setw(7) << fixed << setprecision(1) << row.mean_us << " | "
           << setw(6) << fixed << setprecision(1) << row.p95_us << " | "
           << setw(18) << fixed << setprecision(2) << row.speedup << "x |\n";
    }

    return 0;
  }

  cerr << "Error: unknown command '" << cmd << "'\n";
  return 1;
}
