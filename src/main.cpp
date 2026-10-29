#include "osm_loader.hpp"
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>

using namespace std;

int main(int argc, char *argv[]) {

  if (argc < 3 || string(argv[1]) != "stats") {
    cerr << "Usage: " << argv[0] << " stats <osm-pbf-file>\n";
    return 1;
  }

  string path = argv[2];
  if (!filesystem::exists(path) && filesystem::exists("data/" + path)) {
    path = "data/" + path;
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
