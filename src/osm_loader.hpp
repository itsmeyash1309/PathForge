#pragma once

#include "graph.hpp"
#include <string>

using namespace std;

Graph load_osm_pbf(const string &path);
