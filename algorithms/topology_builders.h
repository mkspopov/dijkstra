#pragma once

#include "graph.h"
#include "topology.h"

CompactTopology BuildSimplyTopology(const WGraph& graph, LevelId levels);

CompactTopology BuildTopologyInertialFlow(const CoordGraph& graph, LevelId levels, double coef = 1.0 / 4, int steps = 0);
