#pragma once

#include "graph.h"
#include "topology.h"

Topology BuildSimplyTopology(const WGraph& graph, LevelId levels);

Topology BuildInertialFlow(const CoordGraph& graph, LevelId levels, double coef = 1.0 / 4, int steps = 1);
