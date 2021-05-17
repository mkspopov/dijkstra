#pragma once

#include "graph.h"
#include "topology.h"

CompactTopology BuildSimplyTopology(const WGraph& graph, LevelId levels);

CompactTopology BuildTopologyInertialFlow(const WGraph& graph, LevelId levels);
