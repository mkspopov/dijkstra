#pragma once

#include "graph.h"
#include "topology.h"

CompactTopology BuildSimplyTopology(const WeightGraph<EdgeProperty>& graph, LevelId levels);

CompactTopology BuildTopologyInertialFlow(const WeightGraph<EdgeProperty>& graph, LevelId levels);
