#pragma once

#include "graph.h"

#include <filesystem>

class GraphDeserializer {
public:
    explicit GraphDeserializer(std::filesystem::path path);

    CoordGraph ReadCoordinates();
    WGraph ReadDistances();
    WGraph ReadTravelTimes();

private:
    WGraph ReadGraph();

    std::filesystem::path path_;
};
