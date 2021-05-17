//
// Created by mkspopov on 11.11.2020.
//

#pragma once

#include "graph.h"

#include <fstream>
#include <string>

class GraphDeserializer {
public:
    explicit GraphDeserializer(const std::string& filename);

    WGraph DimacsDeserialize();

private:
    std::ifstream ifStream_;
};
