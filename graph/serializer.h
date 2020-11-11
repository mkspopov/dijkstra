//
// Created by mkspopov on 11.11.2020.
//

#ifndef DIJKSTRA_SERIALIZER_H
#define DIJKSTRA_SERIALIZER_H

#include "graph.h"

#include <fstream>
#include <string>

class GraphDeserializer {
public:
    explicit GraphDeserializer(const std::string& filename);

    Graph DimacsDeserialize();

private:
    std::ifstream ifStream_;
};


#endif //DIJKSTRA_SERIALIZER_H
