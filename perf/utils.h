//
// Created by mkspopov on 19.04.2021.
//

#include "../utils/types.h"
#include "../utils/utils.h"

#include <vector>

std::vector<VertexId> GenerateRandomVertices(int vertexCount, int maxVertex) {
    std::vector<VertexId> vertices(vertexCount);
    std::uniform_int_distribution<> dis(0, maxVertex - 1);
    std::generate_n(vertices.begin(), vertexCount, [&dis]() {
        return dis(GetRng());
    });
    return vertices;
}
