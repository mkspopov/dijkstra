//
// Created by mkspopov on 11.11.2020.
//

#include "serializer.h"

#include <cassert>
#include <limits>

GraphDeserializer::GraphDeserializer(const std::string& filename)
    : ifStream_(std::ifstream(filename)) {
}

WeightGraph<EdgeProperty> GraphDeserializer::DimacsDeserialize() {
    /*
     * Skips the first seven lines. Reads strings `a {from} {to} {distance}`.
     */
    for (int i = 0; i < 7; ++i) {
        ifStream_.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    char aChar = 'a';
    GraphBuilder<WeightGraph<EdgeProperty>, EdgeProperty> builder;
    VertexId verticesCount = 0;

    while (ifStream_ >> aChar) {
        assert(aChar == 'a');
        VertexId from, to;
        Weight distance;
        ifStream_ >> from >> to >> distance;

        while (verticesCount < std::max(from, to)) {
            const auto prev = verticesCount;
            verticesCount = builder.AddVertex() + 1;
            ASSERT(prev + 1 == verticesCount);
        }
        builder.AddEdge(--from, --to, EdgeProperty{distance});
    }

    return builder.Build();
}
