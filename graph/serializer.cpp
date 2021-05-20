//
// Created by mkspopov on 11.11.2020.
//

#include "serializer.h"

#include <fstream>
#include <limits>

GraphDeserializer::GraphDeserializer(std::filesystem::path path) : path_(std::move(path)) {
    // USA-road-TYPE1.NAME.TYPE2
}

CoordGraph GraphDeserializer::ReadCoordinates() {
    auto filename = path_.filename().string();
    filename[9] = 'd';
    path_.replace_filename(filename);
    path_.replace_extension("co");
    std::ifstream file(path_);
    ASSERT(file.is_open());

    for (int i = 0; i < 4; ++i) {
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    std::string type1, type2, type3;
    VertexId totalVertices;
    char pChar;
    file >> pChar >> type1 >> type2 >> type3 >> totalVertices;
    ASSERT_EQUAL(pChar, 'p');
    ASSERT_EQUAL(type1, "aux");
    ASSERT_EQUAL(type2, "sp");
    ASSERT_EQUAL(type3, "co");
    for (int i = 0; i < 3; ++i) {
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }

    char aChar = 'v';
    CoordGraph graph;

    while (file >> aChar) {
        ASSERT(aChar == 'v');
        VertexId vertexId;
        Point coords;
        file >> vertexId >> coords.x >> coords.y;

        graph.AddVertex(coords);
    }

    ASSERT_EQUAL(totalVertices, graph.VerticesCount());

    return graph;
}

WGraph GraphDeserializer::ReadDistances() {
    auto filename = path_.filename().string();
    filename[9] = 'd';
    path_.replace_filename(filename);
    return ReadGraph();
}

WGraph GraphDeserializer::ReadTravelTimes() {
    auto filename = path_.filename().string();
    filename[9] = 't';
    path_.replace_filename(filename);
    return ReadGraph();
}

WGraph GraphDeserializer::ReadGraph() {
    path_.replace_extension("gr");
    std::ifstream file(path_);
    ASSERT(file.is_open());
    for (int i = 0; i < 4; ++i) {
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    std::string type;
    VertexId totalVertices;
    EdgeId totalEdges;
    char pChar;
    file >> pChar >> type >> totalVertices >> totalEdges;
    ASSERT_EQUAL(pChar, 'p');
    ASSERT_EQUAL(type, "sp");
    for (int i = 0; i < 3; ++i) {
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    char aChar = 'a';
    GraphBuilder<WGraph, EdgeProperty> builder;
    VertexId verticesCount = 0;

    while (file >> aChar) {
        ASSERT(aChar == 'a');
        VertexId from, to;
        Weight distance;
        file >> from >> to >> distance;

        while (verticesCount < std::max(from, to)) {
            const auto prev = verticesCount;
            verticesCount = builder.AddVertex() + 1;
            ASSERT(prev + 1 == verticesCount);
        }
        builder.AddEdge(--from, --to, EdgeProperty{distance});
    }

    ASSERT_EQUAL(totalEdges, builder.graph_.EdgesCount());
    ASSERT_EQUAL(totalVertices, builder.graph_.VerticesCount());

    return builder.Build();
}
