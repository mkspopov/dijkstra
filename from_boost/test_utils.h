#pragma once

#include "serializer.h"
#include "utils.h"

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <filesystem>
#include <iostream>

using namespace boost;

static const auto NEW_YORK_GRAPH = "graphs/NY/USA-road-d.NY.gr";

const std::vector<float>& CalcDistancesBoost();

std::filesystem::path FindFile(std::filesystem::path path);

std::vector<VertexId> GenerateRandomVertices(int vertexCount, int maxVertex);

const CoordGraph& TestCoordGraph();

const WGraph& TestGraph();

template<class... Types>
struct TypeArray {
    using as_tuple = std::tuple<Types...>;

    template<std::size_t I>
    using get = std::tuple_element_t<I, as_tuple>;

    static constexpr std::size_t size = sizeof...(Types);
};


template <auto Start, auto End, class F>
constexpr void constexpr_for(F&& f)
{
    if constexpr (Start < End) {
        f(std::integral_constant<decltype(Start), Start>());
        constexpr_for<Start + 1, End>(f);
    }
}
