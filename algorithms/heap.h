//
// Created by mkspopov on 11.11.2020.
//

#pragma once

#include "../types.h"

#include <queue>
#include <type_traits>

struct HeapElement {
    HeapElement(VertexId vertex, Weight weight)
            : vertex(vertex)
            , weight(weight) {
    }

    bool operator>(const HeapElement& rhs) const {
        if (weight == rhs.weight) {
            return vertex > rhs.vertex;
        }
        return weight > rhs.weight;
    }

    VertexId vertex;
    Weight weight;
};

template <class T>
using Heap = std::priority_queue<T, std::vector<T>, std::greater<>>;
