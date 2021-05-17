//
// Created by mkspopov on 11.11.2020.
//

#pragma once

#include "color.h"

#include <queue>
#include <type_traits>

struct HeapElement {
    HeapElement(VertexId vertex, Weight weight)
        : vertex(vertex)
        , weight(weight)
    {}

    bool operator>(const HeapElement& rhs) const {
        if (weight == rhs.weight) {
            return vertex < rhs.vertex;
        }
        return weight > rhs.weight;
    }

    VertexId vertex;
    Weight weight;
};

class StdHeap {
public:
    template <class ...Args>
    void Emplace(Args&&... args) {
        heap_.emplace(std::forward<Args>(args)...);
    }

    void Push(HeapElement t) {
        heap_.emplace(t);
    }

    void Decrease(HeapElement t) {
        heap_.emplace(t);
    }

    bool Empty() const {
        return heap_.empty();
    }

    auto Extract() {
        auto top = heap_.top();
        heap_.pop();
        return top;
    }

private:
    std::priority_queue<HeapElement, std::vector<HeapElement>, std::greater<>> heap_;
};
