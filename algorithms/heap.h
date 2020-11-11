//
// Created by mkspopov on 11.11.2020.
//

#ifndef DIJKSTRA_HEAP_H
#define DIJKSTRA_HEAP_H

#include <queue>
#include <type_traits>

template <class T>
using Heap = std::priority_queue<T, std::vector<T>, std::greater<>>;

#endif //DIJKSTRA_HEAP_H
