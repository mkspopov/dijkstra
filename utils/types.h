//
// Created by mkspopov on 13.11.2020.
//

#pragma once

#include <cstdint>
#include <limits>

using EdgeId = uint32_t;
using LevelId = uint8_t;
using VertexId = uint32_t;
using Weight = float;

static constexpr int UNDEFINED = std::numeric_limits<int>::max();
