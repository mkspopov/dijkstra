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

static constexpr uint32_t UNDEFINED = std::numeric_limits<uint32_t>::max();
