#pragma once

#include "types.h"

class DijkstraVisitor {
public:
    virtual ~DijkstraVisitor() = default;

    virtual void DiscoverVertex(VertexId) {}
    virtual void ExamineVertex(VertexId) {}
    virtual void FinishVertex(VertexId) {}

    virtual void ExamineEdge(EdgeId) {}
    virtual void EdgeRelaxed(EdgeId) {}
    virtual void EdgeNotRelaxed(EdgeId) {}
};

DijkstraVisitor& GetTrivialVisitor();
