//
// Created by mkspopov on 25.01.2021.
//

#include "visitor.h"

DijkstraVisitor& GetTrivialVisitor() {
    static DijkstraVisitor visitor;
    return visitor;
}
