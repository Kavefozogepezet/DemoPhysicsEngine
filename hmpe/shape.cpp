//
// Created by User on 15/03/2023.
//

#include "shape.hpp"

HMPE_START

void ShapeVisitor::visit() {
    shape.visit(*this);
}

Vec2 Polygon::support(Vec2 normal) {
    float max = Math::dot(points[0], normal);
    Vec2 point = points[0];

    for (auto& p : points) {
        float current = Math::dot(p, normal);
        if (current > max) {
            max = current;
            point = p;
        }
    }
    return point;
}

void Polygon::visit(ShapeVisitor & visitor) {
    visitor.accept(*this);
}

HMPE_END
