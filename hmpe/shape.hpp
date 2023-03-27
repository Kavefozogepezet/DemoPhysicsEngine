//
// Created by User on 15/03/2023.
//

#ifndef HMPE_SHAPE_HPP
#define HMPE_SHAPE_HPP

#include <vector>

#include "hmpe/utils.hpp"
#include "hmmath/math.hpp"

HMPE_START

struct ShapeVisitor;
struct Polygon;

struct Shape {
    virtual ~Shape() = default;

    virtual Vec2 support(Vec2 normal) = 0;
    virtual void visit(ShapeVisitor&) = 0;
};

struct HMPE_API ShapeVisitor {
    Shape& shape;

    explicit ShapeVisitor(Shape &shape) : shape(shape) {}
    virtual ~ShapeVisitor() = default;

    void visit();
    virtual void accept(Polygon &shape) = 0;
};

struct HMPE_API Polygon : public Shape {
    std::vector<Vec2> points;

    Vec2 support(Vec2 normal) override;
    void visit(ShapeVisitor& visitor) override;
};

HMPE_END
#endif //HMPE_SHAPE_HPP
