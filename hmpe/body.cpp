//
// Created by User on 14/03/2023.
//

#include "hmpe/body.hpp"
#include "hmpe/shape.hpp"
#include "hmmath/math.hpp"

HMPE_START

Vec2 Body::support(const Vec2 &normal) const {
    Vec2 rotatedNormal = Math::rotate(normal, -rotation);
    return Vec2(position) + glm::rotate(shape->support(rotatedNormal), rotation);
}

HMPE_END
