//
// Created by User on 15/03/2023.
//

#ifndef HMPE_COLLISION_HPP
#define HMPE_COLLISION_HPP

#include <vector>

#include "hmpe/utils.hpp"
#include "hmmath/math.hpp"

HMPE_START

class Body;

struct HMPE_API ContactInfo {
    BodyRef body1;
    Vec2 point1 = {};
    Vec2 localPoint1 = {};

    BodyRef body2;
    Vec2 point2 = {};
    Vec2 localPoint2 = {};

    bool collision = false;
    Vec2 normal = {};
    float depth = 0.0f;

    Vec2 dualPoint1 {};
    Vec2 dualPoint2 {};

    std::vector<Vec2> md {};
    std::vector<Vec2> closest {};

    static ContactInfo of(BodyRef b1, BodyRef b2);
};

void HMPE_API collide(ContactInfo& info);

bool HMPE_API overlap(BodyRef body1, BodyRef body2);

HMPE_END
#endif //HMPE_COLLISION_HPP
