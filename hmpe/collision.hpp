//
// Created by User on 15/03/2023.
//

#ifndef HMPE_COLLISION_HPP
#define HMPE_COLLISION_HPP

#include <vector>
#include <list>

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

    static ContactInfo of(BodyRef b1, BodyRef b2);
};

class Manifold {
public:
    Manifold() = default;

    void addContact(ContactInfo& info, UpdateID now);
    [[nodiscard]]
    bool isOutdated(UpdateID now) const;

    [[nodiscard]]
    auto begin() { return contacts.begin(); };
    [[nodiscard]]
    auto end() { return contacts.end(); }

    [[nodiscard]]
    auto begin() const { return contacts.begin(); };
    [[nodiscard]]
    auto end() const { return contacts.end(); }
private:
    std::list<ContactInfo> contacts {};
    UpdateID last = 0;
};

void HMPE_API collide(ContactInfo& info);

bool HMPE_API overlap(BodyRef body1, BodyRef body2);

HMPE_END
#endif //HMPE_COLLISION_HPP
