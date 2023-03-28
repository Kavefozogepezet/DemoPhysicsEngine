//
// Created by User on 14/03/2023.
//

#ifndef HMPE_WORLD_HPP
#define HMPE_WORLD_HPP

#include <unordered_set>
#include <list>
#include <functional>

#include "hmpe/utils.hpp"
#include "hmmath/math.hpp"
#include "hmpe/collision.hpp"

HMPE_START

class HMPE_API World {
public:
    using CallbackFunc = std::function<void(ContactInfo&)>;
    using ManifoldStorage = std::unordered_map<BodyRef, std::unordered_map<BodyRef, Manifold>>;
public:
    Vec2 gravity;

    explicit World(const Vec2& gravity = { 0.0f, -9.81f });
    World(const World&) = delete;
    ~World() noexcept;

    BodyRef createBody();
    void destroyBody(BodyRef body);

    void update(float delta);

    inline auto begin() { return bodies.begin(); }
    inline auto end() { return bodies.end(); }

    void setCollisionCallback(CallbackFunc callback) const;

    const ManifoldStorage& getManifoldStorage() const { return manifolds; }
private:
    UpdateID now = 0;
    std::unordered_set<BodyRef> bodies {};
    ManifoldStorage manifolds {};

    mutable CallbackFunc callback = nullptr;

    void applyNaturalForces(float delta);
    void gatherCollisions();
    void resolveCollisions(float delta);
    void invokeCallback();
    void stepSimulation(float delta);

    Manifold& getManifold(BodyRef b1, BodyRef b2);
    void clearOutdatedManifolds();
};

HMPE_END
#endif //HMPE_WORLD_HPP
