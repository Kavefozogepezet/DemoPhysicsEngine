//
// Created by User on 14/03/2023.
//

#include <iostream>
#include "hmpe/world.hpp"
#include "hmpe/body.hpp"
#include "solver.hpp"

HMPE_START

World::World(const Vec2 &gravity) :
    gravity(gravity)
{}

World::~World() noexcept {
    std::cout << "deleted world" << std::endl;
    for(auto body : bodies)
        delete body;
}

BodyRef World::createBody() {
    auto body = new Body();
    bodies.insert(body);
    return body;
}

void World::destroyBody(BodyRef body) {
    if(bodies.erase(body))
        delete body;
}

void World::update(float delta) {
    applyNaturalForces(delta);
    gatherCollisions();;
    resolveCollisions(delta);
    invokeCallback();
    stepSimulation(delta);
    now++;
}

void World::setCollisionCallback(CallbackFunc callback) const {
    this->callback = callback;
}

void World::applyNaturalForces(float delta) {
    for(auto& body : bodies)
        if(body->type == BodyType::DYNAMIC)
            body->linearVelocity += gravity * delta;
}

void World::gatherCollisions() {
    for(auto bodyIt1 = bodies.begin(); bodyIt1 != bodies.end(); bodyIt1++) {
        auto bodyIt2 = bodyIt1;
        for(++bodyIt2; bodyIt2 != bodies.end(); bodyIt2++) {
            auto info = ContactInfo::of(*bodyIt1, *bodyIt2);
            collide(info);
            if(info.collision)
                getManifold(info.body1, info.body2).addContact(info, now);
        }
    }
    clearOutdatedManifolds();
}

void World::resolveCollisions(float delta) {
    std::list<Solver> solvers;

    for(auto& sub : manifolds) {
        for(auto& man : sub.second) {
            for(auto& info : man.second) {
                if(info.body2->type == BodyType::KINEMATIC) {
                    solvers.push_back(Solver::kinematicContact(info, delta));
                    solvers.push_back(Solver::kinematicFriction(info, &*solvers.rbegin()));
                } else {
                    solvers.push_back(Solver::contact(info, delta));
                    solvers.push_back(Solver::friction(info, &*solvers.rbegin()));
                }
            }
        }
    }

    if(solvers.empty())
        return;

    for(size_t i = 0; i < 10; i++) {
        for (auto &solver: solvers)
            solver.solve();
        for (auto &solver: solvers)
            solver.apply();
    }
}

void World::invokeCallback() {
    //if(callback)
    //    for(auto& info : collisions)
    //        callback(info);
}

void World::stepSimulation(float delta) {
    for(auto body : bodies) {
        if(body->type == BodyType::DYNAMIC) {
            body->position += body->linearVelocity * delta;
            body->rotation += body->angularVelocity * delta;
        }
    }
}

Manifold& World::getManifold(BodyRef b1, BodyRef b2) {
    if(b1 > b2) {
        auto temp = b1;
        b1 = b2;
        b2 = temp;
    }
    return manifolds[b1][b2];
}

void World::clearOutdatedManifolds() {
    for(auto it1 = manifolds.begin(); it1 != manifolds.end();) {
        auto& sub = it1->second;
        for(auto it2 = sub.begin(); it2 != sub.end();) {
            if(it2->second.isOutdated(now))  it2 = sub.erase(it2);
            else it2++;
        }
        if(sub.empty()) it1 = manifolds.erase(it1);
        else it1++;
    }
}

HMPE_END
