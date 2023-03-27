//
// Created by User on 14/03/2023.
//

#include <iostream>
#include "hmpe/world.hpp"
#include "hmpe/body.hpp"
#include "solver.hpp"

HMPE_START

void Manifold::addContact(ContactInfo &info, UpdateID now) {
    last = now;
    for(auto it = contacts.begin(); it != contacts.end();) {
        float l1 = Math::length(it->point1 - info.point1);
        float l2 = Math::length(it->point2 - info.point2);
        bool tooClose = l1 < 0.1f || l2 < 0.1f;

        Vec2 oldPos1 = info.body1->position + it->localPoint1;
        Vec2 oldPos2 = info.body2->position + it->localPoint2;
        bool r1 = !Math::floatEq(Math::length(oldPos1 - it->point1), 0.0f, 50000.0f);
        bool r2 = !Math::floatEq(Math::length(oldPos2 - it->point2), 0.0f, 50000.0f);

        if(tooClose || r1 || r2) it = contacts.erase(it);
        else it++;
    }
    contacts.push_front(info);
    if(contacts.size() > 2)
        contacts.pop_back();
}

bool Manifold::isOutdated(UpdateID now) const {
    return now != last;
}

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
            if(info.collision) {
                info.md.reserve(M_PI / 0.2f);
                for(float i = 0.0f; i < 2 * M_PI; i += 0.1f) {
                    Vec2 n = Math::rotate(Vec2 { 1.0f, 0.0f }, i);
                    info.md.push_back((*bodyIt1)->support(n) - (*bodyIt2)->support(-n));
                }
                getManifold(info.body1, info.body2).addContact(info, now);
            }
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
