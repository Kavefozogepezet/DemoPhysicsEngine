//
// Created by User on 18/03/2023.
//

#include <iostream>
#include <utility>

#include "solver.hpp"
#include "hmpe/body.hpp"
#include "hmmath/math.hpp"

HMPE_START

static const float BETA = 0.1f;

static float getContactBias(ContactInfo& info, float delta) {
    return -(BETA / delta) * info.depth;
}

CVec::CVec() : values {} {}

CVec::CVec(float f1, float f2, float f3, float f4, float f5, float f6) :
    values{ f1, f2, f3, f4, f5, f6 }
{}

CVec::CVec(const Vec2& v1, float w1, const Vec2& v2, float w2) :
    values { v1.x, v1.y, w1, v2.x, v2.y, w2 }
{}

Vec2 CVec::getV1() { return { values[0], values[1] }; }
float CVec::getF1() { return values[2]; }
Vec2 CVec::getV2() { return { values[3], values[4] }; }
float CVec::getF2() { return values[5]; }

float& CVec::operator[](size_t idx) { return values[idx]; }
const float& CVec::operator[](size_t idx) const { return values[idx]; }

float dot(const CVec& c1, const CVec& c2) {
    float product = 0.0f;
    for(size_t i = 0; i < 6; i++)
        product += c1[i] * c2[i];
    return product;
}

CVec compwisemul(const CVec& c1, const CVec& c2) {
    return {
            c1[0] * c2[0], c1[1] * c2[1], c1[2] * c2[2],
            c1[3] * c2[3], c1[4] * c2[4], c1[5] * c2[5]
    };
}

CVec operator*(const CVec& v, float f) {
    CVec product {};
    for(size_t i = 0; i < 6; i++)
        product[i] = v[i] * f;
    return product;
}

CVec operator*(float f, const CVec& v) {
    return v * f;
}

Solver Solver::contact(ContactInfo &info, float delta) {
    Vec2 n = info.normal;
    Vec2 r1 = info.localPoint1;
    Vec2 r2 = info.localPoint2;

    CVec j = {
            -n, -(r1.x * n.y - r1.y * n.x),
            n, r2.x * n.y - r2.y * n.x
    };
    float bias = getContactBias(info, delta);
    LambdaFunc func = [](float l) { return std::max(l, 0.0f); };
    return { info, j, bias, func };
}

Solver Solver::kinematicContact(ContactInfo &info, float delta) {
    Vec2 n = info.normal;
    Vec2 r1 = info.localPoint1;
    CVec j = { -n, -(r1.x * n.y - r1.y * n.x) };
    float bias = getContactBias(info, delta);
    LambdaFunc func = [](float l) { return std::max(l, 0.0f); };
    return { info, j, bias, func };
}

Solver Solver::friction(ContactInfo &info, Solver *contactSolver) {
    Vec2 r1 = info.localPoint1;
    Vec2 r2 = info.localPoint2;

    float coefficient = std::max(0.0f, (info.body1->friction + info.body2->friction)) * 0.5f;

    Vec2 t = { info.normal.y, -info.normal.x };
    CVec j = {
            -t, -(r1.x * t.y - r1.y * t.x),
            t, r2.x * t.y - r2.y * t.x
    };
    LambdaFunc func = [=](float l) {
        float bound = contactSolver->getSolution().lambda * coefficient;
        return Math::clamp(l, -bound, bound);
    };
    Solver s { info, j, 0.0f, func };
    s.type = 1;
    return s;
}

Solver Solver::kinematicFriction(ContactInfo &info, Solver *contactSolver) {
    Vec2 r1 = info.localPoint1;
    float coefficient = std::max(0.0f, (info.body1->friction + info.body2->friction)) * 0.5f;

    Vec2 t = { info.normal.y, -info.normal.x };
    CVec j = { -t, -(r1.x * t.y - r1.y * t.x) };
    LambdaFunc func = [=](float l) {
        float bound = contactSolver->getSolution().lambda * coefficient;
        return Math::clamp(l, -bound, bound);
    };
    return { info, j, 0.0f, func };
}

Solver::Solver(ContactInfo &info, CVec jacobian, float bias, LambdaFunc func) :
    c(info),
    j(jacobian),
    b(bias),
    lf(std::move(func)),
    solution {}
{
    CVec m_inv = {
            1.0f / c.body1->mass, 1.0f / c.body1->mass, 1.0f / c.body1->angularMass,
            1.0f / c.body2->mass, 1.0f / c.body2->mass, 1.0f / c.body2->angularMass
    };
    m_eff = 1.0f / dot(compwisemul(m_inv, j), j);
}

void Solver::solve() {
    CVec v = {
            c.body1->linearVelocity, c.body1->angularVelocity,
            c.body2->linearVelocity, c.body2->angularVelocity
    };
    float jv = dot(j, v);
    float lambda = lf(m_eff * (-(jv + b)));
    CVec dI = j * lambda;

    solution = {
            lambda,
            dI.getV1() / c.body1->mass,
            dI.getF1() / c.body1->angularMass,
            dI.getV2() / c.body2->mass,
            dI.getF2() / c.body2->angularMass
    };
}

void Solver::apply() const {
    c.body1->linearVelocity += solution.deltaV1;
    c.body1->angularVelocity += solution.deltaW1;
    c.body2->linearVelocity += solution.deltaV2;
    c.body2->angularVelocity += solution.deltaW2;
}

const Solution& Solver::getSolution() const {
    return solution;
}

HMPE_END
