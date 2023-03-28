//
// Created by User on 17/03/2023.
//

#ifndef HMPE_SOLVER_HPP
#define HMPE_SOLVER_HPP

#include "hmpe/utils.hpp"
#include "hmpe/collision.hpp"
#include <functional>

HMPE_START

struct Solution {
    float lambda;

    Vec2 deltaV1;
    float deltaW1;

    Vec2 deltaV2;
    float deltaW2;
};

class HMPE_API CVec {
public:
    CVec();
    CVec(float f1, float f2, float f3, float f4 = 0.0f, float f5 = 0.0f, float f6 = 0.0f);
    CVec(const Vec2& v1, float f1, const Vec2& v2 = {}, float f2 = 0.0f);

    Vec2 getV1();
    float getF1();
    Vec2 getV2();
    float getF2();

    float& operator[](size_t idx);
    const float& operator[](size_t idx) const;
private:
    float values[6];
};

float HMPE_API dot(const CVec& c1, const CVec& c2);
CVec HMPE_API compwisemul(const CVec& c1, const CVec& c2);
CVec HMPE_API operator*(const CVec& v, float f);
CVec HMPE_API operator*(float f, const CVec& v);

class HMPE_API Solver {
public:
    using LambdaFunc = std::function<float(float)>;

    static Solver kinematicContact(ContactInfo& info, float delta);
    static Solver contact(ContactInfo& info, float delta);
    static Solver kinematicFriction(ContactInfo& info, Solver* contactSolver);
    static Solver friction(ContactInfo& info, Solver* contactSolver);

    void solve();
    void apply() const;

    [[nodiscard]]
    const Solution& getSolution() const;
private:
    Solver(ContactInfo& info, CVec jacobian, float bias, LambdaFunc  func);

    ContactInfo& c;
    CVec j;
    CVec m;
    float m_eff;
    float b;
    LambdaFunc lf;
    Solution solution;

    int type = 0;
};

HMPE_END
#endif //HMPE_SOLVER_HPP
