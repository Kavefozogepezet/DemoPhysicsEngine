//
// Created by User on 14/03/2023.
//

#ifndef HMPE_BODY_HPP
#define HMPE_BODY_HPP

#include <vector>

#include "hmpe/utils.hpp"
#include "hmmath/math.hpp"
#include "hmpe/shape.hpp"

HMPE_START

struct UserData {
    virtual ~UserData() = default;
};

enum class BodyType {
    DYNAMIC,
    KINEMATIC
};

class HMPE_API Body {
public:
    Vec2 position {};
    float rotation = 0.0f;

    Vec2 linearVelocity {};
    float angularVelocity = 0.0f;

    Unique<Shape> shape { nullptr };
    float mass = 1.0f;
    float angularMass = 1.0f;
    float friction = 0.5f;

    BodyType type = BodyType::DYNAMIC;

    [[nodiscard]]
    Vec2 support(const Vec2& normal) const;

    template<class DataType, typename ... Args>
    void createUserData(Args ... args) {
        userData = std::make_unique<DataType>(args...);
    }

    template<class DataType>
    DataType& getUserData() {
        return *userData;
    }
private:
    Unique<UserData> userData { nullptr };
};

HMPE_END
#endif //HMPE_BODY_HPP
