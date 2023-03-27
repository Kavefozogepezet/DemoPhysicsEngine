//
// Created by User on 12/03/2023.
//

#ifndef HMMATH_HPP
#define HMMATH_HPP

#include <cmath>
#include <limits>
#include <glm/glm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/matrix_transform.hpp>

#ifdef HMMATH_NO_GLOBAL_TYPES
namespace Math {
#endif
template<int length, typename Type = float, glm::precision p = glm::defaultp>
using Vec = glm::vec<length, Type, p>;

using Vec2 = Vec<2, float>;
using Vec3 = Vec<3, float>;
using Vec4 = Vec<4, float>;

using Vec2i = glm::ivec2;
using Vec3i = glm::ivec3;
using Vec4i = glm::ivec4;

template<int col, int row, typename Type = float, glm::precision p = glm::defaultp>
using Mat = glm::mat<col, row, Type, p>;

using Mat2 = glm::mat2;
using Mat3 = glm::mat3;
using Mat4 = glm::mat4;
#ifdef HMMATH_NO_GLOBAL_TYPES
}
#endif

namespace Math {
    using namespace glm;

    inline bool floatEq(float f1, float f2, float bias = 10.0f) {
        constexpr float epsilon = std::numeric_limits<float>::epsilon();
        float corrected = epsilon * bias * std::max(1.0f, std::fabs(f1) + std::fabs(f2));
        return std::fabs(f1 - f2) < corrected;
    }

    template<typename Type, typename Base>
    Type lerp(const Type& t1, const Type& t2, Base ratio) {
        constexpr Base one = static_cast<Base>(1);
        return t1 * (one - ratio) + t2 * ratio;
    }

    template<typename Type = float, glm::precision p = glm::defaultp>
    [[nodiscard]]
    Vec<3, Type, p> compwisemul(const Vec<3, Type, p>& v1, const Vec<3, Type, p>& v2) {
        return {
            v1.x * v2.x,
            v1.y * v2.y,
            v1.z * v2.z
        };
    }

    template<typename Type>
    Type clamp(const Type& value, const Type& min, const Type& max) {
        return std::max(std::min(value, max), min);
    }

    inline bool isOpposing(const Vec2& v1, const Vec2& v2) {
        return Math::dot(v1, v2) < 0.0f;
    }

    inline Vec2 normalToOrigin(const Vec2& v1, const Vec2& v2) {
        Vec2 t = v1 - v2;
        Vec2 n = Math::normalize(Vec2 { t.y, -t.x });
        float indicator = Math::dot(n, v1);
        return indicator > 0.0f ? -n : n;
    }

    inline bool isPenetrating(const Vec2& p1, const Vec2& p2, const Vec2& n) {
        return Math::dot(p1, n) * Math::dot(p2, n) < 0.0f;
    }
};

#endif //HMMATH_HPP
