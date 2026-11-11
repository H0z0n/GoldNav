// Минимальный engine-agnostic векторный тип для goldnav_core (pathfinder/navmesh/pathfollower).
// Намеренно НЕ включает HLSDK Vector — goldnav_core не должен знать о движке.
// Layout (три последовательных float, без паддинга) совместим с HLSDK Vector, поэтому
// на границе (engine_collision_world.cpp, goldnav_module.cpp), которая уже включает HLSDK,
// конверсия — это копирование трёх float, а не пересчёт.
#pragma once

#include <cmath>

namespace goldnav {

struct Vec3
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Vec3() = default;
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
    Vec3 operator-(const Vec3& o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
    Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }

    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }

    float LengthSq() const { return x * x + y * y + z * z; }
    float Length() const { return std::sqrt(LengthSq()); }

    float Length2DSq() const { return x * x + y * y; }
    float Length2D() const { return std::sqrt(Length2DSq()); }

    Vec3 Normalized() const
    {
        const float len = Length();
        if (len < 1e-6f)
        {
            return Vec3(0.0f, 0.0f, 0.0f);
        }
        return Vec3(x / len, y / len, z / len);
    }
};

inline float Dot(const Vec3& a, const Vec3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3 Cross(const Vec3& a, const Vec3& b)
{
    return Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

inline float Distance(const Vec3& a, const Vec3& b)
{
    return (a - b).Length();
}

inline float Distance2D(const Vec3& a, const Vec3& b)
{
    return (a - b).Length2D();
}

} // namespace goldnav
