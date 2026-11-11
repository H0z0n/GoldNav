// ICollisionWorld — единственное, что навигация знает о геометрии карты. Боевая
// реализация (metamod/engine_collision_world.cpp) вызывает TRACE_HULL/POINT_CONTENTS
// движка; тесты подставляют синтетический мир из коробок (tests/fixtures/box_world.h).
// Благодаря этому генератор навмеша, планировщик пути и "ходок" собираются и
// тестируются без SDK.
//
// Все позиции хулла — "feet": центр основания хулла (XY по центру, Z по низу).
// Для point-хулла feet — сама точка. Движковый TRACE_HULL ждёт ЦЕНТР хулла; пересчёт
// (+половина высоты) делает реализация, а не вызывающий — так нельзя ошибиться и
// утопить хулл наполовину в пол.
#pragma once

#include "core/gn_math.h"
#include "nav_types.h"

namespace goldnav {

struct HullTrace
{
    bool startSolid = false; // хулл уже пересекал геометрию в начальной точке
    bool allSolid = false;   // хулл так и не вышел из геометрии
    float fraction = 1.0f;   // 1.0 — путь свободен целиком
    Vec3 endPos;             // feet в конце трассы
    Vec3 normal;             // нормаль поверхности, в которую упёрлись (если fraction < 1)
    int entity = -1;         // индекс entity, в которую упёрлись (0 — мир), -1 — ни во что

    bool Clear() const { return !startSolid && !allSolid && fraction >= 1.0f; }
    bool Hit() const { return !startSolid && !allSolid && fraction < 1.0f; }
};

enum class NavContents
{
    Empty,
    Solid,
    Water,
    Hazard, // лава/слизь — туда не ходим
};

class ICollisionWorld
{
public:
    virtual ~ICollisionWorld() = default;

    // Свип хулла между двумя feet-позициями по статичной геометрии карты (мир +
    // brush-энтити). Подвижные entity (игроки, NPC) не учитываются.
    virtual void TraceHull(const Vec3& startFeet, const Vec3& endFeet, HullIndex hull, HullTrace& out) const = 0;

    virtual NavContents PointContents(const Vec3& point) const = 0;
};

} // namespace goldnav
