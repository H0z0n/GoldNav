// ObstacleTracker — следит за entity на карте и сообщает навмешу, где изменилась
// геометрия.
//
// Два вида объектов:
//   - brush-энтити (SOLID_BSP: двери, платформы, ломаемые стёкла, func_wall_toggle).
//     Трассы навмеша видят их сами, в текущем положении; трекеру остаётся заметить,
//     что объект сдвинулся/исчез/появился, дождаться остановки и выдать "грязную"
//     область — узлы вокруг перепроверятся.
//   - entity с коробкой (SOLID_BBOX/SLIDEBOX: ящик, бочка, стоящий NPC). В статичные
//     трассы они не попадают, поэтому постоявшая на месте entity заносится в
//     ObstacleSet (см. navmesh/obstacle_set.h), а когда двинулась или исчезла —
//     убирается оттуда.
// Игроки, монстры и entity, которых GoldNav сейчас ведёт сам, препятствиями не
// считаются: они подвижны, их обходит "ходок" на месте.
#pragma once

#include <functional>
#include <unordered_map>
#include <vector>
#include <extdll.h>

#include "navmesh/obstacle_set.h"

namespace goldnav {

struct DirtyBox
{
    Vec3 mins;
    Vec3 maxs;
};

class ObstacleTracker
{
public:
    using ExcludePredicate = std::function<bool(int entIndex)>;

    void Reset();

    // При старте карты: всё, что уже стоит, сразу считается препятствием (до первичной
    // генерации навмеша), "грязных" областей не выдаёт.
    void InitialScan(ObstacleSet& obstacles, const ExcludePredicate& isExcluded);

    // Периодический обход entity (сам ограничивает частоту). В outDirty добавляются
    // области, где навмеш нужно перепроверить.
    void Update(float now, ObstacleSet& obstacles, const ExcludePredicate& isExcluded,
                std::vector<DirtyBox>& outDirty);

    // Entity перестала быть препятствием прямо сейчас (её повёл GoldNav). true — если
    // она была в наборе; тогда outDirty — её область.
    bool ForceRemove(int entIndex, ObstacleSet& obstacles, DirtyBox& outDirty);

    size_t TrackedCount() const { return m_records.size(); }

private:
    struct Record
    {
        int serial = 0;
        bool isBrush = false;
        Vec3 mins, maxs;             // текущие границы
        Vec3 stableMins, stableMaxs; // границы, учтённые в навмеше
        bool hasStable = false;
        bool registered = false;     // коробка лежит в ObstacleSet
        float lastChange = 0.0f;
        bool seen = false;
    };

    std::unordered_map<int, Record> m_records;
    float m_nextScan = 0.0f;
};

} // namespace goldnav
