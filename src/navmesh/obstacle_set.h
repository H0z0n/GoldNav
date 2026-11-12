// Динамические препятствия навмеша: коробки стоящих на карте entity (ящик, бочка,
// отдыхающий NPC). Статичную геометрию карты и brush-энтити (двери, стены) видит
// движковый трассировщик; а вот entity с SOLID_BBOX в его "статичные" трассы не
// попадают — поэтому их коробки хранятся здесь и добавляются к любой трассе навмеша
// аналитически (свип коробки хулла против коробок препятствий).
//
// Engine-agnostic: кто и когда считается препятствием, решает metamod/obstacle_tracker.
#pragma once

#include <vector>
#include "collision_world.h"

namespace goldnav {

struct ObstacleBox
{
    int id = 0; // индекс entity
    Vec3 mins;
    Vec3 maxs;
};

class ObstacleSet
{
public:
    void Clear() { m_boxes.clear(); }
    size_t Count() const { return m_boxes.size(); }
    const std::vector<ObstacleBox>& Boxes() const { return m_boxes; }

    // Добавляет или обновляет коробку entity.
    void Set(int id, const Vec3& mins, const Vec3& maxs);

    // Убирает коробку entity; true — если она была (тогда в outBox её последние границы).
    bool Remove(int id, ObstacleBox* outBox = nullptr);

    bool Contains(int id) const;

    // Уточняет уже посчитанную трассу хулла (halfWidth x height, feet-позиции): если
    // по дороге стоит препятствие ближе, чем найденное столкновение, трасса обрезается.
    void Clip(const Vec3& startFeet, const Vec3& endFeet, float halfWidth, float height, HullTrace& trace) const;

    bool ContainsPoint(const Vec3& point) const;

private:
    std::vector<ObstacleBox> m_boxes;
};

// ICollisionWorld = базовый мир (карта) + динамические препятствия.
class ObstacleWorld : public ICollisionWorld
{
public:
    ObstacleWorld(const ICollisionWorld& base, const ObstacleSet& obstacles) : m_base(base), m_obstacles(obstacles) {}

    void TraceHull(const Vec3& startFeet, const Vec3& endFeet, HullIndex hull, HullTrace& out) const override
    {
        m_base.TraceHull(startFeet, endFeet, hull, out);
        if (m_obstacles.Count() != 0)
        {
            const HullDef& def = GetHullDef(hull);
            m_obstacles.Clip(startFeet, endFeet, def.halfWidth, def.height, out);
        }
    }

    NavContents PointContents(const Vec3& point) const override
    {
        if (m_obstacles.Count() != 0 && m_obstacles.ContainsPoint(point))
        {
            return NavContents::Solid;
        }
        return m_base.PointContents(point);
    }

private:
    const ICollisionWorld& m_base;
    const ObstacleSet& m_obstacles;
};

} // namespace goldnav
