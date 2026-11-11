// Синтетический мир из осевых коробок — подмена движка для тестов генератора
// навмеша, планировщика и "ходока". Реализует ту же семантику трассы, на которую
// опирается боевой код: свип коробки-хулла, startSolid при старте внутри геометрии,
// остановка в 1/32 юнита от поверхности (как DIST_EPSILON движка).
#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include "navmesh/collision_world.h"
#include "pathfollower/ground_mover.h"

namespace goldnav_test {

class BoxWorld : public goldnav::ICollisionWorld
{
public:
    struct Box
    {
        goldnav::Vec3 mins;
        goldnav::Vec3 maxs;
        int entity = 0;                 // 0 — статичный мир, >0 — подвижная entity
        goldnav::NavContents contents = goldnav::NavContents::Solid;
    };

    // Статичная геометрия.
    void AddSolid(const goldnav::Vec3& mins, const goldnav::Vec3& maxs)
    {
        Box box;
        box.mins = mins;
        box.maxs = maxs;
        m_boxes.push_back(box);
    }

    // Подвижное препятствие (игрок/NPC): видно только "ходоку", не генератору.
    size_t AddEntity(const goldnav::Vec3& mins, const goldnav::Vec3& maxs, int entityId)
    {
        Box box;
        box.mins = mins;
        box.maxs = maxs;
        box.entity = entityId;
        m_boxes.push_back(box);
        return m_boxes.size() - 1;
    }

    // Объём воды/лавы (не твёрдый).
    void AddVolume(const goldnav::Vec3& mins, const goldnav::Vec3& maxs, goldnav::NavContents contents)
    {
        Box box;
        box.mins = mins;
        box.maxs = maxs;
        box.contents = contents;
        m_volumes.push_back(box);
    }

    Box& BoxAt(size_t index) { return m_boxes[index]; }

    mutable size_t traceCount = 0;

    void TraceHull(const goldnav::Vec3& startFeet, const goldnav::Vec3& endFeet, goldnav::HullIndex hull,
                   goldnav::HullTrace& out) const override
    {
        const goldnav::HullDef& def = goldnav::GetHullDef(hull);
        TraceShape(startFeet, endFeet, def.halfWidth, def.height, false, out);
    }

    goldnav::NavContents PointContents(const goldnav::Vec3& p) const override
    {
        for (const Box& box : m_boxes)
        {
            if (box.entity == 0 && Inside(p, box.mins, box.maxs, 0.0f))
            {
                return goldnav::NavContents::Solid;
            }
        }
        for (const Box& box : m_volumes)
        {
            if (Inside(p, box.mins, box.maxs, 0.0f))
            {
                return box.contents;
            }
        }
        return goldnav::NavContents::Empty;
    }

    // Свип коробки с основанием 2*halfWidth и высотой height, заданной feet-позицией.
    void TraceShape(const goldnav::Vec3& start, const goldnav::Vec3& end, float halfWidth, float height,
                    bool includeEntities, goldnav::HullTrace& out) const
    {
        ++traceCount;
        out = goldnav::HullTrace();
        out.endPos = end;

        const float d[3] = { end.x - start.x, end.y - start.y, end.z - start.z };
        const float p[3] = { start.x, start.y, start.z };

        float bestT = 1.0f;
        int bestAxis = -1;
        float bestSign = 0.0f;
        int bestEntity = -1;

        for (const Box& box : m_boxes)
        {
            if (box.entity != 0 && !includeEntities)
            {
                continue;
            }

            // Расширяем коробку на размер хулла: тогда хулл сводится к точке feet.
            const float lo[3] = { box.mins.x - halfWidth, box.mins.y - halfWidth, box.mins.z - height };
            const float hi[3] = { box.maxs.x + halfWidth, box.maxs.y + halfWidth, box.maxs.z };

            const float tol = 0.001f;
            if (p[0] > lo[0] + tol && p[0] < hi[0] - tol &&
                p[1] > lo[1] + tol && p[1] < hi[1] - tol &&
                p[2] > lo[2] + tol && p[2] < hi[2] - tol)
            {
                out.startSolid = true;
                out.allSolid = true;
                out.fraction = 0.0f;
                out.endPos = start;
                out.entity = box.entity;
                return;
            }

            float tEnter = -1e30f;
            float tExit = 1e30f;
            int enterAxis = -1;
            float enterSign = 0.0f;
            bool miss = false;

            for (int axis = 0; axis < 3; ++axis)
            {
                if (std::fabs(d[axis]) < 1e-9f)
                {
                    if (p[axis] <= lo[axis] + tol || p[axis] >= hi[axis] - tol)
                    {
                        miss = true;
                        break;
                    }
                    continue;
                }

                float t1 = (lo[axis] - p[axis]) / d[axis];
                float t2 = (hi[axis] - p[axis]) / d[axis];
                float sign = -1.0f; // входим через lo-грань: нормаль смотрит в минус
                if (t1 > t2)
                {
                    std::swap(t1, t2);
                    sign = 1.0f;
                }
                if (t1 > tEnter)
                {
                    tEnter = t1;
                    enterAxis = axis;
                    enterSign = sign;
                }
                tExit = std::min(tExit, t2);
            }

            if (miss || enterAxis < 0 || tEnter >= tExit || tEnter < -1e-4f || tEnter >= bestT)
            {
                continue;
            }

            bestT = std::max(tEnter, 0.0f);
            bestAxis = enterAxis;
            bestSign = enterSign;
            bestEntity = box.entity;
        }

        if (bestAxis < 0)
        {
            return;
        }

        // Останавливаемся в 1/32 юнита от поверхности, как движок.
        const float along = std::fabs(d[bestAxis]);
        float fraction = bestT - (along > 1e-9f ? 0.03125f / along : 0.0f);
        fraction = std::max(0.0f, std::min(1.0f, fraction));

        out.fraction = fraction;
        out.endPos = goldnav::Vec3(p[0] + d[0] * fraction, p[1] + d[1] * fraction, p[2] + d[2] * fraction);
        out.normal = goldnav::Vec3(bestAxis == 0 ? bestSign : 0.0f, bestAxis == 1 ? bestSign : 0.0f,
                                   bestAxis == 2 ? bestSign : 0.0f);
        out.entity = bestEntity;
    }

private:
    static bool Inside(const goldnav::Vec3& p, const goldnav::Vec3& mins, const goldnav::Vec3& maxs, float tol)
    {
        return p.x > mins.x + tol && p.x < maxs.x - tol &&
               p.y > mins.y + tol && p.y < maxs.y - tol &&
               p.z > mins.z + tol && p.z < maxs.z - tol;
    }

    std::vector<Box> m_boxes;
    std::vector<Box> m_volumes;
};

// "Ходок" в коробочном мире: коллизионная форма entity + подвижные препятствия.
class BoxMoveTracer : public goldnav::IMoveTracer
{
public:
    BoxMoveTracer(const BoxWorld& world, goldnav::HullIndex hull)
        : m_world(world), m_halfWidth(goldnav::GetHullDef(hull).halfWidth), m_height(goldnav::GetHullDef(hull).height) {}

    void Trace(const goldnav::Vec3& fromFeet, const goldnav::Vec3& toFeet, goldnav::HullTrace& out) override
    {
        m_world.TraceShape(fromFeet, toFeet, m_halfWidth, m_height, true, out);
    }

private:
    const BoxWorld& m_world;
    float m_halfWidth;
    float m_height;
};

} // namespace goldnav_test
