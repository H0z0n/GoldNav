#include "obstacle_set.h"

#include <algorithm>
#include <cmath>

namespace goldnav {

namespace {
constexpr float kInsideTolerance = 0.001f;
constexpr float kSurfaceGap = 0.03125f; // как DIST_EPSILON движка: останавливаемся чуть не доходя
} // namespace

void ObstacleSet::Set(int id, const Vec3& mins, const Vec3& maxs)
{
    for (ObstacleBox& box : m_boxes)
    {
        if (box.id == id)
        {
            box.mins = mins;
            box.maxs = maxs;
            return;
        }
    }

    ObstacleBox box;
    box.id = id;
    box.mins = mins;
    box.maxs = maxs;
    m_boxes.push_back(box);
}

bool ObstacleSet::Remove(int id, ObstacleBox* outBox)
{
    for (size_t i = 0; i < m_boxes.size(); ++i)
    {
        if (m_boxes[i].id == id)
        {
            if (outBox)
            {
                *outBox = m_boxes[i];
            }
            m_boxes[i] = m_boxes.back();
            m_boxes.pop_back();
            return true;
        }
    }
    return false;
}

bool ObstacleSet::Contains(int id) const
{
    for (const ObstacleBox& box : m_boxes)
    {
        if (box.id == id)
        {
            return true;
        }
    }
    return false;
}

bool ObstacleSet::ContainsPoint(const Vec3& p) const
{
    for (const ObstacleBox& box : m_boxes)
    {
        if (p.x > box.mins.x && p.x < box.maxs.x &&
            p.y > box.mins.y && p.y < box.maxs.y &&
            p.z > box.mins.z && p.z < box.maxs.z)
        {
            return true;
        }
    }
    return false;
}

void ObstacleSet::Clip(const Vec3& startFeet, const Vec3& endFeet, float halfWidth, float height, HullTrace& trace) const
{
    if (trace.startSolid || trace.allSolid)
    {
        return; // хулл и так в геометрии карты
    }

    const float p[3] = { startFeet.x, startFeet.y, startFeet.z };
    const float d[3] = { endFeet.x - startFeet.x, endFeet.y - startFeet.y, endFeet.z - startFeet.z };

    float bestT = trace.fraction;
    int bestAxis = -1;
    float bestSign = 0.0f;
    int bestId = -1;

    for (const ObstacleBox& box : m_boxes)
    {
        // Расширяем коробку на размер хулла — тогда хулл сводится к точке feet.
        const float lo[3] = { box.mins.x - halfWidth, box.mins.y - halfWidth, box.mins.z - height };
        const float hi[3] = { box.maxs.x + halfWidth, box.maxs.y + halfWidth, box.maxs.z };

        if (p[0] > lo[0] + kInsideTolerance && p[0] < hi[0] - kInsideTolerance &&
            p[1] > lo[1] + kInsideTolerance && p[1] < hi[1] - kInsideTolerance &&
            p[2] > lo[2] + kInsideTolerance && p[2] < hi[2] - kInsideTolerance)
        {
            trace.startSolid = true;
            trace.allSolid = true;
            trace.fraction = 0.0f;
            trace.endPos = startFeet;
            trace.entity = box.id;
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
                if (p[axis] <= lo[axis] + kInsideTolerance || p[axis] >= hi[axis] - kInsideTolerance)
                {
                    miss = true;
                    break;
                }
                continue;
            }

            float t1 = (lo[axis] - p[axis]) / d[axis];
            float t2 = (hi[axis] - p[axis]) / d[axis];
            float sign = -1.0f;
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
        bestId = box.id;
    }

    if (bestAxis < 0)
    {
        return;
    }

    const float along = std::fabs(d[bestAxis]);
    float fraction = bestT - (along > 1e-9f ? kSurfaceGap / along : 0.0f);
    fraction = std::max(0.0f, std::min(1.0f, fraction));

    trace.fraction = fraction;
    trace.endPos = Vec3(p[0] + d[0] * fraction, p[1] + d[1] * fraction, p[2] + d[2] * fraction);
    trace.normal = Vec3(bestAxis == 0 ? bestSign : 0.0f, bestAxis == 1 ? bestSign : 0.0f, bestAxis == 2 ? bestSign : 0.0f);
    trace.entity = bestId;
}

} // namespace goldnav
