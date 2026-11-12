#include "obstacle_tracker.h"

#include <cmath>
#include <meta_api.h>
#include <util.h> // INDEXENT

namespace goldnav {

namespace {

constexpr float kScanInterval = 0.25f;
constexpr float kBrushSettleTime = 0.3f; // дверь должна постоять столько, чтобы считаться остановившейся
constexpr float kBoxSettleTime = 1.0f;   // entity должна простоять столько, чтобы стать препятствием
constexpr float kMoveEpsilon = 0.5f;
constexpr float kMinBoxWidth = 8.0f;
constexpr float kMinBoxHeight = 4.0f;

enum class Kind { None, Brush, Box };

bool Differs(const Vec3& a, const Vec3& b)
{
    return std::fabs(a.x - b.x) > kMoveEpsilon || std::fabs(a.y - b.y) > kMoveEpsilon ||
           std::fabs(a.z - b.z) > kMoveEpsilon;
}

Kind Classify(const edict_t* ent, Vec3& outMins, Vec3& outMaxs)
{
    if (!ent || ent->free || !ent->pvPrivateData || (ent->v.flags & FL_KILLME))
    {
        return Kind::None;
    }

    const entvars_t& v = ent->v;

    if (v.solid == SOLID_BSP)
    {
        // У brush-энтити origin обычно нулевой, а геометрия задана моделью — берём
        // абсолютные границы, которые движок пересчитывает при каждом сдвиге/повороте.
        outMins = Vec3(v.absmin.x, v.absmin.y, v.absmin.z);
        outMaxs = Vec3(v.absmax.x, v.absmax.y, v.absmax.z);
        if (outMaxs.x - outMins.x < 1.0f || outMaxs.y - outMins.y < 1.0f)
        {
            return Kind::None;
        }
        return Kind::Brush;
    }

    if (v.solid != SOLID_BBOX && v.solid != SOLID_SLIDEBOX)
    {
        return Kind::None;
    }
    if (v.flags & (FL_CLIENT | FL_FAKECLIENT | FL_MONSTER))
    {
        return Kind::None; // ходят сами
    }
    if (v.movetype == MOVETYPE_FOLLOW)
    {
        return Kind::None;
    }

    const float width = v.maxs.x - v.mins.x;
    const float depth = v.maxs.y - v.mins.y;
    const float height = v.maxs.z - v.mins.z;
    if (width < kMinBoxWidth || depth < kMinBoxWidth || height < kMinBoxHeight)
    {
        return Kind::None; // гранаты, гильзы и прочая мелочь
    }

    outMins = Vec3(v.origin.x + v.mins.x, v.origin.y + v.mins.y, v.origin.z + v.mins.z);
    outMaxs = Vec3(v.origin.x + v.maxs.x, v.origin.y + v.maxs.y, v.origin.z + v.maxs.z);
    return Kind::Box;
}

DirtyBox MakeDirty(const Vec3& mins, const Vec3& maxs)
{
    DirtyBox box;
    box.mins = mins;
    box.maxs = maxs;
    return box;
}

DirtyBox Union(const Vec3& aMins, const Vec3& aMaxs, const Vec3& bMins, const Vec3& bMaxs)
{
    DirtyBox box;
    box.mins = Vec3(std::fmin(aMins.x, bMins.x), std::fmin(aMins.y, bMins.y), std::fmin(aMins.z, bMins.z));
    box.maxs = Vec3(std::fmax(aMaxs.x, bMaxs.x), std::fmax(aMaxs.y, bMaxs.y), std::fmax(aMaxs.z, bMaxs.z));
    return box;
}

int FirstNonClient()
{
    return gpGlobals ? gpGlobals->maxClients + 1 : 1;
}

int EntityLimit()
{
    return gpGlobals ? gpGlobals->maxEntities : 0;
}

} // namespace

void ObstacleTracker::Reset()
{
    m_records.clear();
    m_nextScan = 0.0f;
}

void ObstacleTracker::InitialScan(ObstacleSet& obstacles, const ExcludePredicate& isExcluded)
{
    Reset();

    for (int i = FirstNonClient(); i < EntityLimit(); ++i)
    {
        edict_t* ent = INDEXENT(i);
        Vec3 mins, maxs;
        const Kind kind = Classify(ent, mins, maxs);
        if (kind == Kind::None || (isExcluded && isExcluded(i)))
        {
            continue;
        }

        Record record;
        record.serial = ent->serialnumber;
        record.isBrush = (kind == Kind::Brush);
        record.mins = record.stableMins = mins;
        record.maxs = record.stableMaxs = maxs;
        record.hasStable = true;
        if (kind == Kind::Box)
        {
            obstacles.Set(i, mins, maxs);
            record.registered = true;
        }
        m_records[i] = record;
    }
}

void ObstacleTracker::Update(float now, ObstacleSet& obstacles, const ExcludePredicate& isExcluded,
                             std::vector<DirtyBox>& outDirty)
{
    if (now < m_nextScan)
    {
        return;
    }
    m_nextScan = now + kScanInterval;

    for (auto& entry : m_records)
    {
        entry.second.seen = false;
    }

    for (int i = FirstNonClient(); i < EntityLimit(); ++i)
    {
        edict_t* ent = INDEXENT(i);
        Vec3 mins, maxs;
        const Kind kind = Classify(ent, mins, maxs);
        if (kind == Kind::None || (isExcluded && isExcluded(i)))
        {
            continue;
        }

        auto it = m_records.find(i);
        if (it != m_records.end() &&
            (it->second.serial != ent->serialnumber || it->second.isBrush != (kind == Kind::Brush)))
        {
            // В слоте уже другая entity — старую "хороним" (как исчезнувшую), новую заводим.
            Record& old = it->second;
            if (old.registered)
            {
                obstacles.Remove(i);
            }
            if (old.hasStable)
            {
                outDirty.push_back(MakeDirty(old.stableMins, old.stableMaxs));
            }
            m_records.erase(it);
            it = m_records.end();
        }

        if (it == m_records.end())
        {
            Record record;
            record.serial = ent->serialnumber;
            record.isBrush = (kind == Kind::Brush);
            record.mins = mins;
            record.maxs = maxs;
            record.lastChange = now;
            record.seen = true;
            m_records[i] = record;
            continue;
        }

        Record& record = it->second;
        record.seen = true;

        if (Differs(record.mins, mins) || Differs(record.maxs, maxs))
        {
            record.mins = mins;
            record.maxs = maxs;
            record.lastChange = now;

            // Коробка тронулась с места — её прежнее место сразу освобождаем.
            if (record.registered)
            {
                obstacles.Remove(i);
                record.registered = false;
                outDirty.push_back(MakeDirty(record.stableMins, record.stableMaxs));
                record.hasStable = false;
            }
            continue;
        }

        const float settle = record.isBrush ? kBrushSettleTime : kBoxSettleTime;
        if (now - record.lastChange < settle)
        {
            continue;
        }

        const bool moved = !record.hasStable || Differs(record.stableMins, mins) || Differs(record.stableMaxs, maxs);
        if (!moved && (record.isBrush || record.registered))
        {
            continue;
        }

        // Объект остановился в новом положении — учитываем его в навмеше.
        if (record.isBrush)
        {
            outDirty.push_back(record.hasStable ? Union(record.stableMins, record.stableMaxs, mins, maxs)
                                                : MakeDirty(mins, maxs));
        }
        else
        {
            obstacles.Set(i, mins, maxs);
            record.registered = true;
            outDirty.push_back(MakeDirty(mins, maxs));
        }
        record.stableMins = mins;
        record.stableMaxs = maxs;
        record.hasStable = true;
    }

    // Исчезнувшие (удалены, стали нетвёрдыми, их повёл GoldNav).
    for (auto it = m_records.begin(); it != m_records.end(); )
    {
        Record& record = it->second;
        if (record.seen)
        {
            ++it;
            continue;
        }
        if (record.registered)
        {
            obstacles.Remove(it->first);
        }
        if (record.hasStable)
        {
            outDirty.push_back(MakeDirty(record.stableMins, record.stableMaxs));
        }
        it = m_records.erase(it);
    }
}

bool ObstacleTracker::ForceRemove(int entIndex, ObstacleSet& obstacles, DirtyBox& outDirty)
{
    const auto it = m_records.find(entIndex);
    if (it == m_records.end())
    {
        return false;
    }

    const Record record = it->second;
    m_records.erase(it);

    if (!record.registered)
    {
        return false;
    }

    obstacles.Remove(entIndex);
    outDirty = MakeDirty(record.stableMins, record.stableMaxs);
    return true;
}

} // namespace goldnav
