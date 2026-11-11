#include "engine_collision_world.h"

#include <cstring>
#include <meta_api.h>
#include <util.h> // INDEXENT/ENTINDEX/STRING

namespace goldnav {

namespace {

// dlls/extdef.h: point_hull=0, human_hull=1, large_hull=2, head_hull=3. Размеры —
// gHullMins/gHullMaxs движка: 32x32x72, 64x64x64, 32x32x36. TRACE_HULL принимает
// ЦЕНТР хулла, поэтому feet-позицию поднимаем на половину высоты.
int EngineHullFor(HullIndex hull)
{
    switch (hull)
    {
        case HullIndex::Point:    return point_hull;
        case HullIndex::Ducking:  return head_hull;
        case HullIndex::Standing: return human_hull;
        case HullIndex::Large:    return large_hull;
        default:                  return human_hull;
    }
}

int EntityIndexOf(edict_t* ent)
{
    return ent ? ENTINDEX(ent) : 0;
}

bool IsWorld(edict_t* ent)
{
    return ent == nullptr || ENTINDEX(ent) == 0;
}

} // namespace

void EngineCollisionWorld::TraceHull(const Vec3& startFeet, const Vec3& endFeet, HullIndex hull, HullTrace& out) const
{
    const float half = GetHullDef(hull).height * 0.5f;
    const Vector start(startFeet.x, startFeet.y, startFeet.z + half);
    const Vector end(endFeet.x, endFeet.y, endFeet.z + half);
    const int engineHull = EngineHullFor(hull);

    TraceResult tr{};
    edict_t* skip = nullptr;

    // ignore_monsters отсекает игроков и монстров, но НЕ entity с MOVETYPE_PUSHSTEP
    // (толкаемые ящики и NPC, сделанные плагинами, — в том числе те, кого ведём мы
    // сами). Навмеш описывает статичную карту, поэтому такую entity пропускаем и
    // трассируем заново; brush-энтити (SOLID_BSP: стены, двери, стёкла) остаются.
    for (int attempt = 0; attempt < 4; ++attempt)
    {
        g_engfuncs.pfnTraceHull(start, end, ignore_monsters, engineHull, skip, &tr);

        edict_t* hit = tr.pHit;
        const bool touched = tr.fStartSolid || tr.fAllSolid || tr.flFraction < 1.0f;
        if (touched && !IsWorld(hit) && hit->v.solid != SOLID_BSP)
        {
            skip = hit;
            continue;
        }
        break;
    }

    out.startSolid = tr.fStartSolid != 0;
    out.allSolid = tr.fAllSolid != 0;
    out.fraction = tr.flFraction;
    out.endPos = Vec3(tr.vecEndPos.x, tr.vecEndPos.y, tr.vecEndPos.z - half);
    out.normal = Vec3(tr.vecPlaneNormal.x, tr.vecPlaneNormal.y, tr.vecPlaneNormal.z);
    out.entity = (out.startSolid || out.fraction < 1.0f) ? EntityIndexOf(tr.pHit) : -1;
}

NavContents EngineCollisionWorld::PointContents(const Vec3& point) const
{
    const Vector p(point.x, point.y, point.z);
    const int contents = g_engfuncs.pfnPointContents(p);

    switch (contents)
    {
        case CONTENTS_SOLID:
        case CONTENTS_SKY:
            return NavContents::Solid;
        case CONTENTS_SLIME:
        case CONTENTS_LAVA:
            return NavContents::Hazard;
        case CONTENTS_WATER:
            return NavContents::Water;
        default:
            break;
    }

    // Течения (CONTENTS_CURRENT_0..CONTENTS_CURRENT_DOWN = -9..-14) — тоже вода.
    // Числами: в урезанном HLSDK из примера Metamod-R этих констант нет.
    if (contents <= -9 && contents >= -14)
    {
        return NavContents::Water;
    }
    return NavContents::Empty;
}

void CollectSeedPoints(std::vector<Vec3>& outSeeds)
{
    static const char* const kSeedClasses[] = {
        "info_player_start",       // CT / одиночная игра
        "info_player_deathmatch",  // T / deathmatch
        "info_vip_start",
        "hostage_entity",
        "monster_scientist",       // заложники на старых картах
        "info_hostage_rescue",
        "info_bomb_target",
        "info_teleport_destination", // зоны, куда попадают только телепортом
        "armoury_entity",
        "info_player_coop",
        "info_player_teamspawn",
    };

    const int maxEntities = gpGlobals ? gpGlobals->maxEntities : 0;
    for (int i = 1; i < maxEntities; ++i)
    {
        edict_t* ent = INDEXENT(i);
        if (!ent || ent->free || !ent->pvPrivateData)
        {
            continue;
        }

        const char* classname = STRING(ent->v.classname);
        if (!classname || !classname[0])
        {
            continue;
        }

        bool isSeed = (ent->v.flags & FL_CLIENT) != 0; // уже подключившиеся игроки
        for (const char* seedClass : kSeedClasses)
        {
            if (isSeed)
            {
                break;
            }
            isSeed = std::strcmp(classname, seedClass) == 0;
        }
        if (!isSeed)
        {
            continue;
        }

        outSeeds.emplace_back(ent->v.origin.x, ent->v.origin.y, ent->v.origin.z);
    }
}

EntityHullInfo ComputeEntityHull(const edict_t* ent)
{
    EntityHullInfo info;
    if (!ent)
    {
        return info;
    }

    const Vector& mins = ent->v.mins;
    const Vector& maxs = ent->v.maxs;
    const float width = maxs.x - mins.x;
    const float height = maxs.z - mins.z;

    if (width <= 8.0f)
    {
        // Движок сталкивает такую entity с миром как точку в origin: bbox не участвует.
        info.hull = HullIndex::Point;
        info.feetOffset = Vec3(0.0f, 0.0f, 0.0f);
        return info;
    }

    if (width <= 36.0f)
    {
        info.hull = (height <= 36.0f) ? HullIndex::Ducking : HullIndex::Standing;
    }
    else
    {
        info.hull = HullIndex::Large;
    }

    const float halfWidth = GetHullDef(info.hull).halfWidth;
    info.feetOffset = Vec3(mins.x + halfWidth, mins.y + halfWidth, mins.z);

    // ReHLDS умеет центрировать хулл по симметричному bbox (sv_rehlds_hull_centering).
    static cvar_t* hullCentering = nullptr;
    static bool lookedUp = false;
    if (!lookedUp)
    {
        hullCentering = g_engfuncs.pfnCVarGetPointer("sv_rehlds_hull_centering");
        lookedUp = true;
    }
    if (hullCentering && hullCentering->value != 0.0f)
    {
        if (mins.x + maxs.x == 0.0f) info.feetOffset.x = 0.0f;
        if (mins.y + maxs.y == 0.0f) info.feetOffset.y = 0.0f;
    }

    return info;
}

void EntityMoveTracer::Trace(const Vec3& fromFeet, const Vec3& toFeet, HullTrace& out)
{
    const Vector start(fromFeet.x - m_feetOffset.x, fromFeet.y - m_feetOffset.y, fromFeet.z - m_feetOffset.z);
    const Vector end(toFeet.x - m_feetOffset.x, toFeet.y - m_feetOffset.y, toFeet.z - m_feetOffset.z);

    TraceResult tr{};
    g_engfuncs.pfnTraceMonsterHull(m_ent, start, end, dont_ignore_monsters, m_ent, &tr);

    out.startSolid = tr.fStartSolid != 0;
    out.allSolid = tr.fAllSolid != 0;
    out.fraction = tr.flFraction;
    out.endPos = Vec3(tr.vecEndPos.x + m_feetOffset.x, tr.vecEndPos.y + m_feetOffset.y, tr.vecEndPos.z + m_feetOffset.z);
    out.normal = Vec3(tr.vecPlaneNormal.x, tr.vecPlaneNormal.y, tr.vecPlaneNormal.z);
    out.entity = (out.startSolid || out.fraction < 1.0f) ? EntityIndexOf(tr.pHit) : -1;
}

} // namespace goldnav
