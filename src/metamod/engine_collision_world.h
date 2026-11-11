// Боевая реализация ICollisionWorld поверх TRACE_HULL/POINT_CONTENTS движка, плюс
// всё, что навигации нужно знать о конкретной entity: каким хуллом движок сталкивает
// её с миром и где у этого хулла "ноги" относительно origin.
#pragma once

#include <vector>
#include <extdll.h>

#include "navmesh/collision_world.h"
#include "pathfollower/ground_mover.h"

namespace goldnav {

class EngineCollisionWorld : public ICollisionWorld
{
public:
    void TraceHull(const Vec3& startFeet, const Vec3& endFeet, HullIndex hull, HullTrace& out) const override;
    NavContents PointContents(const Vec3& point) const override;
};

// Точки, от которых начинается заливка навмеша: спавны игроков, заложники, точки
// телепортов и т.п. — места, где на карте заведомо "можно стоять".
void CollectSeedPoints(std::vector<Vec3>& outSeeds);

// Как движок сталкивает entity с миром. Для entity нестандартного размера движок
// берёт ближайший стандартный хулл и совмещает его МИНИМАЛЬНЫЙ угол с минимальным
// углом bbox entity (см. SV_HullForBsp) — то есть хулл оказывается смещён
// относительно origin. feetOffset учитывает это смещение: feet = origin + feetOffset.
struct EntityHullInfo
{
    HullIndex hull = HullIndex::Standing;
    Vec3 feetOffset;
};

EntityHullInfo ComputeEntityHull(const edict_t* ent);

// IMoveTracer для конкретной entity: TRACE_MONSTER_HULL её собственным bbox, с
// учётом остальных entity (игроки, NPC, двери).
class EntityMoveTracer : public IMoveTracer
{
public:
    EntityMoveTracer(edict_t* ent, const Vec3& feetOffset) : m_ent(ent), m_feetOffset(feetOffset) {}
    void Trace(const Vec3& fromFeet, const Vec3& toFeet, HullTrace& out) override;

private:
    edict_t* m_ent;
    Vec3 m_feetOffset;
};

} // namespace goldnav
