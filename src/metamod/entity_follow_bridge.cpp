#include "entity_follow_bridge.h"

#include <cmath>
#include <meta_api.h>
#include <util.h> // INDEXENT

namespace goldnav {

namespace {

constexpr float kTurnRateDegPerSec = 540.0f;
constexpr float kRadToDeg = 57.29577951f;

float WrapDegrees(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

edict_t* GroundEdict(int groundEntity)
{
    edict_t* world = INDEXENT(0);
    if (groundEntity <= 0)
    {
        return world;
    }

    edict_t* ground = INDEXENT(groundEntity);
    if (!ground || ground->free)
    {
        return world;
    }
    // Стоя "на игроке/монстре", SV_Physics_Toss сам снимает FL_ONGROUND и начинает
    // двигать entity — поэтому такую опору подменяем миром. Лифты и поезда
    // (brush-энтити) оставляем: по groundentity движок возит стоящих на них.
    if (ground->v.flags & (FL_CLIENT | FL_MONSTER))
    {
        return world;
    }
    return ground;
}

} // namespace

void ApplyFollowStep(edict_t* ent, const EntityHullInfo& hull, const FollowStep& step,
                     const MoverState& mover, bool turnToFace, float frameTime)
{
    if (!ent)
    {
        return;
    }

    const Vector origin(step.feet.x - hull.feetOffset.x, step.feet.y - hull.feetOffset.y,
                        step.feet.z - hull.feetOffset.z);

    if (origin.x != ent->v.origin.x || origin.y != ent->v.origin.y || origin.z != ent->v.origin.z)
    {
        g_engfuncs.pfnSetOrigin(ent, origin);
    }

    ent->v.velocity = Vector(0.0f, 0.0f, 0.0f);
    ent->v.flags |= FL_ONGROUND;
    ent->v.groundentity = GroundEdict(mover.groundEntity);

    if (turnToFace && step.facing.Length2DSq() > 0.25f && frameTime > 0.0f)
    {
        const float wanted = std::atan2(step.facing.y, step.facing.x) * kRadToDeg;
        const float delta = WrapDegrees(wanted - ent->v.angles.y);
        const float maxTurn = kTurnRateDegPerSec * frameTime;

        float turn = delta;
        if (turn > maxTurn) turn = maxTurn;
        if (turn < -maxTurn) turn = -maxTurn;

        ent->v.angles.y = WrapDegrees(ent->v.angles.y + turn);
        ent->v.ideal_yaw = wanted;
    }
}

void HaltEntity(edict_t* ent)
{
    if (!ent)
    {
        return;
    }
    ent->v.velocity = Vector(0.0f, 0.0f, 0.0f);
}

void EmulateTouch(edict_t* ent, int otherIndex)
{
    if (!ent || otherIndex <= 0 || !gpGamedllFuncs || !gpGamedllFuncs->dllapi_table)
    {
        return;
    }

    edict_t* other = INDEXENT(otherIndex);
    if (!other || other == ent || other->free || !other->pvPrivateData || !ent->pvPrivateData)
    {
        return;
    }
    if ((ent->v.flags & FL_KILLME) || (other->v.flags & FL_KILLME))
    {
        return;
    }

    DLL_FUNCTIONS* dll = gpGamedllFuncs->dllapi_table;
    if (!dll->pfnTouch)
    {
        return;
    }

    if (other->v.solid != SOLID_NOT)
    {
        dll->pfnTouch(other, ent);
    }
    // Обработчик мог удалить любую из entity.
    if (ent->free || other->free || (ent->v.flags & FL_KILLME) || (other->v.flags & FL_KILLME))
    {
        return;
    }
    if (ent->v.solid != SOLID_NOT)
    {
        dll->pfnTouch(ent, other);
    }
}

} // namespace goldnav
