// Единственное место, где результат PathFollower (engine-agnostic) записывается в
// реальный edict_t.
//
// GoldNav двигает entity кинематически: сам выставляет origin (через SET_ORIGIN,
// чтобы движок перелинковал entity) и держит velocity нулевой, а FL_ONGROUND —
// взведённым. Так физика движка для этой entity остаётся пассивной при любом
// movetype: SV_Physics_Step/Toss ничего не делают, если entity "стоит на земле без
// скорости", а для MOVETYPE_NONE/FLY/NOCLIP гравитации нет и так. Иначе движок
// двигал бы entity второй раз поверх нас.
#pragma once

#include <extdll.h>

#include "pathfollower/path_follower.h"
#include "metamod/engine_collision_world.h"

namespace goldnav {

// Записывает шаг в entity: позицию, "стою на земле", поворот по ходу движения.
void ApplyFollowStep(edict_t* ent, const EntityHullInfo& hull, const FollowStep& step,
                     const MoverState& mover, bool turnToFace, float frameTime);

// Останавливает entity на месте (обнуляет скорость).
void HaltEntity(edict_t* ent);

// Вызывает Touch игровой DLL для пары entity в обе стороны — как это сделал бы
// движок, столкни он их сам (SV_Impact). Нужно, потому что при кинематическом
// движении движок столкновений не видит: без этого, например, двери CS не
// открывались бы от "прикосновения" NPC.
void EmulateTouch(edict_t* ent, int otherIndex);

} // namespace goldnav
