// GroundMove — кинематический "ходок": сам двигает entity по полу трассировками,
// не полагаясь на физику движка.
//
// Почему не pev->velocity: физика движка для не-игроков (SV_Physics_Step ->
// SV_FlyMove) умеет только скользить. Она не поднимается на ступеньки вообще —
// entity со скоростью упирается в любой порожек в пару юнитов и встаёт. Плюс трение
// движка каждый кадр съедает заданную скорость на величину, зависящую от FPS.
// Поэтому шаг делается как у игрока (PM_WalkMove): скольжение вдоль стен, при
// упоре — попытка "вверх на ступеньку, вперёд, вниз", затем прилипание к полу на
// спусках; без пола под ногами — падение с гравитацией.
//
// Engine-agnostic: геометрию видит только через IMoveTracer.
#pragma once

#include "navmesh/collision_world.h"

namespace goldnav {

// Свип собственной коллизионной формы entity между двумя feet-позициями, включая
// столкновения с другими entity (игроки, NPC, двери). Боевая реализация —
// TRACE_MONSTER_HULL (см. metamod/entity_follow_bridge.cpp).
class IMoveTracer
{
public:
    virtual ~IMoveTracer() = default;
    virtual void Trace(const Vec3& fromFeet, const Vec3& toFeet, HullTrace& out) = 0;
};

struct MoverConfig
{
    float stepHeight = 18.0f;
    float gravity = 800.0f;
    float minGroundNormalZ = 0.7f;
    float maxFallSpeed = 2000.0f;
};

struct MoverState
{
    bool initialized = false;
    bool onGround = false;
    float fallSpeed = 0.0f;  // вертикальная скорость (юнитов/с): вниз положительная, в прыжке — отрицательная
    float fallStartZ = 0.0f; // высота, с которой началось текущее падение
    int groundEntity = -1;   // на чём стоим (0 — мир)
};

struct MoveOutcome
{
    Vec3 feet;                // новая позиция
    bool blocked = false;     // упёрлись во что-то, чего не смогли ни обойти скольжением, ни перешагнуть
    Vec3 blockNormal;         // нормаль препятствия (горизонтальная)
    int blockEntity = -1;     // во что упёрлись (0 — мир, >0 — entity)
    int touchedEntity = -1;   // entity, которой коснулись за этот шаг (даже если проскользили мимо)
    bool landed = false;      // в этом шаге приземлились после падения
    float landedDrop = 0.0f;  // с какой высоты упали
    bool stuckInSolid = false; // entity стоит внутри геометрии и вытолкнуть её не удалось
};

// Начинает прыжок: entity отрывается от земли со скоростью, достаточной, чтобы поднять
// ноги на height юнитов. Дальше её ведёт обычная "воздушная" ветка GroundMove.
void StartJump(MoverState& state, const Vec3& feet, float height, float gravity);

// Один шаг движения за dt секунд с желаемой горизонтальной скоростью wishVelocity
// (Z игнорируется).
MoveOutcome GroundMove(IMoveTracer& tracer, const Vec3& feet, const Vec3& wishVelocity, float dt,
                       MoverState& state, const MoverConfig& config);

} // namespace goldnav
