// Пробные перемещения хулла по геометрии карты: "сможет ли entity дойти отсюда
// вон туда и где окажется". Одна и та же логика используется генератором навмеша
// (проверка рёбер), посевом (привязка произвольной точки к сетке) и планировщиком
// пути (привязка старта/цели к узлам) — поэтому рёбра графа по построению
// согласованы с тем, как "ходок" (pathfollower/ground_mover) реально двигает entity:
// шаг вверх на ступеньку, шаг вперёд, шаг вниз.
#pragma once

#include "collision_world.h"

namespace goldnav {

struct ProbeConfig
{
    float stepHeight = 18.0f;       // максимальная высота ступеньки (sv_stepsize)
    float maxDrop = 150.0f;         // с какой высоты ещё можно спрыгнуть
    float minGroundNormalZ = 0.7f;  // положе этого — пол, круче — склон, по которому не ходят
    float fineStep = 4.0f;          // длина подшага при точной проверке лестниц
};

enum class ProbeKind
{
    Blocked, // упёрлись в стену/слишком высокую ступеньку/крутой склон
    Walk,    // дошли, двигаясь по полу и ступенькам
    Drop,    // дошли, но по дороге спрыгнули с уступа выше stepHeight
    NoFloor, // впереди провал глубже maxDrop
};

struct ProbeResult
{
    ProbeKind kind = ProbeKind::Blocked;
    Vec3 feet; // где оказались (валидно для Walk/Drop)

    bool Reached() const { return kind == ProbeKind::Walk || kind == ProbeKind::Drop; }
};

// Пробует пройти хуллом от fromFeet (хулл должен стоять на полу) по прямой до
// точки (toX, toY) и возвращает, где и как он там окажется.
ProbeResult ProbeMove(const ICollisionWorld& world, const Vec3& fromFeet, float toX, float toY,
                      HullIndex hull, const ProbeConfig& config);

// Опускает хулл из точки вниз до пола. false, если хулл в точке не помещается,
// пол дальше maxDistance либо под точкой крутой склон.
bool DropToFloor(const ICollisionWorld& world, const Vec3& feet, HullIndex hull, float maxDistance,
                 const ProbeConfig& config, Vec3& outFeet);

// Помещается ли хулл в точке (стоя feet-ами на ней).
bool HullFits(const ICollisionWorld& world, const Vec3& feet, HullIndex hull);

} // namespace goldnav
