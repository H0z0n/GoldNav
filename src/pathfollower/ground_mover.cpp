#include "ground_mover.h"

#include <algorithm>
#include <cmath>

namespace goldnav {

namespace {

constexpr int kMaxBumps = 4;
constexpr float kMinMove = 0.01f;

struct SlideResult
{
    Vec3 end;
    bool hit = false;       // было хоть одно столкновение
    bool stuck = false;     // стартовали внутри геометрии
    bool hitGround = false; // среди столкновений была поверхность-пол
    bool hitCeiling = false;
    Vec3 wallNormal;        // нормаль первой "стены"
    bool hitWall = false;
    int wallEntity = -1;
    int groundEntity = -1;
    int touchedEntity = -1;
};

// Скольжение вдоль поверхностей (аналог SV_FlyMove/PM_FlyMove): упёрлись — убираем из
// остатка движения составляющую вглубь поверхности и продолжаем.
// flattenWalls: крутые поверхности считаются вертикальными стенами — иначе скольжение
// "заезжало" бы вверх по склону, по которому ходить нельзя.
SlideResult Slide(IMoveTracer& tracer, const Vec3& start, const Vec3& delta, bool flattenWalls,
                  float minGroundNormalZ)
{
    SlideResult result;
    result.end = start;

    Vec3 remaining = delta;
    Vec3 planes[kMaxBumps];
    int planeCount = 0;

    for (int bump = 0; bump < kMaxBumps; ++bump)
    {
        if (remaining.LengthSq() < kMinMove * kMinMove)
        {
            break;
        }

        HullTrace tr;
        tracer.Trace(result.end, result.end + remaining, tr);

        if (tr.startSolid || tr.allSolid)
        {
            result.stuck = true;
            break;
        }

        if (tr.fraction > 0.0f)
        {
            result.end = tr.endPos;
            planeCount = 0; // сдвинулись — старые плоскости больше не ограничивают
        }
        if (tr.fraction >= 1.0f)
        {
            break;
        }

        result.hit = true;
        if (tr.entity > 0)
        {
            result.touchedEntity = tr.entity;
        }

        Vec3 normal = tr.normal;
        const bool isGround = normal.z >= minGroundNormalZ;
        if (isGround)
        {
            result.hitGround = true;
            result.groundEntity = tr.entity;
        }
        else
        {
            if (normal.z < -0.7f)
            {
                result.hitCeiling = true;
            }
            if (flattenWalls && normal.z > 0.0f)
            {
                normal.z = 0.0f;
                normal = normal.Normalized();
                if (normal.LengthSq() < 0.5f)
                {
                    normal = tr.normal;
                }
            }
            if (!result.hitWall)
            {
                result.hitWall = true;
                result.wallNormal = normal;
                result.wallEntity = tr.entity;
            }
        }

        remaining = remaining * (1.0f - tr.fraction);
        if (planeCount >= kMaxBumps)
        {
            break;
        }
        planes[planeCount++] = normal;

        // Ищем направление, параллельное всем накопленным плоскостям.
        Vec3 clipped;
        int i = 0;
        for (; i < planeCount; ++i)
        {
            const float into = Dot(remaining, planes[i]);
            clipped = remaining - planes[i] * into;

            int j = 0;
            for (; j < planeCount; ++j)
            {
                if (j != i && Dot(clipped, planes[j]) < -0.001f)
                {
                    break;
                }
            }
            if (j == planeCount)
            {
                break;
            }
        }

        if (i != planeCount)
        {
            remaining = clipped;
        }
        else if (planeCount == 2)
        {
            // Зажаты между двумя плоскостями — идём вдоль их стыка.
            const Vec3 crease = Cross(planes[0], planes[1]).Normalized();
            remaining = crease * Dot(crease, remaining);
        }
        else
        {
            break;
        }

        // Развернуло назад — стоим, чтобы не дрожать в углу.
        if (Dot(remaining, delta) <= 0.0f)
        {
            break;
        }
    }

    return result;
}

bool IsGroundHit(const HullTrace& tr, float minGroundNormalZ)
{
    return tr.Hit() && tr.normal.z >= minGroundNormalZ;
}

} // namespace

void StartJump(MoverState& state, const Vec3& feet, float height, float gravity)
{
    if (height <= 0.0f || gravity <= 0.0f)
    {
        return;
    }
    state.initialized = true;
    state.onGround = false;
    state.groundEntity = -1;
    state.fallStartZ = feet.z;
    state.fallSpeed = -std::sqrt(2.0f * gravity * height);
}

MoveOutcome GroundMove(IMoveTracer& tracer, const Vec3& feetIn, const Vec3& wishVelocity, float dt,
                       MoverState& state, const MoverConfig& config)
{
    MoveOutcome outcome;
    outcome.feet = feetIn;

    if (dt <= 0.0f)
    {
        return outcome;
    }

    Vec3 feet = feetIn;
    HullTrace tr;

    // Entity оказалась внутри геометрии (заспавнили в полу, придавило дверью) —
    // пробуем вытолкнуть вверх; не вышло — честно сообщаем.
    tracer.Trace(feet, feet, tr);
    if (tr.startSolid || tr.allSolid)
    {
        static const float kLifts[] = { 1.0f, 2.0f, 4.0f, 8.0f, 18.0f, 36.0f };
        bool freed = false;
        for (float lift : kLifts)
        {
            const Vec3 candidate(feet.x, feet.y, feet.z + lift);
            tracer.Trace(candidate, candidate, tr);
            if (!tr.startSolid && !tr.allSolid)
            {
                feet = candidate;
                freed = true;
                break;
            }
        }
        if (!freed)
        {
            outcome.stuckInSolid = true;
            outcome.blocked = true;
            return outcome;
        }
        state.initialized = false;
    }

    if (!state.initialized)
    {
        // Первый шаг: выясняем, стоим ли мы на чём-нибудь.
        tracer.Trace(feet, Vec3(feet.x, feet.y, feet.z - 2.0f), tr);
        state.onGround = IsGroundHit(tr, config.minGroundNormalZ);
        state.groundEntity = state.onGround ? tr.entity : -1;
        state.fallSpeed = 0.0f;
        state.fallStartZ = feet.z;
        state.initialized = true;
    }

    const Vec3 wish(wishVelocity.x, wishVelocity.y, 0.0f);

    if (state.onGround)
    {
        const Vec3 delta = wish * dt;
        Vec3 end = feet;

        if (delta.LengthSq() >= kMinMove * kMinMove)
        {
            // Попытка 1: по ровному, со скольжением вдоль стен.
            const SlideResult flat = Slide(tracer, feet, delta, true, config.minGroundNormalZ);
            end = flat.end;
            bool hitWall = flat.hitWall || flat.stuck;
            Vec3 wallNormal = flat.wallNormal;
            int wallEntity = flat.wallEntity;
            outcome.touchedEntity = flat.touchedEntity;

            // Попытка 2: если упёрлись — вверх на ступеньку, вперёд, обратно вниз.
            // Берём тот вариант, что продвинул дальше.
            if (flat.hit || flat.stuck)
            {
                HullTrace up;
                tracer.Trace(feet, Vec3(feet.x, feet.y, feet.z + config.stepHeight), up);
                const float raise = (up.startSolid || up.allSolid) ? 0.0f : (up.endPos.z - feet.z);

                if (raise > 1.0f)
                {
                    const SlideResult stepped = Slide(tracer, up.endPos, delta, true, config.minGroundNormalZ);
                    if (!stepped.stuck)
                    {
                        HullTrace down;
                        tracer.Trace(stepped.end, Vec3(stepped.end.x, stepped.end.y, stepped.end.z - raise), down);

                        // Спуск обратно не должен закончиться на крутом склоне: туда не встать.
                        const bool landsOk = !down.startSolid && !down.allSolid &&
                                             (down.fraction >= 1.0f || down.normal.z >= config.minGroundNormalZ);
                        if (landsOk)
                        {
                            const float flatDist = Distance2D(feet, flat.end);
                            const float stepDist = Distance2D(feet, down.endPos);
                            if (stepDist > flatDist + 0.05f)
                            {
                                end = down.endPos;
                                hitWall = stepped.hitWall;
                                wallNormal = stepped.wallNormal;
                                wallEntity = stepped.wallEntity;
                                if (stepped.touchedEntity > 0)
                                {
                                    outcome.touchedEntity = stepped.touchedEntity;
                                }
                            }
                        }
                    }
                }
            }

            if (hitWall)
            {
                // "Заблокированы" — если прошли заметно меньше, чем собирались.
                const float wanted = delta.Length2D();
                const float moved = Distance2D(feet, end);
                if (moved < wanted * 0.5f)
                {
                    outcome.blocked = true;
                    outcome.blockNormal = wallNormal;
                    outcome.blockEntity = wallEntity;
                }
            }
        }

        // Прилипание к полу: на спуске по лестнице/склону не подпрыгиваем на каждой
        // ступеньке, а плавно съезжаем. Нет пола в пределах ступеньки — начинаем падать.
        HullTrace ground;
        tracer.Trace(end, Vec3(end.x, end.y, end.z - config.stepHeight), ground);
        if (ground.startSolid || ground.allSolid)
        {
            // Не должно случаться (end получен из трасс), но на всякий случай остаёмся на месте.
            end = feet;
        }
        else if (IsGroundHit(ground, config.minGroundNormalZ))
        {
            end = ground.endPos;
            state.groundEntity = ground.entity;
        }
        else
        {
            state.onGround = false;
            state.fallSpeed = 0.0f;
            state.fallStartZ = end.z;
            state.groundEntity = -1;
        }

        outcome.feet = end;
        return outcome;
    }

    // --- В воздухе ---
    state.fallSpeed = std::min(state.fallSpeed + config.gravity * dt, config.maxFallSpeed);
    const Vec3 delta(wish.x * dt, wish.y * dt, -state.fallSpeed * dt);

    const SlideResult air = Slide(tracer, feet, delta, false, config.minGroundNormalZ);
    outcome.feet = air.end;
    outcome.touchedEntity = air.touchedEntity;

    if (air.stuck)
    {
        // Зажало в воздухе (например, между entity) — считаем, что стоим.
        state.onGround = true;
        state.fallSpeed = 0.0f;
        outcome.blocked = true;
        return outcome;
    }

    if (air.hitCeiling && state.fallSpeed < 0.0f)
    {
        state.fallSpeed = 0.0f; // упёрлись головой — дальше только вниз
    }

    // Пол, задетый на взлёте (например, край пандуса), приземлением не считается.
    if (air.hitGround && state.fallSpeed >= 0.0f)
    {
        state.onGround = true;
        state.groundEntity = air.groundEntity;
        outcome.landed = true;
        outcome.landedDrop = state.fallStartZ - air.end.z;
        state.fallSpeed = 0.0f;
    }
    else if (air.hitWall)
    {
        outcome.blockNormal = air.wallNormal;
        outcome.blockEntity = air.wallEntity;
    }

    return outcome;
}

} // namespace goldnav
