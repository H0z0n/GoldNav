#include "path_follower.h"

#include <algorithm>
#include <cmath>

namespace goldnav {

namespace {

Vec3 Flat(const Vec3& v)
{
    return Vec3(v.x, v.y, 0.0f);
}

} // namespace

bool PathFollower::AdvanceWaypoints(FollowState& state, const Vec3& feet, bool onGround, float speed) const
{
    // Чем быстрее идём, тем раньше начинаем поворачивать к следующей точке: на
    // скорости в саму точку всё равно не вписаться, а поворот "в последний момент"
    // выглядит рывком.
    const float turnRadius = std::max(m_config.waypointRadius,
                                      std::min(m_config.maxTurnInRadius, speed * m_config.turnInPerSpeed));

    while (state.currentIndex < state.path.size())
    {
        const Vec3& target = state.path[state.currentIndex];
        const bool isLast = (state.currentIndex + 1 == state.path.size());
        const float dist = Distance2D(feet, target);
        const float heightDiff = std::fabs(target.z - feet.z);
        const bool heightOk = heightDiff <= (isLast ? m_config.heightTolerance : m_config.waypointHeightTolerance);

        bool reached = false;
        if (isLast)
        {
            // В цель "прибываем" только стоя на земле: пролетая над ней в падении — нет.
            reached = onGround && heightOk && dist <= m_config.finalRadius;
        }
        else if (heightOk)
        {
            if (dist <= turnRadius)
            {
                reached = true;
            }
            else if (state.hasSegmentStart && dist <= m_config.passLateralLimit)
            {
                // Проскочили: мы уже за плоскостью, проходящей через точку поперёк
                // направления подхода к ней.
                const Vec3 approach = Flat(target - state.segmentStart);
                const Vec3 beyond = Flat(feet - target);
                if (approach.LengthSq() > 1.0f && Dot(approach, beyond) > 0.0f)
                {
                    reached = true;
                }
            }
        }

        if (!reached)
        {
            return false;
        }

        ++state.currentIndex;
        state.segmentStart = feet;
        state.hasSegmentStart = true;
    }

    return true;
}

FollowStep PathFollower::Update(FollowState& state, IMoveTracer& tracer, MoverState& mover,
                                const Vec3& feet, float speed, float dt) const
{
    FollowStep step;
    step.feet = feet;

    if (!state.active || state.path.empty())
    {
        step.result = FollowResult::NoPath;
        return step;
    }

    if (!state.hasSegmentStart)
    {
        state.segmentStart = feet;
        state.hasSegmentStart = true;
    }

    // Пока mover не инициализирован, считаем что стоим: первый же GroundMove уточнит.
    const bool groundedBefore = !mover.initialized || mover.onGround;

    if (AdvanceWaypoints(state, feet, groundedBefore, speed))
    {
        state.active = false;
        state.velocity = Vec3();
        step.result = FollowResult::Arrived;
        return step;
    }

    const Vec3 target = state.path[state.currentIndex];
    const bool isLast = (state.currentIndex + 1 == state.path.size());
    const float distToTarget = Distance2D(feet, target);
    const float stepHeight = m_config.mover.stepHeight;

    // Мы прямо под точкой, а она этажом выше: по этому пути до неё уже не дойти.
    // Если же точка НИЖЕ нас — это спрыгивание с уступа: продолжаем идти к ней,
    // пока край не кончится под ногами (хулл шире шага сетки и ещё опирается на уступ).
    if (groundedBefore && distToTarget <= m_config.waypointRadius &&
        target.z - feet.z > std::max(m_config.heightTolerance, state.jumpHeight + 2.0f))
    {
        step.result = FollowResult::OffPath;
        return step;
    }

    Vec3 direction = Flat(target - feet).Normalized();

    float moveSpeed = speed;
    const bool finalApproach = isLast && distToTarget < m_config.slowDownRadius;
    if (finalApproach)
    {
        const float factor = std::max(m_config.minSpeedFactor, distToTarget / m_config.slowDownRadius);
        moveSpeed = speed * factor;
    }
    // Не перелетаем цель: последний шаг ровно до неё.
    if (isLast && dt > 0.0f && moveSpeed * dt > distToTarget)
    {
        moveSpeed = distToTarget / dt;
    }

    Vec3 wishDir = direction;
    const bool sidestepping = state.sidestepTimeLeft > 0.0f;
    if (sidestepping)
    {
        // Идём наискось: вбок и вперёд. Давление вперёд прижимает к препятствию, и
        // скольжение ведёт вдоль его грани ровно до угла — без лишнего крюка.
        state.sidestepTimeLeft -= dt;
        wishDir = (direction + state.sidestepDir).Normalized();
        if (wishDir.LengthSq() < 0.5f)
        {
            wishDir = state.sidestepDir;
        }
    }

    // --- Инерция ---
    // Скорость не прыгает к желаемой мгновенно, а подтягивается к ней с ограниченным
    // ускорением: плавный разгон, повороты по дуге вместо излома. В воздухе и на
    // последних юнитах перед целью управляем напрямую — там важна точность.
    const Vec3 desired = wishDir * moveSpeed;
    const bool airborne = mover.initialized && !mover.onGround;
    Vec3 velocity = desired;
    if (!airborne && !finalApproach && m_config.acceleration > 0.0f)
    {
        Vec3 delta = desired - state.velocity;
        const float maxDelta = m_config.acceleration * speed * dt;
        const float deltaLen = delta.Length2D();
        if (deltaLen > maxDelta && deltaLen > 0.0f)
        {
            delta = delta * (maxDelta / deltaLen);
        }
        velocity = Flat(state.velocity + delta);
    }

    const MoveOutcome outcome = GroundMove(tracer, feet, velocity, dt, mover, m_config.mover);

    step.feet = outcome.feet;
    step.touchedEntity = outcome.touchedEntity;
    step.stuckInSolid = outcome.stuckInSolid;
    step.blocked = outcome.blocked;
    step.blockEntity = outcome.blockEntity;
    step.blockNormal = outcome.blockNormal;

    const float moved = Distance2D(feet, outcome.feet);
    step.speed = dt > 0.0f ? moved / dt : 0.0f;
    step.facing = moved > 0.05f ? Flat(outcome.feet - feet).Normalized() : direction;
    step.result = FollowResult::Moving;

    // Упёрлись в стену — "разгон" в неё не копится: продолжаем от фактической скорости.
    // Иначе помним заданную (а не измеренную: при очень коротком кадре сдвиг за кадр
    // меньше порога движения, и измеренная скорость навсегда осталась бы нулевой).
    if (outcome.blocked && mover.onGround && dt > 0.0f)
    {
        state.velocity = Flat(outcome.feet - feet) * (1.0f / dt);
    }
    else
    {
        state.velocity = airborne ? desired : velocity;
    }

    // --- Прыжок ---
    // Следующая точка пути — на уступе выше ступеньки, мы упёрлись в него, а прыгать
    // на такую высоту entity умеет: отталкиваемся. Дальше в воздухе идём к точке.
    state.jumpCooldown -= dt;
    if (mover.onGround && outcome.blocked && state.jumpHeight > stepHeight && state.jumpCooldown <= 0.0f)
    {
        const float rise = target.z - outcome.feet.z;
        if (rise > stepHeight * 0.75f && rise <= state.jumpHeight + 2.0f &&
            Distance2D(outcome.feet, target) <= m_config.jumpMaxDistance)
        {
            StartJump(mover, outcome.feet, rise + m_config.jumpMargin, m_config.mover.gravity);
            state.jumpCooldown = m_config.jumpCooldown;
            state.windowValid = false;
            step.jumped = true;
            return step;
        }
    }

    // Препятствие осталось позади (перестали его касаться) — заканчиваем обход,
    // оставив короткий хвост, чтобы угол хулла тоже прошёл.
    if (sidestepping && !outcome.blocked && outcome.touchedEntity <= 0)
    {
        state.sidestepTimeLeft = std::min(state.sidestepTimeLeft, m_config.sidestepClearTime);
    }

    // Упёрлись в подвижную entity — сразу начинаем обходить её боком, не дожидаясь
    // окна контроля: человек не толкается в стоящего на дороге, а огибает.
    if (outcome.blocked && outcome.blockEntity > 0 && state.sidestepTimeLeft <= 0.0f)
    {
        Vec3 tangent(-outcome.blockNormal.y, outcome.blockNormal.x, 0.0f);
        const float along = Dot(tangent, direction);
        if (std::fabs(along) < 0.15f)
        {
            tangent = tangent * static_cast<float>(state.sidestepSign); // в лоб — сторона по очереди
        }
        else if (along < 0.0f)
        {
            tangent = tangent * -1.0f;
        }
        state.sidestepDir = tangent.Normalized();
        state.sidestepTimeLeft = m_config.sidestepTime;
    }

    // Приземлились после падения не там, куда вёл путь — пусть перестроят.
    if (outcome.landed && outcome.landedDrop > stepHeight)
    {
        if (Distance2D(outcome.feet, target) > m_config.offPathLandingDistance ||
            std::fabs(target.z - outcome.feet.z) > m_config.heightTolerance)
        {
            step.result = FollowResult::OffPath;
            return step;
        }
        state.windowValid = false; // после прыжка окно продвижения начинаем заново
    }

    // --- Контроль продвижения ---
    // Смотрим не на "двигались ли", а на "стали ли ближе к текущей точке": entity,
    // которая елозит вдоль стены туда-сюда, тоже застряла.
    if (!mover.onGround)
    {
        state.windowValid = false; // в полёте продвижение не оцениваем
        return step;
    }

    if (!state.windowValid)
    {
        state.windowValid = true;
        state.windowTime = 0.0f;
        state.windowIntended = 0.0f;
        state.windowSidestep = 0.0f;
        state.windowStartDist = Distance2D(outcome.feet, target);
        state.windowIndex = state.currentIndex;
        return step;
    }

    state.windowTime += dt;
    state.windowIntended += moveSpeed * dt;
    if (sidestepping)
    {
        state.windowSidestep += dt;
    }

    if (state.windowTime >= m_config.progressWindow)
    {
        bool progressed = state.currentIndex != state.windowIndex;
        if (!progressed)
        {
            const float gained = state.windowStartDist - Distance2D(outcome.feet, target);
            progressed = gained >= state.windowIntended * m_config.minProgressFraction;
        }

        // Окно, большей частью ушедшее на обход препятствия, застреванием не считаем:
        // пока огибаем, к цели не приближаемся. Но и огибать бесконечно нельзя.
        const bool mostlySidestep = state.windowSidestep > state.windowTime * 0.5f;

        if (progressed)
        {
            state.stuckStrikes = 0;
            state.idleWindows = 0;
        }
        else if (mostlySidestep && ++state.idleWindows < m_config.idleWindowsBeforeStuck)
        {
            // ждём, чем закончится обход
        }
        else
        {
            ++state.stuckStrikes;
            if (state.stuckStrikes >= m_config.strikesBeforeStuck || state.idleWindows >= m_config.idleWindowsBeforeStuck)
            {
                state.stuckStrikes = 0;
                state.idleWindows = 0;
                state.windowValid = false;
                state.sidestepSign = -state.sidestepSign;
                step.result = FollowResult::Stuck;
                return step;
            }

            // Первое окно без продвижения: пробуем сдвинуться вбок (в другую сторону,
            // чем в прошлый раз), прежде чем просить перестроить путь.
            state.sidestepSign = -state.sidestepSign;
            Vec3 tangent(-direction.y, direction.x, 0.0f);
            if (outcome.blocked && outcome.blockNormal.Length2DSq() > 0.25f)
            {
                tangent = Vec3(-outcome.blockNormal.y, outcome.blockNormal.x, 0.0f);
            }
            state.sidestepDir = (tangent * static_cast<float>(state.sidestepSign)).Normalized();
            state.sidestepTimeLeft = m_config.sidestepTime;
        }

        state.windowTime = 0.0f;
        state.windowIntended = 0.0f;
        state.windowSidestep = 0.0f;
        state.windowStartDist = Distance2D(outcome.feet, target);
        state.windowIndex = state.currentIndex;
    }

    return step;
}

} // namespace goldnav
