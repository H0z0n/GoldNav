#include "nav_probe.h"

#include <cmath>

namespace goldnav {

namespace {

// Приподнимаем хулл над полом на время горизонтального свипа: пол после трассы
// вниз находится в 1/32 юнита под ногами, и без зазора свип цеплялся бы за
// малейшую неровность.
constexpr float kLift = 1.0f;

struct MicroStep
{
    bool ok = false;      // удалось продвинуться до цели подшага
    bool dropped = false; // по дороге пол ушёл вниз больше чем на stepHeight
    Vec3 feet;
};

// Один шаг "как у ходока": сначала по ровному, если упёрлись — с подъёмом на
// ступеньку; затем опускаемся на пол.
MicroStep StepOnce(const ICollisionWorld& world, const Vec3& from, float toX, float toY,
                   HullIndex hull, const ProbeConfig& config, float dropBudget)
{
    MicroStep result;
    HullTrace tr;

    const Vec3 start(from.x, from.y, from.z + kLift);
    Vec3 top;
    bool moved = false;

    world.TraceHull(start, Vec3(toX, toY, start.z), hull, tr);
    if (tr.startSolid || tr.allSolid)
    {
        return result;
    }
    if (tr.fraction >= 1.0f)
    {
        top = tr.endPos;
        moved = true;
    }
    else
    {
        // Поднимаемся ровно на stepHeight от пола (а не от приподнятого start): иначе
        // пробник брал бы ступеньки на kLift выше, чем реальный "ходок", и в граф
        // попадали бы рёбра, по которым entity физически не проходит.
        HullTrace up;
        world.TraceHull(start, Vec3(start.x, start.y, from.z + config.stepHeight), hull, up);
        if (!up.startSolid && !up.allSolid && up.endPos.z - start.z > 0.5f)
        {
            const Vec3 raised = up.endPos;
            world.TraceHull(raised, Vec3(toX, toY, raised.z), hull, tr);
            if (tr.Clear())
            {
                top = tr.endPos;
                moved = true;
            }
        }
    }

    if (!moved)
    {
        return result;
    }

    // Вниз: сначала в пределах ступеньки (обычная ходьба)...
    const float raise = top.z - from.z;
    HullTrace down;
    world.TraceHull(top, Vec3(top.x, top.y, top.z - raise - config.stepHeight), hull, down);
    if (down.startSolid || down.allSolid)
    {
        return result;
    }

    if (down.fraction >= 1.0f)
    {
        // ...а если пола нет — это уступ: падаем дальше, пока хватает бюджета высоты.
        if (dropBudget <= 0.0f)
        {
            return result;
        }
        const Vec3 ledge = down.endPos;
        world.TraceHull(ledge, Vec3(ledge.x, ledge.y, ledge.z - dropBudget), hull, down);
        if (down.startSolid || down.allSolid || down.fraction >= 1.0f)
        {
            return result;
        }
        result.dropped = true;
    }

    if (down.normal.z < config.minGroundNormalZ)
    {
        return result; // приземлились бы на крутой склон
    }

    result.ok = true;
    result.feet = down.endPos;
    return result;
}

} // namespace

bool HullFits(const ICollisionWorld& world, const Vec3& feet, HullIndex hull)
{
    HullTrace tr;
    const Vec3 p(feet.x, feet.y, feet.z + kLift);
    world.TraceHull(p, p, hull, tr);
    return !tr.startSolid && !tr.allSolid;
}

bool DropToFloor(const ICollisionWorld& world, const Vec3& feet, HullIndex hull, float maxDistance,
                 const ProbeConfig& config, Vec3& outFeet)
{
    HullTrace tr;
    const Vec3 start(feet.x, feet.y, feet.z + kLift);
    world.TraceHull(start, Vec3(start.x, start.y, start.z - maxDistance - kLift), hull, tr);
    if (!tr.Hit())
    {
        return false;
    }
    if (tr.normal.z < config.minGroundNormalZ)
    {
        return false;
    }
    outFeet = tr.endPos;
    return true;
}

ProbeResult ProbeMove(const ICollisionWorld& world, const Vec3& fromFeet, float toX, float toY,
                      HullIndex hull, const ProbeConfig& config)
{
    ProbeResult result;

    const float dx = toX - fromFeet.x;
    const float dy = toY - fromFeet.y;
    const float dist = std::sqrt(dx * dx + dy * dy);
    if (dist < 0.01f)
    {
        result.kind = ProbeKind::Walk;
        result.feet = fromFeet;
        return result;
    }

    // Быстрый путь: одним шагом. На открытом месте это две трассы на ребро.
    const MicroStep single = StepOnce(world, fromFeet, toX, toY, hull, config, config.maxDrop);
    if (single.ok && !single.dropped && single.feet.z - fromFeet.z >= -config.stepHeight)
    {
        result.kind = ProbeKind::Walk;
        result.feet = single.feet;
        return result;
    }

    // Точный путь: мелкими подшагами, каждый со своим подъёмом/спуском на ступеньку.
    // Нужен для крутых лестниц (несколько ступенек на одну клетку) и чтобы отличить
    // спуск по лестнице (Walk) от прыжка с уступа (Drop).
    const int steps = static_cast<int>(std::ceil(dist / config.fineStep));
    Vec3 pos = fromFeet;
    bool dropped = false;
    float dropBudget = config.maxDrop;

    for (int i = 1; i <= steps; ++i)
    {
        const float t = static_cast<float>(i) / static_cast<float>(steps);
        const float tx = fromFeet.x + dx * t;
        const float ty = fromFeet.y + dy * t;

        const MicroStep step = StepOnce(world, pos, tx, ty, hull, config, dropBudget);
        if (!step.ok)
        {
            // Различаем "стена" и "пропасть" только для диагностики вызывающего.
            result.kind = ProbeKind::Blocked;
            return result;
        }
        if (step.dropped)
        {
            dropped = true;
            dropBudget -= (pos.z - step.feet.z);
        }
        pos = step.feet;
    }

    result.kind = dropped ? ProbeKind::Drop : ProbeKind::Walk;
    result.feet = pos;
    return result;
}

} // namespace goldnav
