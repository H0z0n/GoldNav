#include "nav_generator.h"

#include <cmath>
#include "platform/platform_time.h"

namespace goldnav {

namespace {
// Насколько может отличаться высота пола, чтобы две "посадки" в одну клетку считались
// одним узлом. Разные этажи в одной колонке разнесены минимум на высоту хулла, так что
// половина ступеньки заведомо их не склеит.
constexpr float kNodeMergeTolerance = 9.0f;
constexpr float kSeedDropDistance = 4096.0f;
constexpr float kDeepWaterHeight = 40.0f;
} // namespace

void NavMeshGenerator::Begin(NavGrid& grid, const ICollisionWorld& world, NavLayer layer, const NavGenConfig& config)
{
    m_grid = &grid;
    m_world = &world;
    m_config = config;
    m_queue.clear();
    m_expandedCount = 0;
    m_clearanceDirty = false;
    m_hitNodeLimit = false;

    grid.Initialize(layer, config.cellSize);
}

uint32_t NavMeshGenerator::FindOrCreateNode(int gx, int gy, const Vec3& feet, bool& outCreated)
{
    outCreated = false;

    const uint32_t existing = m_grid->FindNodeInColumn(gx, gy, feet.z, kNodeMergeTolerance);
    if (existing != kInvalidNodeId)
    {
        return existing;
    }

    if (m_grid->NodeCount() >= m_config.maxNodes)
    {
        m_hitNodeLimit = true;
        return kInvalidNodeId;
    }

    const Vec3 center(m_grid->CellCenter(gx), m_grid->CellCenter(gy), feet.z);

    const NavContents atFeet = m_world->PointContents(Vec3(center.x, center.y, center.z + 2.0f));
    if (atFeet == NavContents::Hazard)
    {
        return kInvalidNodeId;
    }

    const LayerSpec& spec = m_grid->Spec();
    uint8_t variantMask = 0;
    for (int v = 0; v < spec.variantCount; ++v)
    {
        if (HullFits(*m_world, center, spec.variants[v]))
        {
            variantMask |= VariantBit(v);
        }
    }
    if (variantMask == 0)
    {
        return kInvalidNodeId;
    }

    uint8_t flags = 0;
    if (atFeet == NavContents::Water)
    {
        flags |= kNodeWater;
        if (m_world->PointContents(Vec3(center.x, center.y, center.z + kDeepWaterHeight)) == NavContents::Water)
        {
            flags |= kNodeDeepWater;
        }
    }

    outCreated = true;
    return m_grid->AddNode(gx, gy, feet.z, variantMask, flags);
}

bool NavMeshGenerator::AddSeed(const Vec3& position)
{
    if (!m_grid || !m_world)
    {
        return false;
    }

    const LayerSpec& spec = m_grid->Spec();
    const HullIndex hull = spec.variants[0];

    // Точка посева может оказаться чуть в полу или в стене (origin спавна, точка
    // прицела на стене) — ищем рядом место, где хулл помещается.
    static const Vec3 kNudges[] = {
        Vec3(0, 0, 0), Vec3(0, 0, 18), Vec3(0, 0, 36),
        Vec3(16, 0, 18), Vec3(-16, 0, 18), Vec3(0, 16, 18), Vec3(0, -16, 18),
        Vec3(32, 0, 18), Vec3(-32, 0, 18), Vec3(0, 32, 18), Vec3(0, -32, 18),
        Vec3(0, 0, 72),
    };

    Vec3 floor;
    bool haveFloor = false;
    for (const Vec3& nudge : kNudges)
    {
        const Vec3 candidate = position + nudge;
        if (!HullFits(*m_world, candidate, hull))
        {
            continue;
        }
        if (DropToFloor(*m_world, candidate, hull, kSeedDropDistance, m_config.probe, floor))
        {
            haveFloor = true;
            break;
        }
    }
    if (!haveFloor)
    {
        return false;
    }

    if (std::fabs(floor.x) > m_config.maxCoord || std::fabs(floor.y) > m_config.maxCoord ||
        std::fabs(floor.z) > m_config.maxCoord)
    {
        return false;
    }

    int gx = 0;
    int gy = 0;
    m_grid->WorldToCell(floor, gx, gy);

    if (m_grid->FindNodeInColumn(gx, gy, floor.z, m_config.probe.stepHeight) != kInvalidNodeId)
    {
        return true; // здесь уже есть навмеш
    }

    bool any = false;
    // Сначала своя клетка, затем восемь соседних: центр своей клетки может быть в стене.
    for (int i = -1; i < kNavDirCount; ++i)
    {
        const int cx = gx + (i < 0 ? 0 : kNavDirDx[i]);
        const int cy = gy + (i < 0 ? 0 : kNavDirDy[i]);

        const ProbeResult probe = ProbeMove(*m_world, floor, m_grid->CellCenter(cx), m_grid->CellCenter(cy),
                                            hull, m_config.probe);
        if (probe.kind != ProbeKind::Walk)
        {
            continue;
        }

        bool created = false;
        const uint32_t node = FindOrCreateNode(cx, cy, probe.feet, created);
        if (node == kInvalidNodeId)
        {
            continue;
        }
        any = true;
        if (created)
        {
            m_queue.push_back(node);
        }
        if (i < 0)
        {
            break; // своя клетка подошла — соседи раскроются от неё сами
        }
    }

    return any;
}

void NavMeshGenerator::ExpandNode(uint32_t nodeId)
{
    const LayerSpec& spec = m_grid->Spec();

    // Копии: AddNode ниже может перевыделить массив узлов.
    const Vec3 from = m_grid->Node(nodeId).pos;
    const int gx = m_grid->Node(nodeId).gx;
    const int gy = m_grid->Node(nodeId).gy;
    const uint8_t fromMask = m_grid->Node(nodeId).variantMask;

    for (int dir = 0; dir < kNavDirCount; ++dir)
    {
        const int tx = gx + kNavDirDx[dir];
        const int ty = gy + kNavDirDy[dir];
        const float targetX = m_grid->CellCenter(tx);
        const float targetY = m_grid->CellCenter(ty);

        if (std::fabs(targetX) > m_config.maxCoord || std::fabs(targetY) > m_config.maxCoord)
        {
            continue;
        }

        // От высокого варианта к низкому: где прошёл высокий хулл, пройдёт и низкий
        // того же основания — второй раз трассировать не нужно.
        for (int v = spec.variantCount - 1; v >= 0; --v)
        {
            if ((fromMask & VariantBit(v)) == 0)
            {
                continue;
            }

            const ProbeResult probe = ProbeMove(*m_world, from, targetX, targetY, spec.variants[v], m_config.probe);
            if (!probe.Reached())
            {
                continue;
            }
            if (std::fabs(probe.feet.z) > m_config.maxCoord)
            {
                break;
            }

            bool created = false;
            const uint32_t target = FindOrCreateNode(tx, ty, probe.feet, created);
            if (target == kInvalidNodeId)
            {
                break;
            }

            uint8_t passMask = 0;
            for (int lower = 0; lower <= v; ++lower)
            {
                passMask |= VariantBit(lower);
            }
            passMask &= fromMask;
            passMask &= m_grid->Node(target).variantMask;

            m_grid->SetEdge(nodeId, dir, target, passMask,
                            probe.kind == ProbeKind::Drop ? static_cast<uint8_t>(kEdgeDrop)
                                                          : static_cast<uint8_t>(kEdgeWalk));
            if (created)
            {
                m_queue.push_back(target);
            }
            break;
        }
    }
}

void NavMeshGenerator::Finish()
{
    m_grid->ComputeClearance();
    m_clearanceDirty = false;
}

bool NavMeshGenerator::Tick(double budgetMs)
{
    if (!m_grid)
    {
        return true;
    }

    const double startTime = PlatformNowMs();

    while (!m_queue.empty())
    {
        const uint32_t nodeId = m_queue.front();
        m_queue.pop_front();

        if (!m_grid->Node(nodeId).expanded)
        {
            ExpandNode(nodeId);
            m_grid->Node(nodeId).expanded = true;
            ++m_expandedCount;
            m_clearanceDirty = true;
        }

        if (budgetMs >= 0.0 && PlatformNowMs() - startTime >= budgetMs)
        {
            break;
        }
    }

    if (m_queue.empty() && m_clearanceDirty)
    {
        Finish();
    }

    return IsIdle();
}

void NavMeshGenerator::RunToCompletion()
{
    Tick(-1.0);
}

} // namespace goldnav
