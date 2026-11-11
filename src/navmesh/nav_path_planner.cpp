#include "nav_path_planner.h"

#include <algorithm>
#include <cmath>
#include "pathfinder/path_smoother.h"

namespace goldnav {

namespace {
constexpr float kNodeMatchTolerance = 9.0f;
constexpr size_t kStartCandidates = 12;
} // namespace

uint32_t NavPathPlanner::FindStartNode(const NavGrid& grid, const ICollisionWorld* world, const Vec3& fromFeet,
                                       HullIndex hull, const PlanConfig& config)
{
    const int variant = VariantOfHull(hull);

    // Entity может висеть в воздухе или стоять на ступеньке — берём пол под ней.
    Vec3 floor = fromFeet;
    if (world)
    {
        Vec3 dropped;
        if (DropToFloor(*world, fromFeet, hull, 256.0f, config.probe, dropped))
        {
            floor = dropped;
        }
    }

    grid.FindNearbyNodes(floor, variant, config.startSearchRadius, 2.0f, kStartCandidates, m_candidates);
    if (m_candidates.empty())
    {
        return kInvalidNodeId;
    }
    if (!world)
    {
        return m_candidates.front();
    }

    for (uint32_t id : m_candidates)
    {
        const NavNode& node = grid.Node(id);
        const ProbeResult probe = ProbeMove(*world, floor, node.pos.x, node.pos.y, hull, config.probe);
        if (probe.Reached() && std::fabs(probe.feet.z - node.pos.z) <= kNodeMatchTolerance)
        {
            return id;
        }
    }

    // Ни до одного не дошли пробником (например, entity слегка застряла в геометрии).
    // Узел в своей же клетке на своей высоте всё равно годится: "ходок" сам вытолкнет.
    const NavNode& nearest = grid.Node(m_candidates.front());
    if (Distance2D(nearest.pos, floor) <= grid.CellSize() &&
        std::fabs(nearest.pos.z - floor.z) <= config.probe.stepHeight)
    {
        return m_candidates.front();
    }
    return kInvalidNodeId;
}

PlanResult NavPathPlanner::Plan(const NavGrid& grid, const ICollisionWorld* world, const Vec3& fromFeet,
                                const Vec3& to, HullIndex hull, const PlanConfig& config)
{
    PlanResult result;

    if (grid.NodeCount() == 0 || LayerOfHull(hull) != grid.Layer())
    {
        result.status = PlanStatus::NoStartNode;
        return result;
    }

    const int variant = VariantOfHull(hull);

    result.startNode = FindStartNode(grid, world, fromFeet, hull, config);
    if (result.startNode == kInvalidNodeId)
    {
        result.status = PlanStatus::NoStartNode;
        return result;
    }

    result.goalNode = grid.FindNearestNode(to, variant, config.goalSearchRadius, 1.5f);
    if (result.goalNode == kInvalidNodeId)
    {
        result.status = PlanStatus::NoGoalNode;
        return result;
    }

    const NavGraphView view(grid, variant, config.cost);
    AStarOptions options;
    options.allowPartial = config.allowPartial;
    options.maxExpansions = config.maxExpansions;

    const AStarResult search = m_astar.Search(view, result.startNode, result.goalNode, m_nodePath, options);
    if (search == AStarResult::NotFound || m_nodePath.empty())
    {
        result.status = PlanStatus::NoPath;
        return result;
    }

    result.status = (search == AStarResult::Found) ? PlanStatus::Found : PlanStatus::Partial;
    result.endNode = m_nodePath.back();

    // --- Сглаживание ---
    // Срезанный отрезок не должен подходить к стенам ближе, чем это делал сырой путь
    // на том же участке, но и не обязан держаться дальше smoothWallDist: на открытом
    // месте путь идёт с запасом от углов, в узком коридоре — как есть.
    const std::vector<uint32_t>& nodes = m_nodePath;
    const DirectPathPredicate canGoDirect = [&](size_t from, size_t toIndex) {
        uint8_t rawMin = kMaxWallDist;
        uint8_t rawFlags = grid.Node(nodes[toIndex]).flags;
        for (size_t i = from + 1; i < toIndex; ++i)
        {
            const NavNode& node = grid.Node(nodes[i]);
            rawMin = std::min(rawMin, node.wallDist[variant]);
            rawFlags |= node.flags;
        }
        const uint8_t required = std::max<uint8_t>(1, std::min(rawMin, config.smoothWallDist));
        // A* обошёл воду — значит и срезать через неё нельзя. Если же сырой путь сам
        // идёт по воде (иначе никак), запрещать её при срезании бессмысленно.
        const uint8_t avoid = static_cast<uint8_t>((kNodeWater | kNodeDeepWater) & ~rawFlags);
        return grid.LineOfSight(nodes[from], nodes[toIndex], variant, required, avoid);
    };

    const std::vector<size_t> kept = SmoothPathIndices(nodes.size(), canGoDirect, config.smoothLookahead);

    result.points.reserve(kept.size() + 1);
    for (size_t i = 0; i < kept.size(); ++i)
    {
        // Стартовый узел пропускаем: entity уже рядом с ним, и возврат к центру
        // клетки выглядел бы как рывок назад. Оставляем его, только если он вплотную
        // к стене — тогда срезать мимо него рискованно.
        if (i == 0 && kept.size() > 1 && grid.Node(nodes[0]).wallDist[variant] >= 1)
        {
            continue;
        }
        result.points.push_back(grid.Node(nodes[kept[i]]).pos);
    }

    // --- Точная конечная точка ---
    if (result.status == PlanStatus::Found && world)
    {
        const NavNode& goal = grid.Node(result.goalNode);
        if (Distance2D(goal.pos, to) <= grid.CellSize() * 1.5f)
        {
            const ProbeResult probe = ProbeMove(*world, goal.pos, to.x, to.y, hull, config.probe);
            if (probe.kind == ProbeKind::Walk && Distance2D(probe.feet, goal.pos) > 1.0f)
            {
                if (result.points.size() == 1 && result.startNode == result.goalNode)
                {
                    result.points.clear(); // мы уже в клетке цели — идём сразу в точку
                }
                result.points.push_back(probe.feet);
            }
        }
    }

    if (result.points.empty())
    {
        result.points.push_back(grid.Node(result.endNode).pos);
    }

    return result;
}

} // namespace goldnav
