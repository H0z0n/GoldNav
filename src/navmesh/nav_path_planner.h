// NavPathPlanner — путь "из точки в точку" по готовому NavGrid:
//   1. привязка старта к узлу, до которого от стартовой точки реально можно дойти
//      (а не просто ближайшему — ближайший может оказаться за стеной);
//   2. привязка цели к ближайшему узлу;
//   3. A* с "человеческими" весами (см. NavGraphView); если цель недостижима —
//      путь до ближайшего к ней достижимого места;
//   4. сглаживание по прямой видимости на сетке;
//   5. точная конечная точка, если до неё можно дойти от последнего узла.
// Engine-agnostic: геометрию видит только через ICollisionWorld (может быть nullptr —
// тогда шаги 1 и 5 упрощаются до "ближайший узел").
#pragma once

#include <vector>
#include "nav_grid.h"
#include "nav_probe.h"
#include "hull_filtered_graph_view.h"
#include "collision_world.h"
#include "pathfinder/astar.h"

namespace goldnav {

enum class PlanStatus
{
    Found,       // путь до цели
    Partial,     // цель недостижима, путь ведёт к ближайшей достижимой точке
    NoStartNode, // рядом со стартом нет навмеша
    NoGoalNode,  // рядом с целью нет навмеша
    NoPath,      // идти некуда
};

struct PlanConfig
{
    float startSearchRadius = 72.0f;
    float goalSearchRadius = 1024.0f;
    ProbeConfig probe;
    NavCostConfig cost;
    uint8_t smoothWallDist = 2;   // сколько клеток от стен держит срезанный путь на открытом месте
    size_t smoothLookahead = 48;
    size_t maxExpansions = 0;
    bool allowPartial = true;
};

struct PlanResult
{
    PlanStatus status = PlanStatus::NoPath;
    std::vector<Vec3> points; // feet-позиции, по которым идти (без стартовой)
    uint32_t startNode = kInvalidNodeId;
    uint32_t goalNode = kInvalidNodeId; // узел, к которому привязана цель
    uint32_t endNode = kInvalidNodeId;  // узел, где путь заканчивается на самом деле

    bool HasPath() const { return status == PlanStatus::Found || status == PlanStatus::Partial; }
};

class NavPathPlanner
{
public:
    PlanResult Plan(const NavGrid& grid, const ICollisionWorld* world, const Vec3& fromFeet,
                    const Vec3& to, HullIndex hull, const PlanConfig& config);

    // Узел, до которого от точки можно дойти по прямой; kInvalidNodeId, если такого нет.
    uint32_t FindStartNode(const NavGrid& grid, const ICollisionWorld* world, const Vec3& fromFeet,
                           HullIndex hull, const PlanConfig& config);

    size_t LastExpansions() const { return m_astar.LastExpansions(); }

private:
    AStarPathfinder m_astar;
    std::vector<uint32_t> m_nodePath;
    std::vector<uint32_t> m_candidates;
};

} // namespace goldnav
