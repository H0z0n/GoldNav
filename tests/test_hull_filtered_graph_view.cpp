#include "micro_test.h"
#include "navmesh/nav_grid.h"
#include "navmesh/hull_filtered_graph_view.h"
#include "pathfinder/astar.h"

using namespace goldnav;

static const uint8_t kDuck = VariantBit(0);
static const uint8_t kBoth = VariantBit(0) | VariantBit(1);

static void Link(NavGrid& grid, uint32_t a, uint32_t b, uint8_t mask, uint8_t type = kEdgeWalk, bool bothWays = true)
{
    const int dir = NavDirFromDelta(grid.Node(b).gx - grid.Node(a).gx, grid.Node(b).gy - grid.Node(a).gy);
    grid.SetEdge(a, dir, b, mask, type);
    if (bothWays)
    {
        grid.SetEdge(b, NavOppositeDir(dir), a, mask, type);
    }
}

// Коридор из трёх клеток, средняя — с низким потолком (только пригнувшись).
static NavGrid BuildCorridor()
{
    NavGrid grid;
    grid.Initialize(NavLayer::Medium, 16.0f);
    const uint32_t a = grid.AddNode(0, 0, 0.0f, kBoth, 0);
    const uint32_t b = grid.AddNode(1, 0, 0.0f, kDuck, 0);
    const uint32_t c = grid.AddNode(2, 0, 0.0f, kBoth, 0);
    Link(grid, a, b, kDuck);
    Link(grid, b, c, kDuck);
    grid.ComputeClearance();
    return grid;
}

static void Test_StandingCannotPassLowCeiling()
{
    const NavGrid grid = BuildCorridor();
    const NavGraphView standing(grid, 1);

    NeighborList neighbors;
    standing.GetNeighbors(0, neighbors);
    GN_CHECK(neighbors.count == 0);

    AStarPathfinder astar;
    std::vector<uint32_t> path;
    GN_CHECK(!astar.FindPath(standing, 0, 2, path));
}

static void Test_DuckingPassesLowCeiling()
{
    const NavGrid grid = BuildCorridor();
    const NavGraphView ducking(grid, 0);

    AStarPathfinder astar;
    std::vector<uint32_t> path;
    GN_CHECK(astar.FindPath(ducking, 0, 2, path));
    GN_CHECK(path.size() == 3);
}

static void Test_CostsAreNeverBelowDistance()
{
    const NavGrid grid = BuildCorridor();
    const NavGraphView ducking(grid, 0);

    NeighborList neighbors;
    ducking.GetNeighbors(1, neighbors);
    GN_CHECK(neighbors.count == 2);
    for (int i = 0; i < neighbors.count; ++i)
    {
        // Допустимость евклидовой эвристики A*.
        GN_CHECK(neighbors.items[i].cost >= 16.0f);
    }
}

static void Test_DropEdges_AreOneWayAndCanBeDisabled()
{
    NavGrid grid;
    grid.Initialize(NavLayer::Medium, 16.0f);
    const uint32_t top = grid.AddNode(0, 0, 64.0f, kBoth, 0);
    const uint32_t bottom = grid.AddNode(1, 0, 0.0f, kBoth, 0);
    Link(grid, top, bottom, kBoth, kEdgeDrop, false);
    grid.ComputeClearance();

    AStarPathfinder astar;
    std::vector<uint32_t> path;

    const NavGraphView view(grid, 1);
    GN_CHECK(astar.FindPath(view, top, bottom, path));
    GN_CHECK(!astar.FindPath(view, bottom, top, path));

    NavCostConfig noDrops;
    noDrops.allowDrops = false;
    const NavGraphView careful(grid, 1, noDrops);
    GN_CHECK(!astar.FindPath(careful, top, bottom, path));
}

static void Test_WaterIsMoreExpensive()
{
    NavGrid grid;
    grid.Initialize(NavLayer::Medium, 16.0f);
    const uint32_t a = grid.AddNode(0, 0, 0.0f, kBoth, 0);
    const uint32_t dry = grid.AddNode(1, 0, 0.0f, kBoth, 0);
    const uint32_t wet = grid.AddNode(-1, 0, 0.0f, kBoth, kNodeWater);
    Link(grid, a, dry, kBoth);
    Link(grid, a, wet, kBoth);
    grid.ComputeClearance();

    const NavGraphView view(grid, 1);
    NeighborList neighbors;
    view.GetNeighbors(a, neighbors);
    GN_CHECK(neighbors.count == 2);

    float dryCost = 0.0f, wetCost = 0.0f;
    for (int i = 0; i < neighbors.count; ++i)
    {
        if (neighbors.items[i].nodeId == dry) dryCost = neighbors.items[i].cost;
        if (neighbors.items[i].nodeId == wet) wetCost = neighbors.items[i].cost;
    }
    GN_CHECK(wetCost > dryCost);
}

static void RunAllTests()
{
    GN_RUN(Test_StandingCannotPassLowCeiling);
    GN_RUN(Test_DuckingPassesLowCeiling);
    GN_RUN(Test_CostsAreNeverBelowDistance);
    GN_RUN(Test_DropEdges_AreOneWayAndCanBeDisabled);
    GN_RUN(Test_WaterIsMoreExpensive);
}

GN_TEST_MAIN()
