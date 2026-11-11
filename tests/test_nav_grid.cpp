#include "micro_test.h"
#include "navmesh/nav_grid.h"

using namespace goldnav;

static const uint8_t kBoth = VariantBit(0) | VariantBit(1);

// Ровная площадка width x height клеток (cellSize 16), 8-связная, оба варианта.
// blocked(gx, gy) == true — клетки нет.
template <typename TBlocked>
static NavGrid BuildFlat(int width, int height, TBlocked blocked)
{
    NavGrid grid;
    grid.Initialize(NavLayer::Medium, 16.0f);

    for (int gy = 0; gy < height; ++gy)
        for (int gx = 0; gx < width; ++gx)
            if (!blocked(gx, gy))
                grid.AddNode(gx, gy, 0.0f, kBoth, 0);

    for (uint32_t id = 0; id < grid.NodeCount(); ++id)
    {
        const int gx = grid.Node(id).gx;
        const int gy = grid.Node(id).gy;
        for (int dir = 0; dir < kNavDirCount; ++dir)
        {
            const uint32_t other = grid.FindNodeInColumn(gx + kNavDirDx[dir], gy + kNavDirDy[dir], 0.0f, 1.0f);
            if (other == kInvalidNodeId) continue;
            if (NavDirIsDiagonal(dir))
            {
                // Диагональ не срезает угол: обе смежные клетки должны существовать.
                if (grid.FindNodeInColumn(gx + kNavDirDx[dir], gy, 0.0f, 1.0f) == kInvalidNodeId) continue;
                if (grid.FindNodeInColumn(gx, gy + kNavDirDy[dir], 0.0f, 1.0f) == kInvalidNodeId) continue;
            }
            grid.SetEdge(id, dir, other, kBoth, kEdgeWalk);
        }
    }

    grid.ComputeClearance();
    return grid;
}

static uint32_t At(const NavGrid& grid, int gx, int gy)
{
    return grid.FindNodeInColumn(gx, gy, 0.0f, 1.0f);
}

static void Test_Directions()
{
    for (int dir = 0; dir < kNavDirCount; ++dir)
    {
        GN_CHECK(NavDirFromDelta(kNavDirDx[dir], kNavDirDy[dir]) == dir);
        const int opposite = NavOppositeDir(dir);
        GN_CHECK(kNavDirDx[opposite] == -kNavDirDx[dir] && kNavDirDy[opposite] == -kNavDirDy[dir]);
    }
    GN_CHECK(NavDirFromDelta(0, 0) == -1);
}

static void Test_CellMapping_NegativeCoordinates()
{
    NavGrid grid;
    grid.Initialize(NavLayer::Medium, 16.0f);
    int gx = 0, gy = 0;
    grid.WorldToCell(Vec3(-0.5f, 15.9f, 0), gx, gy);
    GN_CHECK(gx == -1 && gy == 0);
    grid.WorldToCell(Vec3(-16.0f, -16.1f, 0), gx, gy);
    GN_CHECK(gx == -1 && gy == -2);
    GN_CHECK(grid.CellCenter(-1) == -8.0f);
    GN_CHECK(grid.CellCenter(0) == 8.0f);
}

static void Test_Columns_HoldSeveralFloors()
{
    NavGrid grid;
    grid.Initialize(NavLayer::Medium, 16.0f);
    const uint32_t low = grid.AddNode(-3, 5, 0.0f, kBoth, 0);
    const uint32_t high = grid.AddNode(-3, 5, 128.0f, kBoth, 0);
    grid.AddNode(-3, 6, 64.0f, kBoth, 0);

    GN_CHECK(grid.NodeCount() == 3);
    GN_CHECK(grid.ColumnCount() == 2);
    GN_CHECK(grid.FindNodeInColumn(-3, 5, 4.0f, 9.0f) == low);
    GN_CHECK(grid.FindNodeInColumn(-3, 5, 120.0f, 9.0f) == high);
    GN_CHECK(grid.FindNodeInColumn(-3, 5, 64.0f, 9.0f) == kInvalidNodeId);
    GN_CHECK(grid.FindNodeInColumn(0, 0, 0.0f, 9.0f) == kInvalidNodeId);

    // Ближайший узел учитывает высоту: с первого этажа не "прыгаем" на второй.
    GN_CHECK(grid.FindNearestNode(Vec3(-40, 88, 10), 1, 64.0f) == low);
    GN_CHECK(grid.FindNearestNode(Vec3(-40, 88, 120), 1, 64.0f) == high);
}

static void Test_FindNearest_RespectsVariantAndRadius()
{
    NavGrid grid;
    grid.Initialize(NavLayer::Medium, 16.0f);
    const uint32_t duckOnly = grid.AddNode(0, 0, 0.0f, VariantBit(0), 0);
    const uint32_t both = grid.AddNode(5, 0, 0.0f, kBoth, 0);

    GN_CHECK(grid.FindNearestNode(Vec3(8, 8, 0), 0, 200.0f) == duckOnly);
    GN_CHECK(grid.FindNearestNode(Vec3(8, 8, 0), 1, 200.0f) == both);
    GN_CHECK(grid.FindNearestNode(Vec3(8, 8, 0), 1, 40.0f) == kInvalidNodeId);

    std::vector<uint32_t> nearby;
    grid.FindNearbyNodes(Vec3(8, 8, 0), 0, 200.0f, 1.0f, 8, nearby);
    GN_CHECK(nearby.size() == 2);
    GN_CHECK(nearby.size() == 2 && nearby[0] == duckOnly && nearby[1] == both);
}

static void Test_FindNearest_EmptyGrid()
{
    NavGrid grid;
    grid.Initialize(NavLayer::Medium, 16.0f);
    GN_CHECK(grid.FindNearestNode(Vec3(0, 0, 0), 1, 1000.0f) == kInvalidNodeId);
}

static void Test_Clearance()
{
    const NavGrid grid = BuildFlat(9, 9, [](int, int) { return false; });

    GN_CHECK(grid.Node(At(grid, 0, 4)).wallDist[1] == 0);
    GN_CHECK(grid.Node(At(grid, 1, 4)).wallDist[1] == 1);
    GN_CHECK(grid.Node(At(grid, 2, 4)).wallDist[1] == 2);
    GN_CHECK(grid.Node(At(grid, 3, 4)).wallDist[1] == 3);
    GN_CHECK(grid.Node(At(grid, 4, 4)).wallDist[1] == kMaxWallDist);
}

static void Test_LineOfSight_OpenAndBlocked()
{
    // Столб 1x1 в центре площадки 11x11.
    const NavGrid grid = BuildFlat(11, 11, [](int gx, int gy) { return gx == 5 && gy == 5; });

    // Прямая через столб — нельзя.
    GN_CHECK(!grid.LineOfSight(At(grid, 1, 5), At(grid, 9, 5), 1, 1));
    GN_CHECK(!grid.LineOfSight(At(grid, 2, 2), At(grid, 8, 8), 1, 1));

    // Прямая вдали от столба — можно.
    GN_CHECK(grid.LineOfSight(At(grid, 3, 8), At(grid, 8, 9), 1, 1));

    // Косая линия вплотную к столбу (через клетки с wallDist 0) — нельзя при
    // требовании отступа, хотя сами клетки проходимы.
    GN_CHECK(!grid.LineOfSight(At(grid, 2, 3), At(grid, 8, 4), 1, 1));
    GN_CHECK(grid.LineOfSight(At(grid, 2, 3), At(grid, 8, 4), 1, 0));

    // Осевая линия вдоль края площадки идёт ровно по рёбрам — отступ для неё не нужен.
    GN_CHECK(grid.LineOfSight(At(grid, 0, 0), At(grid, 10, 0), 1, 3));
    // Как и точная диагональ.
    GN_CHECK(grid.LineOfSight(At(grid, 0, 10), At(grid, 4, 6), 1, 3));

    GN_CHECK(grid.LineOfSight(At(grid, 3, 3), At(grid, 3, 3), 1, 3));
}

static void Test_LineOfSight_DoesNotCrossDropsOrFloors()
{
    NavGrid grid;
    grid.Initialize(NavLayer::Medium, 16.0f);
    const uint32_t a = grid.AddNode(0, 0, 64.0f, kBoth, 0);
    const uint32_t b = grid.AddNode(1, 0, 0.0f, kBoth, 0);
    const uint32_t c = grid.AddNode(2, 0, 0.0f, kBoth, 0);
    const uint32_t cUpper = grid.AddNode(2, 0, 128.0f, kBoth, 0);
    grid.SetEdge(a, 0, b, kBoth, kEdgeDrop);
    grid.SetEdge(b, 0, c, kBoth, kEdgeWalk);
    grid.SetEdge(c, 1, b, kBoth, kEdgeWalk);
    grid.ComputeClearance();

    GN_CHECK(!grid.LineOfSight(a, c, 1, 0));     // через спрыгивание не срезаем
    GN_CHECK(grid.LineOfSight(b, c, 1, 0));
    GN_CHECK(!grid.LineOfSight(b, cUpper, 1, 0)); // та же клетка, но другой этаж
    GN_CHECK(!grid.LineOfSight(c, cUpper, 1, 0));
}

static void Test_LineOfSight_RespectsVariantMask()
{
    NavGrid grid;
    grid.Initialize(NavLayer::Medium, 16.0f);
    const uint32_t a = grid.AddNode(0, 0, 0.0f, kBoth, 0);
    const uint32_t b = grid.AddNode(1, 0, 0.0f, VariantBit(0), 0); // низкий потолок
    const uint32_t c = grid.AddNode(2, 0, 0.0f, kBoth, 0);
    grid.SetEdge(a, 0, b, VariantBit(0), kEdgeWalk);
    grid.SetEdge(b, 0, c, VariantBit(0), kEdgeWalk);
    grid.ComputeClearance();

    GN_CHECK(grid.LineOfSight(a, c, 0, 0));
    GN_CHECK(!grid.LineOfSight(a, c, 1, 0));
}

static void RunAllTests()
{
    GN_RUN(Test_Directions);
    GN_RUN(Test_CellMapping_NegativeCoordinates);
    GN_RUN(Test_Columns_HoldSeveralFloors);
    GN_RUN(Test_FindNearest_RespectsVariantAndRadius);
    GN_RUN(Test_FindNearest_EmptyGrid);
    GN_RUN(Test_Clearance);
    GN_RUN(Test_LineOfSight_OpenAndBlocked);
    GN_RUN(Test_LineOfSight_DoesNotCrossDropsOrFloors);
    GN_RUN(Test_LineOfSight_RespectsVariantMask);
}

GN_TEST_MAIN()
