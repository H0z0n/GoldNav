#include "micro_test.h"
#include "fixtures/nav_sim.h"

using namespace goldnav;
using namespace goldnav_test;

static const uint8_t kDuckBit = VariantBit(VariantOfHull(HullIndex::Ducking));
static const uint8_t kStandBit = VariantBit(VariantOfHull(HullIndex::Standing));

static int CountNodesAt(const NavGrid& grid, float x, float y)
{
    int gx = 0, gy = 0;
    grid.WorldToCell(Vec3(x, y, 0), gx, gy);
    int count = 0;
    for (uint32_t id = grid.ColumnHead(gx, gy); id != kInvalidNodeId; id = grid.Node(id).nextInColumn)
    {
        ++count;
    }
    return count;
}

static void Test_FlatRoom_FillsExactlyTheStandableArea()
{
    BoxWorld world;
    AddRoom(world, 256.0f);

    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(0, 0, 10) });

    // Хулл 32x32 встаёт не ближе 16 юнитов к стене: центры клеток от -232 до 232
    // с шагом 16 — по 30 на ось.
    GN_CHECK(grid.NodeCount() == 900);

    // Узла вплотную к стене быть не должно, а в середине — должен.
    GN_CHECK(grid.FindNearestNode(Vec3(-248, 0, 0), 1, 4.0f) == kInvalidNodeId);
    const uint32_t center = grid.FindNearestNode(Vec3(8, 8, 0), 1, 4.0f);
    GN_CHECK(center != kInvalidNodeId);
    if (center != kInvalidNodeId)
    {
        const NavNode& node = grid.Node(center);
        GN_CHECK((node.variantMask & kStandBit) != 0);
        GN_CHECK((node.variantMask & kDuckBit) != 0);
        GN_CHECK(std::fabs(node.pos.z) < 0.1f);
        for (int dir = 0; dir < kNavDirCount; ++dir)
        {
            GN_CHECK(node.edgeType[dir] == kEdgeWalk);
        }
        GN_CHECK(node.wallDist[1] == kMaxWallDist);
    }

    // У стены wallDist = 0, на клетку дальше — 1.
    const uint32_t atWall = grid.FindNearestNode(Vec3(-232, 8, 0), 1, 4.0f);
    const uint32_t nextToIt = grid.FindNearestNode(Vec3(-216, 8, 0), 1, 4.0f);
    GN_CHECK(atWall != kInvalidNodeId && grid.Node(atWall).wallDist[1] == 0);
    GN_CHECK(nextToIt != kInvalidNodeId && grid.Node(nextToIt).wallDist[1] == 1);
}

static void Test_Wall_HasNoNodesInsideAndNoEdgesThrough()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    world.AddSolid(Vec3(-8, -256, 0), Vec3(8, 128, 256)); // стена с проходом при y > 128

    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(-128, 0, 10) });

    GN_CHECK(CountNodesAt(grid, 0, 0) == 0);
    GN_CHECK(CountNodesAt(grid, -8, 0) == 0); // хулл с центром в 8 юнитах от стены не помещается

    // Обе стороны стены достижимы через проход.
    GN_CHECK(grid.FindNearestNode(Vec3(128, 0, 0), 1, 16.0f) != kInvalidNodeId);

    // Ни одно ребро не соединяет узлы по разные стороны стены ниже прохода.
    bool crossing = false;
    for (uint32_t id = 0; id < grid.NodeCount(); ++id)
    {
        const NavNode& node = grid.Node(id);
        for (int dir = 0; dir < kNavDirCount; ++dir)
        {
            if (node.edgeType[dir] == kEdgeNone) continue;
            const NavNode& other = grid.Node(node.edgeTarget[dir]);
            if (node.pos.y < 100 && other.pos.y < 100 && (node.pos.x < 0) != (other.pos.x < 0))
            {
                crossing = true;
            }
        }
    }
    GN_CHECK(!crossing);
}

static void Test_Stairs_AreWalkableBothWays()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    // Лестница вдоль X: 8 ступенек по 8 юнитов высотой и 16 глубиной, затем площадка.
    for (int i = 0; i < 8; ++i)
    {
        world.AddSolid(Vec3(16.0f * i, -256, 0), Vec3(256, 256, 8.0f * (i + 1)));
    }

    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(-128, 0, 10) });

    const uint32_t top = grid.FindNearestNode(Vec3(200, 0, 64), 1, 16.0f);
    GN_CHECK(top != kInvalidNodeId);

    // Вся лестница — Walk-рёбра в обе стороны, ни одного Drop.
    int drops = 0;
    for (uint32_t id = 0; id < grid.NodeCount(); ++id)
    {
        for (int dir = 0; dir < kNavDirCount; ++dir)
        {
            if (grid.Node(id).edgeType[dir] == kEdgeDrop) ++drops;
        }
    }
    GN_CHECK(drops == 0);

    NavPathPlanner planner;
    const PlanResult up = planner.Plan(grid, &world, Vec3(-128, 0, 0), Vec3(200, 0, 64), HullIndex::Standing, PlanConfig());
    GN_CHECK(up.status == PlanStatus::Found);
    const PlanResult down = planner.Plan(grid, &world, Vec3(200, 0, 64), Vec3(-128, 0, 0), HullIndex::Standing, PlanConfig());
    GN_CHECK(down.status == PlanStatus::Found);
}

static void Test_SteepStairs_SeveralStepsPerCell()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    // Крутая лестница: ступенька 16 высотой на 8 глубиной — две ступеньки на клетку.
    // Одним шагом на 16 юнитов её не взять (подъём 32 > 18), подшагами — можно.
    for (int i = 0; i < 6; ++i)
    {
        world.AddSolid(Vec3(8.0f * i, -256, 0), Vec3(256, 256, 16.0f * (i + 1)));
    }

    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(-128, 0, 10) });

    NavPathPlanner planner;
    const PlanResult up = planner.Plan(grid, &world, Vec3(-128, 0, 0), Vec3(200, 0, 96), HullIndex::Standing, PlanConfig());
    GN_CHECK(up.status == PlanStatus::Found);
}

static void Test_StepHeightLimit_MatchesTheMover()
{
    // Ступенька ровно 18 — берётся; 19 — уже нет (и генератором, и "ходоком" одинаково).
    for (int height = 18; height <= 19; ++height)
    {
        BoxWorld world;
        AddRoom(world, 256.0f);
        world.AddSolid(Vec3(0, -256, 0), Vec3(256, 256, static_cast<float>(height)));

        NavGrid grid;
        BuildNav(world, grid, HullIndex::Standing, { Vec3(-128, 0, 10) });
        const bool navClimbs = grid.FindNearestNode(Vec3(128, 0, static_cast<float>(height)), 1, 16.0f) != kInvalidNodeId;

        BoxMoveTracer tracer(world, HullIndex::Standing);
        MoverState state;
        MoverConfig config;
        Vec3 feet(-100, 0, 0.03125f);
        for (int i = 0; i < 100; ++i)
        {
            feet = GroundMove(tracer, feet, Vec3(200, 0, 0), 0.02f, state, config).feet;
        }
        const bool moverClimbs = feet.x > 50.0f;

        GN_CHECK(navClimbs == (height == 18));
        GN_CHECK(moverClimbs == navClimbs);
    }
}

static void Test_Ledge_DropIsOneWay()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    world.AddSolid(Vec3(0, -256, 0), Vec3(256, 256, 64)); // платформа высотой 64

    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(128, 0, 70) });

    // Заливка с платформы спрыгнула вниз и раскрыла нижний пол.
    const uint32_t lower = grid.FindNearestNode(Vec3(-128, 0, 0), 1, 16.0f);
    const uint32_t upper = grid.FindNearestNode(Vec3(128, 0, 64), 1, 16.0f);
    GN_CHECK(lower != kInvalidNodeId);
    GN_CHECK(upper != kInvalidNodeId);

    int drops = 0;
    bool anyUpEdge = false;
    for (uint32_t id = 0; id < grid.NodeCount(); ++id)
    {
        const NavNode& node = grid.Node(id);
        for (int dir = 0; dir < kNavDirCount; ++dir)
        {
            if (node.edgeType[dir] == kEdgeNone) continue;
            const NavNode& other = grid.Node(node.edgeTarget[dir]);
            if (node.edgeType[dir] == kEdgeDrop)
            {
                ++drops;
                GN_CHECK(other.pos.z < node.pos.z - 18.0f);
            }
            if (other.pos.z > node.pos.z + 18.0f) anyUpEdge = true;
        }
    }
    GN_CHECK(drops > 0);
    GN_CHECK(!anyUpEdge);

    NavPathPlanner planner;
    const PlanResult down = planner.Plan(grid, &world, Vec3(128, 0, 64), Vec3(-128, 0, 0), HullIndex::Standing, PlanConfig());
    GN_CHECK(down.status == PlanStatus::Found);

    // Наверх пути нет — получаем "подойти как можно ближе", а не отказ.
    const PlanResult up = planner.Plan(grid, &world, Vec3(-128, 0, 0), Vec3(128, 0, 64), HullIndex::Standing, PlanConfig());
    GN_CHECK(up.status == PlanStatus::Partial);
}

static void Test_HighLedge_IsNotADrop()
{
    BoxWorld world;
    AddRoom(world, 256.0f, 512.0f);
    world.AddSolid(Vec3(0, -256, 0), Vec3(256, 256, 300)); // выше maxDrop (150)

    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(128, 0, 310) });

    GN_CHECK(grid.NodeCount() > 0);
    GN_CHECK(grid.FindNearestNode(Vec3(-128, 0, 0), 1, 16.0f) == kInvalidNodeId);
}

static void Test_TwoFloors_SameColumnHasTwoNodes()
{
    BoxWorld world;
    AddRoom(world, 256.0f, 512.0f);
    world.AddSolid(Vec3(-64, -256, 120), Vec3(64, 256, 128)); // мост над полом

    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(0, 0, 10), Vec3(0, 0, 140) });

    GN_CHECK(CountNodesAt(grid, 8, 8) == 2);

    const uint32_t below = grid.FindNearestNode(Vec3(8, 8, 0), 1, 16.0f);
    const uint32_t above = grid.FindNearestNode(Vec3(8, 8, 128), 1, 16.0f);
    GN_CHECK(below != kInvalidNodeId && above != kInvalidNodeId && below != above);
    if (below != kInvalidNodeId && above != kInvalidNodeId)
    {
        GN_CHECK(std::fabs(grid.Node(below).pos.z) < 0.1f);
        GN_CHECK(std::fabs(grid.Node(above).pos.z - 128.0f) < 0.1f);
        // Под мостом 120 юнитов высоты — стоячий хулл (72) помещается.
        GN_CHECK((grid.Node(below).variantMask & kStandBit) != 0);
    }
}

static void Test_LowCeiling_OnlyDuckingPasses()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    // Перегородка от стены до стены с низким лазом (50 юнитов) на всю ширину.
    world.AddSolid(Vec3(-32, -256, 50), Vec3(32, 256, 256));

    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(-128, 0, 10) });

    const uint32_t under = grid.FindNearestNode(Vec3(8, 8, 0), 0, 16.0f);
    GN_CHECK(under != kInvalidNodeId);
    if (under != kInvalidNodeId)
    {
        GN_CHECK((grid.Node(under).variantMask & kDuckBit) != 0);
        GN_CHECK((grid.Node(under).variantMask & kStandBit) == 0);
    }

    // Дальняя сторона раскрыта (через лаз прошёл приседающий хулл), и там снова можно стоять.
    const uint32_t far = grid.FindNearestNode(Vec3(128, 8, 0), 1, 16.0f);
    GN_CHECK(far != kInvalidNodeId);

    NavPathPlanner planner;
    const PlanResult standing = planner.Plan(grid, &world, Vec3(-128, 0, 0), Vec3(128, 0, 0), HullIndex::Standing, PlanConfig());
    GN_CHECK(standing.status == PlanStatus::Partial);
    const PlanResult ducking = planner.Plan(grid, &world, Vec3(-128, 0, 0), Vec3(128, 0, 0), HullIndex::Ducking, PlanConfig());
    GN_CHECK(ducking.status == PlanStatus::Found);
}

static void Test_NarrowDoor_IsFound()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    // Стена с дверью шириной 48 (y от 3 до 51 — нарочно не по сетке).
    world.AddSolid(Vec3(-8, -256, 0), Vec3(8, 3, 256));
    world.AddSolid(Vec3(-8, 51, 0), Vec3(8, 256, 256));

    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(-128, 0, 10) });

    GN_CHECK(grid.FindNearestNode(Vec3(128, 0, 0), 1, 16.0f) != kInvalidNodeId);
}

static void Test_Hazard_IsNotWalkable()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    world.AddVolume(Vec3(-32, -256, 0), Vec3(32, 256, 32), NavContents::Hazard); // полоса лавы
    world.AddVolume(Vec3(100, -256, 0), Vec3(160, 256, 16), NavContents::Water); // лужа

    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(-128, 0, 10), Vec3(200, 0, 10) });

    GN_CHECK(CountNodesAt(grid, 8, 8) == 0);
    const uint32_t wet = grid.FindNearestNode(Vec3(120, 8, 0), 1, 16.0f);
    GN_CHECK(wet != kInvalidNodeId && (grid.Node(wet).flags & kNodeWater) != 0);
}

static void Test_IncrementalTick_MatchesFullRun()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    world.AddSolid(Vec3(-8, -256, 0), Vec3(8, 128, 256));

    NavGrid full;
    BuildNav(world, full, HullIndex::Standing, { Vec3(-128, 0, 10) });

    NavGrid stepped;
    NavMeshGenerator generator;
    generator.Begin(stepped, world, NavLayer::Medium, NavGenConfig());
    GN_CHECK(generator.AddSeed(Vec3(-128, 0, 10)));
    int ticks = 0;
    while (!generator.Tick(0.0) && ticks < 1000000)
    {
        ++ticks; // бюджет 0 мс — по одному узлу за тик
    }
    GN_CHECK(generator.IsIdle());
    GN_CHECK(ticks > 10);
    GN_CHECK(stepped.NodeCount() == full.NodeCount());
}

static void Test_LateSeed_GrowsGraphIntoIsolatedArea()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    world.AddSolid(Vec3(-8, -256, 0), Vec3(8, 256, 256)); // глухая стена — две изолированные половины

    NavGrid grid;
    NavMeshGenerator generator;
    generator.Begin(grid, world, NavLayer::Medium, NavGenConfig());
    generator.AddSeed(Vec3(-128, 0, 10));
    generator.RunToCompletion();

    const size_t before = grid.NodeCount();
    GN_CHECK(grid.FindNearestNode(Vec3(128, 0, 0), 1, 16.0f) == kInvalidNodeId);

    GN_CHECK(generator.AddSeed(Vec3(128, 0, 10)));
    GN_CHECK(!generator.IsIdle());
    generator.RunToCompletion();

    GN_CHECK(grid.NodeCount() == before * 2);
    GN_CHECK(grid.FindNearestNode(Vec3(128, 0, 0), 1, 16.0f) != kInvalidNodeId);

    // Повторный посев туда, где навмеш уже есть, работы не добавляет.
    GN_CHECK(generator.AddSeed(Vec3(100, 20, 10)));
    GN_CHECK(generator.IsIdle());
}

static void Test_SeedInSolid_Fails()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    world.AddSolid(Vec3(-100, -100, 0), Vec3(100, 100, 256));

    NavGrid grid;
    NavMeshGenerator generator;
    generator.Begin(grid, world, NavLayer::Medium, NavGenConfig());
    GN_CHECK(!generator.AddSeed(Vec3(0, 0, 100)));
}

static void Test_PointAndLargeLayers()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    // Щель шириной 20: точка проходит, хулл 32 — нет.
    world.AddSolid(Vec3(-8, -256, 0), Vec3(8, -10, 256));
    world.AddSolid(Vec3(-8, 10, 0), Vec3(8, 256, 256));

    NavGrid pointGrid;
    BuildNav(world, pointGrid, HullIndex::Point, { Vec3(-128, 0, 10) });
    GN_CHECK(pointGrid.FindNearestNode(Vec3(128, 0, 0), 0, 16.0f) != kInvalidNodeId);

    NavGrid mediumGrid;
    BuildNav(world, mediumGrid, HullIndex::Standing, { Vec3(-128, 0, 10) });
    GN_CHECK(mediumGrid.FindNearestNode(Vec3(128, 0, 0), 1, 16.0f) == kInvalidNodeId);

    BoxWorld open;
    AddRoom(open, 256.0f);
    NavGrid largeGrid;
    BuildNav(open, largeGrid, HullIndex::Large, { Vec3(0, 0, 10) });
    // Хулл 64x64 не ближе 32 к стене: центры от -216 до 216 — по 28 на ось.
    GN_CHECK(largeGrid.NodeCount() == 28 * 28);
}

static void RunAllTests()
{
    GN_RUN(Test_FlatRoom_FillsExactlyTheStandableArea);
    GN_RUN(Test_Wall_HasNoNodesInsideAndNoEdgesThrough);
    GN_RUN(Test_Stairs_AreWalkableBothWays);
    GN_RUN(Test_SteepStairs_SeveralStepsPerCell);
    GN_RUN(Test_StepHeightLimit_MatchesTheMover);
    GN_RUN(Test_Ledge_DropIsOneWay);
    GN_RUN(Test_HighLedge_IsNotADrop);
    GN_RUN(Test_TwoFloors_SameColumnHasTwoNodes);
    GN_RUN(Test_LowCeiling_OnlyDuckingPasses);
    GN_RUN(Test_NarrowDoor_IsFound);
    GN_RUN(Test_Hazard_IsNotWalkable);
    GN_RUN(Test_IncrementalTick_MatchesFullRun);
    GN_RUN(Test_LateSeed_GrowsGraphIntoIsolatedArea);
    GN_RUN(Test_SeedInSolid_Fails);
    GN_RUN(Test_PointAndLargeLayers);
}

GN_TEST_MAIN()
