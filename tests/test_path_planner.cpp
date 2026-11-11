#include "micro_test.h"
#include "fixtures/nav_sim.h"

using namespace goldnav;
using namespace goldnav_test;

// Проверяет, что по каждому отрезку пути хулл физически проходит (пробником шага).
static bool PathIsPhysicallyWalkable(const BoxWorld& world, const Vec3& from, const std::vector<Vec3>& points,
                                     HullIndex hull)
{
    ProbeConfig probe;
    Vec3 pos = from;
    for (const Vec3& point : points)
    {
        const ProbeResult step = ProbeMove(world, pos, point.x, point.y, hull, probe);
        if (!step.Reached())
        {
            return false;
        }
        pos = step.feet;
    }
    return true;
}

static float PathLength(const Vec3& from, const std::vector<Vec3>& points)
{
    float length = 0.0f;
    Vec3 prev = from;
    for (const Vec3& p : points)
    {
        length += Distance(prev, p);
        prev = p;
    }
    return length;
}

static void Test_OpenRoom_StraightLine()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(0, 0, 10) });

    NavPathPlanner planner;
    const Vec3 from(-200, -150, 0);
    const Vec3 to(180, 90, 0);
    const PlanResult plan = planner.Plan(grid, &world, from, to, HullIndex::Standing, PlanConfig());

    GN_CHECK(plan.status == PlanStatus::Found);
    GN_CHECK(!plan.points.empty());
    // На пустом месте путь почти прямой: не длиннее прямой больше чем на 10%.
    GN_CHECK(PathLength(from, plan.points) <= Distance(from, to) * 1.10f);
    // И состоит из пары точек, а не из сотни клеток.
    GN_CHECK(plan.points.size() <= 4);
    // Заканчивается ровно в запрошенной точке.
    GN_CHECK(Distance2D(plan.points.back(), to) < 0.5f);
    GN_CHECK(PathIsPhysicallyWalkable(world, from, plan.points, HullIndex::Standing));
}

static void Test_AroundWall_GoesThroughGapWithClearance()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    world.AddSolid(Vec3(-8, -256, 0), Vec3(8, 96, 256)); // проход при y от 96 до 256

    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(-128, 0, 10) });

    NavPathPlanner planner;
    const Vec3 from(-128, -100, 0);
    const Vec3 to(128, -100, 0);
    const PlanResult plan = planner.Plan(grid, &world, from, to, HullIndex::Standing, PlanConfig());

    GN_CHECK(plan.status == PlanStatus::Found);
    GN_CHECK(PathIsPhysicallyWalkable(world, from, plan.points, HullIndex::Standing));

    // Путь проходит через проём, а не сквозь стену.
    float maxY = -1e9f;
    for (const Vec3& p : plan.points) maxY = std::max(maxY, p.y);
    GN_CHECK(maxY > 96.0f + 16.0f);

    // Огибает торец стены с запасом, а не впритирку: хулл касался бы угла при y = 112.
    GN_CHECK(maxY >= 96.0f + 16.0f + 16.0f);

    // И при этом не делает крюк через дальнюю стену.
    GN_CHECK(maxY < 200.0f);
}

static void Test_Unreachable_ReturnsPartialNearGoal()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    // Замкнутая будка вокруг цели.
    world.AddSolid(Vec3(100, -60, 0), Vec3(108, 60, 256));
    world.AddSolid(Vec3(200, -60, 0), Vec3(208, 60, 256));
    world.AddSolid(Vec3(100, -60, 0), Vec3(208, -52, 256));
    world.AddSolid(Vec3(100, 52, 0), Vec3(208, 60, 256));

    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(-128, 0, 10), Vec3(154, 0, 10) });

    NavPathPlanner planner;
    const Vec3 from(-128, 0, 0);
    const Vec3 to(154, 0, 0);
    const PlanResult plan = planner.Plan(grid, &world, from, to, HullIndex::Standing, PlanConfig());

    GN_CHECK(plan.status == PlanStatus::Partial);
    GN_CHECK(!plan.points.empty());
    if (!plan.points.empty())
    {
        // Подошли к будке вплотную со стороны старта.
        GN_CHECK(Distance2D(plan.points.back(), to) < 100.0f);
        GN_CHECK(plan.points.back().x < 100.0f);
    }
}

static void Test_NoNavNearStart()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    world.AddSolid(Vec3(-8, -256, 0), Vec3(8, 256, 256));

    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(-128, 0, 10) });

    NavPathPlanner planner;
    const PlanResult plan = planner.Plan(grid, &world, Vec3(128, 0, 0), Vec3(-128, 0, 0), HullIndex::Standing, PlanConfig());
    GN_CHECK(plan.status == PlanStatus::NoStartNode);
}

static void Test_StartNode_IsNotPickedThroughWall()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    world.AddSolid(Vec3(-2, -256, 0), Vec3(2, 256, 256)); // тонкая глухая стена

    NavGrid grid;
    // Навмеш только слева от стены.
    BuildNav(world, grid, HullIndex::Standing, { Vec3(-128, 0, 10) });

    // Точечная entity справа у самой стены: ближайшие узлы — за стеной, идти к ним нельзя.
    NavPathPlanner planner;
    const uint32_t start = planner.FindStartNode(grid, &world, Vec3(20, 0, 0), HullIndex::Standing, PlanConfig());
    GN_CHECK(start == kInvalidNodeId);
}

static void Test_PrefersDryAndSafeRoute()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    // Прямая дорога идёт через глубокую воду, обход посуху чуть длиннее.
    world.AddVolume(Vec3(-40, -256, 0), Vec3(40, 60, 64), NavContents::Water);

    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(-128, 0, 10) });

    NavPathPlanner planner;
    const PlanResult plan = planner.Plan(grid, &world, Vec3(-128, 0, 0), Vec3(128, 0, 0), HullIndex::Standing, PlanConfig());
    GN_CHECK(plan.status == PlanStatus::Found);

    float maxY = -1e9f;
    for (const Vec3& p : plan.points) maxY = std::max(maxY, p.y);
    GN_CHECK(maxY > 60.0f); // обошли воду
}

static void Test_MultiLevel_GoalOnUpperFloor()
{
    BoxWorld world;
    AddRoom(world, 256.0f, 512.0f);
    // Второй этаж над правой половиной, лестница снизу вверх вдоль X.
    world.AddSolid(Vec3(64, -256, 120), Vec3(256, 256, 128));
    for (int i = 0; i < 16; ++i)
    {
        world.AddSolid(Vec3(-192.0f + 16.0f * i, 100, 0), Vec3(64, 200, 8.0f * (i + 1)));
    }

    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(-200, -100, 10) });

    NavPathPlanner planner;
    // Цель на втором этаже, прямо над точкой первого этажа.
    const PlanResult upstairs = planner.Plan(grid, &world, Vec3(150, -100, 0), Vec3(150, -100, 128),
                                             HullIndex::Standing, PlanConfig());
    GN_CHECK(upstairs.status == PlanStatus::Found);
    if (!upstairs.points.empty())
    {
        GN_CHECK(std::fabs(upstairs.points.back().z - 128.0f) < 1.0f);
    }

    // Та же точка XY, но цель на первом этаже — никуда подниматься не надо.
    const PlanResult downstairs = planner.Plan(grid, &world, Vec3(100, -100, 0), Vec3(150, -100, 0),
                                               HullIndex::Standing, PlanConfig());
    GN_CHECK(downstairs.status == PlanStatus::Found);
    if (!downstairs.points.empty())
    {
        GN_CHECK(std::fabs(downstairs.points.back().z) < 1.0f);
        GN_CHECK(PathLength(Vec3(100, -100, 0), downstairs.points) < 80.0f);
    }
}

static void RunAllTests()
{
    GN_RUN(Test_OpenRoom_StraightLine);
    GN_RUN(Test_AroundWall_GoesThroughGapWithClearance);
    GN_RUN(Test_Unreachable_ReturnsPartialNearGoal);
    GN_RUN(Test_NoNavNearStart);
    GN_RUN(Test_StartNode_IsNotPickedThroughWall);
    GN_RUN(Test_PrefersDryAndSafeRoute);
    GN_RUN(Test_MultiLevel_GoalOnUpperFloor);
}

GN_TEST_MAIN()
