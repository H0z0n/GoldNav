// Сквозные тесты: навмеш -> планировщик -> PathFollower -> GroundMove в коробочном мире.
#include "micro_test.h"
#include "fixtures/nav_sim.h"

using namespace goldnav;
using namespace goldnav_test;

static void Test_NoPath_ReturnsNoPath()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    BoxMoveTracer tracer(world, HullIndex::Standing);
    PathFollower follower;
    FollowState state;
    MoverState mover;

    const FollowStep step = follower.Update(state, tracer, mover, Vec3(0, 0, 0), 200.0f, 0.02f);
    GN_CHECK(step.result == FollowResult::NoPath);
}

static void Test_OpenRoom_ArrivesExactlyAtGoal()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(0, 0, 10) });

    const Vec3 from(-200, -150, 0.03125f);
    const Vec3 to(183, 91, 0);
    const SimResult sim = SimulateMove(world, grid, from, to, HullIndex::Standing, 250.0f);

    GN_CHECK(sim.arrived);
    GN_CHECK(Distance2D(sim.finalFeet, to) < 3.0f);
    GN_CHECK(sim.plans == 1);
    // Время близко к расстояние/скорость: шли прямо и не топтались.
    const float ideal = Distance2D(from, to) / 250.0f;
    GN_CHECK(sim.time < ideal * 1.25f + 0.5f);
}

static void Test_AroundWall_Arrives()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    world.AddSolid(Vec3(-8, -256, 0), Vec3(8, 96, 256));
    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(-128, 0, 10) });

    const Vec3 to(128, -100, 0);
    const SimResult sim = SimulateMove(world, grid, Vec3(-128, -100, 0.03125f), to, HullIndex::Standing);

    GN_CHECK(sim.arrived);
    GN_CHECK(Distance2D(sim.finalFeet, to) < 3.0f);
    GN_CHECK(sim.plans == 1); // путь не пришлось перестраивать — не цеплялись за угол

    // По дороге ни разу не оказались "в стене" (с учётом полуширины хулла).
    bool insideWall = false;
    for (const Vec3& p : sim.trail)
    {
        if (p.y < 96.0f && std::fabs(p.x) < 8.0f + 15.9f) insideWall = true;
    }
    GN_CHECK(!insideWall);
}

static void Test_Maze_Arrives()
{
    BoxWorld world;
    AddRoom(world, 320.0f);
    // Змейка из трёх перегородок с проходами на противоположных концах.
    world.AddSolid(Vec3(-168, -320, 0), Vec3(-152, 200, 256));
    world.AddSolid(Vec3(-8, -200, 0), Vec3(8, 320, 256));
    world.AddSolid(Vec3(152, -320, 0), Vec3(168, 200, 256));
    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(-250, 0, 10) });

    const Vec3 to(250, -250, 0);
    const SimResult sim = SimulateMove(world, grid, Vec3(-250, -250, 0.03125f), to, HullIndex::Standing);

    GN_CHECK(sim.arrived);
    GN_CHECK(Distance2D(sim.finalFeet, to) < 3.0f);
    GN_CHECK(sim.plans == 1);
}

static void Test_UpAndDownStairs_Arrives()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    for (int i = 0; i < 8; ++i)
    {
        world.AddSolid(Vec3(16.0f * i, -256, 0), Vec3(256, 256, 8.0f * (i + 1)));
    }
    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(-128, 0, 10) });

    const SimResult up = SimulateMove(world, grid, Vec3(-200, 50, 0.03125f), Vec3(220, -60, 64), HullIndex::Standing);
    GN_CHECK(up.arrived);
    GN_CHECK(std::fabs(up.finalFeet.z - 64.0f) < 1.0f);
    GN_CHECK(up.plans == 1);

    const SimResult down = SimulateMove(world, grid, Vec3(220, -60, 64.03125f), Vec3(-200, 50, 0), HullIndex::Standing);
    GN_CHECK(down.arrived);
    GN_CHECK(std::fabs(down.finalFeet.z) < 1.0f);
    GN_CHECK(down.plans == 1);
}

static void Test_DropFromLedge_Arrives()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    world.AddSolid(Vec3(0, -256, 0), Vec3(256, 256, 64));
    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(128, 0, 70) });

    const Vec3 to(-180, 40, 0);
    const SimResult sim = SimulateMove(world, grid, Vec3(150, 0, 64.03125f), to, HullIndex::Standing);

    GN_CHECK(sim.arrived);
    GN_CHECK(Distance2D(sim.finalFeet, to) < 3.0f);
    GN_CHECK(std::fabs(sim.finalFeet.z) < 1.0f);
}

static void Test_UnreachableGoal_StopsAsCloseAsPossible()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    world.AddSolid(Vec3(0, -256, 0), Vec3(256, 256, 64));
    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(128, 0, 70) });

    // Снизу на платформу не подняться: идём к её подножию и останавливаемся.
    const SimResult sim = SimulateMove(world, grid, Vec3(-200, 0, 0.03125f), Vec3(100, 0, 64), HullIndex::Standing);

    GN_CHECK(sim.arrived);
    GN_CHECK(sim.lastStatus == PlanStatus::Partial);
    GN_CHECK(sim.finalFeet.x > -60.0f && sim.finalFeet.x < 0.0f);
    GN_CHECK(std::fabs(sim.finalFeet.z) < 1.0f);
}

static void Test_StandingPlayerInTheWay_IsWalkedAround()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(0, 0, 10) });

    // "Игрок" стоит ровно на прямой между стартом и целью. Навмеш о нём не знает.
    world.AddEntity(Vec3(-16, -16, 0), Vec3(16, 16, 72), 3);

    const Vec3 to(200, 0, 0);
    const SimResult sim = SimulateMove(world, grid, Vec3(-200, 0, 0.03125f), to, HullIndex::Standing);

    GN_CHECK(sim.arrived);
    GN_CHECK(Distance2D(sim.finalFeet, to) < 3.0f);

    float maxSide = 0.0f;
    for (const Vec3& p : sim.trail) maxSide = std::max(maxSide, std::fabs(p.y));
    GN_CHECK(maxSide > 30.0f);  // обошли сбоку
    GN_CHECK(maxSide < 120.0f); // без лишнего крюка
    GN_CHECK(sim.time < 5.0f);
}

static void Test_PathBlockedByNewObstacle_GivesUpInsteadOfHanging()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    world.AddSolid(Vec3(-8, -256, 0), Vec3(8, -24, 256));
    world.AddSolid(Vec3(-8, 24, 0), Vec3(8, 256, 256)); // дверной проём шириной 48
    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(-128, 0, 10) });

    // Проём наглухо заставили ящиком уже после генерации.
    world.AddEntity(Vec3(-8, -24, 0), Vec3(8, 24, 72), 9);

    const SimResult sim = SimulateMove(world, grid, Vec3(-128, 0, 0.03125f), Vec3(128, 0, 0), HullIndex::Standing,
                                       200.0f, 0.02f, 60.0f);

    GN_CHECK(!sim.arrived);
    GN_CHECK(sim.gaveUp);     // исчерпали попытки и сообщили о неудаче
    GN_CHECK(sim.time < 30.0f);
    GN_CHECK(sim.finalFeet.x < 0.0f);
}

static void Test_WaypointOnOtherFloor_IsNotCountedAsReached()
{
    BoxWorld world;
    AddRoom(world, 256.0f, 512.0f);
    BoxMoveTracer tracer(world, HullIndex::Standing);
    PathFollower follower;
    FollowState state;
    MoverState mover;

    // Первая точка пути — прямо над нами, на 128 выше (другой этаж).
    state.SetPath({ Vec3(0, 0, 128), Vec3(100, 0, 128) });
    const FollowStep step = follower.Update(state, tracer, mover, Vec3(0, 0, 0.03125f), 200.0f, 0.02f);

    GN_CHECK(state.currentIndex == 0);
    GN_CHECK(step.result == FollowResult::OffPath);
}

static void Test_HighDrop_LandingPointCloseToLedge()
{
    // Уступ высотой 88 (выше heightTolerance): точка приземления всего в клетке от
    // края. Entity не должна "терять путь", пока ещё стоит на краю над ней.
    BoxWorld world;
    AddRoom(world, 256.0f);
    world.AddSolid(Vec3(0, -256, 0), Vec3(256, 256, 88));
    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(128, 0, 95) });

    const Vec3 to(-180, 40, 0);
    const SimResult sim = SimulateMove(world, grid, Vec3(150, 0, 88.03125f), to, HullIndex::Standing);

    GN_CHECK(sim.arrived);
    GN_CHECK(Distance2D(sim.finalFeet, to) < 3.0f);
    GN_CHECK(sim.plans <= 2);
}

static void Test_SmallEntity_PointHull()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    world.AddSolid(Vec3(-8, -256, 0), Vec3(8, 96, 256));
    for (int i = 0; i < 4; ++i)
    {
        world.AddSolid(Vec3(100.0f + 16.0f * i, -256, 0), Vec3(256, 0, 8.0f * (i + 1)));
    }
    NavGrid grid;
    BuildNav(world, grid, HullIndex::Point, { Vec3(-128, 0, 10) });

    const Vec3 to(220, -120, 32);
    const SimResult sim = SimulateMove(world, grid, Vec3(-128, -100, 0.03125f), to, HullIndex::Point, 150.0f);

    GN_CHECK(sim.arrived);
    GN_CHECK(Distance2D(sim.finalFeet, to) < 3.0f);
    GN_CHECK(std::fabs(sim.finalFeet.z - 32.0f) < 1.0f);
}

static void Test_VariousFrameRates_AllArrive()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    world.AddSolid(Vec3(-8, -256, 0), Vec3(8, 96, 256));
    for (int i = 0; i < 8; ++i)
    {
        world.AddSolid(Vec3(100.0f + 16.0f * i, -256, 0), Vec3(256, 0, 8.0f * (i + 1)));
    }
    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(-128, 0, 10) });

    const Vec3 to(236, -120, 64);
    const float frameTimes[] = { 0.001f, 0.01f, 0.033f, 0.1f };
    for (float dt : frameTimes)
    {
        const SimResult sim = SimulateMove(world, grid, Vec3(-128, -100, 0.03125f), to, HullIndex::Standing, 250.0f, dt);
        GN_CHECK(sim.arrived);
        GN_CHECK(Distance2D(sim.finalFeet, to) < 3.0f);
        GN_CHECK(std::fabs(sim.finalFeet.z - 64.0f) < 1.0f);
    }
}

static void RunAllTests()
{
    GN_RUN(Test_NoPath_ReturnsNoPath);
    GN_RUN(Test_OpenRoom_ArrivesExactlyAtGoal);
    GN_RUN(Test_AroundWall_Arrives);
    GN_RUN(Test_Maze_Arrives);
    GN_RUN(Test_UpAndDownStairs_Arrives);
    GN_RUN(Test_DropFromLedge_Arrives);
    GN_RUN(Test_HighDrop_LandingPointCloseToLedge);
    GN_RUN(Test_UnreachableGoal_StopsAsCloseAsPossible);
    GN_RUN(Test_StandingPlayerInTheWay_IsWalkedAround);
    GN_RUN(Test_PathBlockedByNewObstacle_GivesUpInsteadOfHanging);
    GN_RUN(Test_WaypointOnOtherFloor_IsNotCountedAsReached);
    GN_RUN(Test_SmallEntity_PointHull);
    GN_RUN(Test_VariousFrameRates_AllArrive);
}

GN_TEST_MAIN()
