#include "micro_test.h"
#include "fixtures/nav_sim.h"

using namespace goldnav;
using namespace goldnav_test;

// Двигает entity с постоянной желаемой скоростью заданное время.
static Vec3 Run(BoxWorld& world, HullIndex hull, Vec3 feet, const Vec3& wish, float seconds, MoverState& state,
                MoveOutcome* last = nullptr, float dt = 0.02f)
{
    BoxMoveTracer tracer(world, hull);
    MoverConfig config;
    for (float t = 0.0f; t < seconds; t += dt)
    {
        const MoveOutcome outcome = GroundMove(tracer, feet, wish, dt, state, config);
        feet = outcome.feet;
        if (last) *last = outcome;
    }
    return feet;
}

static void Test_WalksOnFlatGroundAtRequestedSpeed()
{
    BoxWorld world;
    AddRoom(world, 512.0f);
    MoverState state;
    const Vec3 end = Run(world, HullIndex::Standing, Vec3(0, 0, 0.03125f), Vec3(200, 0, 0), 1.0f, state);

    GN_CHECK(std::fabs(end.x - 200.0f) < 6.0f); // ровно speed * time, без "трения"
    GN_CHECK(std::fabs(end.y) < 0.01f);
    GN_CHECK(std::fabs(end.z) < 0.1f);
    GN_CHECK(state.onGround);
}

static void Test_ClimbsStairs()
{
    BoxWorld world;
    AddRoom(world, 512.0f);
    for (int i = 0; i < 8; ++i)
    {
        world.AddSolid(Vec3(64.0f + 16.0f * i, -512, 0), Vec3(512, 512, 8.0f * (i + 1)));
    }

    MoverState state;
    const Vec3 end = Run(world, HullIndex::Standing, Vec3(0, 0, 0.03125f), Vec3(150, 0, 0), 2.5f, state);

    GN_CHECK(end.x > 250.0f);
    GN_CHECK(std::fabs(end.z - 64.0f) < 0.5f);
    GN_CHECK(state.onGround);
}

static void Test_WalksDownStairsWithoutFalling()
{
    BoxWorld world;
    AddRoom(world, 512.0f);
    for (int i = 0; i < 8; ++i)
    {
        world.AddSolid(Vec3(64.0f + 16.0f * i, -512, 0), Vec3(512, 512, 8.0f * (i + 1)));
    }

    BoxMoveTracer tracer(world, HullIndex::Standing);
    MoverConfig config;
    MoverState state;
    Vec3 feet(300, 0, 64.03125f);
    bool everAirborne = false;
    for (int i = 0; i < 150; ++i)
    {
        feet = GroundMove(tracer, feet, Vec3(-150, 0, 0), 0.02f, state, config).feet;
        if (!state.onGround) everAirborne = true;
    }

    GN_CHECK(feet.x < 0.0f);
    GN_CHECK(std::fabs(feet.z) < 0.5f);
    GN_CHECK(!everAirborne); // прилипание к ступенькам, а не прыжки
}

static void Test_TooHighStepBlocks()
{
    BoxWorld world;
    AddRoom(world, 512.0f);
    world.AddSolid(Vec3(64, -512, 0), Vec3(512, 512, 24)); // 24 > stepHeight 18

    MoverState state;
    MoveOutcome last;
    const Vec3 end = Run(world, HullIndex::Standing, Vec3(0, 0, 0.03125f), Vec3(150, 0, 0), 1.0f, state, &last);

    GN_CHECK(end.x < 64.0f - 15.9f);
    GN_CHECK(std::fabs(end.z) < 0.1f);
    GN_CHECK(last.blocked);
    GN_CHECK(last.blockEntity == 0);
}

static void Test_SlidesAlongWall()
{
    BoxWorld world;
    AddRoom(world, 512.0f);
    world.AddSolid(Vec3(64, -512, 0), Vec3(80, 512, 256));

    MoverState state;
    // Идём в стену наискось: упираемся по X, но продолжаем ехать вдоль неё по Y.
    const Vec3 end = Run(world, HullIndex::Standing, Vec3(0, 0, 0.03125f), Vec3(150, 150, 0), 1.0f, state);

    GN_CHECK(end.x < 64.0f - 15.9f);
    GN_CHECK(end.y > 120.0f);
}

static void Test_FallsOffLedgeAndLands()
{
    BoxWorld world;
    AddRoom(world, 512.0f);
    world.AddSolid(Vec3(-512, -512, 0), Vec3(0, 512, 100)); // платформа высотой 100 слева

    BoxMoveTracer tracer(world, HullIndex::Standing);
    MoverConfig config;
    MoverState state;
    Vec3 feet(-100, 0, 100.03125f);
    bool landed = false;
    float landedDrop = 0.0f;
    bool wasAirborne = false;
    for (int i = 0; i < 150; ++i)
    {
        const MoveOutcome outcome = GroundMove(tracer, feet, Vec3(150, 0, 0), 0.02f, state, config);
        feet = outcome.feet;
        if (!state.onGround) wasAirborne = true;
        if (outcome.landed)
        {
            landed = true;
            landedDrop = outcome.landedDrop;
        }
    }

    GN_CHECK(wasAirborne);
    GN_CHECK(landed);
    GN_CHECK(std::fabs(landedDrop - 100.0f) < 2.0f);
    GN_CHECK(std::fabs(feet.z) < 0.5f);
    GN_CHECK(feet.x > 100.0f);
    GN_CHECK(state.onGround);
}

static void Test_SpawnedInAir_FallsToGround()
{
    BoxWorld world;
    AddRoom(world, 512.0f);

    MoverState state;
    const Vec3 end = Run(world, HullIndex::Standing, Vec3(0, 0, 200), Vec3(0, 0, 0), 1.5f, state);

    GN_CHECK(std::fabs(end.z) < 0.5f);
    GN_CHECK(state.onGround);
}

static void Test_EmbeddedInFloor_IsPushedOut()
{
    BoxWorld world;
    AddRoom(world, 512.0f);

    MoverState state;
    MoveOutcome last;
    const Vec3 end = Run(world, HullIndex::Standing, Vec3(0, 0, -3), Vec3(100, 0, 0), 0.5f, state, &last);

    GN_CHECK(!last.stuckInSolid);
    GN_CHECK(end.x > 30.0f);
    GN_CHECK(end.z >= 0.0f);
}

static void Test_BlockedByEntity_ReportsIt()
{
    BoxWorld world;
    AddRoom(world, 512.0f);
    world.AddEntity(Vec3(84, -16, 0), Vec3(116, 16, 72), 7);

    MoverState state;
    MoveOutcome last;
    const Vec3 end = Run(world, HullIndex::Standing, Vec3(0, 0, 0.03125f), Vec3(150, 0, 0), 1.0f, state, &last);

    GN_CHECK(end.x < 84.0f - 15.9f);
    GN_CHECK(last.blocked);
    GN_CHECK(last.blockEntity == 7);
}

static void Test_LowFrameRate_StillClimbsAndDoesNotTunnel()
{
    BoxWorld world;
    AddRoom(world, 512.0f);
    for (int i = 0; i < 8; ++i)
    {
        world.AddSolid(Vec3(64.0f + 16.0f * i, -512, 0), Vec3(300, 512, 8.0f * (i + 1)));
    }
    world.AddSolid(Vec3(300, -512, 0), Vec3(316, 512, 256)); // стена за площадкой

    MoverState state;
    // 10 кадров в секунду: шаг 25 юнитов — больше ступеньки.
    const Vec3 end = Run(world, HullIndex::Standing, Vec3(0, 0, 0.03125f), Vec3(250, 0, 0), 3.0f, state, nullptr, 0.1f);

    GN_CHECK(end.z > 60.0f);
    GN_CHECK(end.x < 300.0f - 15.9f); // сквозь стену не прошли
}

static void RunAllTests()
{
    GN_RUN(Test_WalksOnFlatGroundAtRequestedSpeed);
    GN_RUN(Test_ClimbsStairs);
    GN_RUN(Test_WalksDownStairsWithoutFalling);
    GN_RUN(Test_TooHighStepBlocks);
    GN_RUN(Test_SlidesAlongWall);
    GN_RUN(Test_FallsOffLedgeAndLands);
    GN_RUN(Test_SpawnedInAir_FallsToGround);
    GN_RUN(Test_EmbeddedInFloor_IsPushedOut);
    GN_RUN(Test_BlockedByEntity_ReportsIt);
    GN_RUN(Test_LowFrameRate_StillClimbsAndDoesNotTunnel);
}

GN_TEST_MAIN()
