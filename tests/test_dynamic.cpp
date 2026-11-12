// Обновление 1.1: динамические препятствия, прыжки, плавность.
#include "micro_test.h"
#include "fixtures/nav_sim.h"
#include "navmesh/obstacle_set.h"

using namespace goldnav;
using namespace goldnav_test;

static NavGenConfig JumpConfig(float maxJump)
{
    NavGenConfig config;
    config.maxJump = maxJump;
    return config;
}

// "Ходок", который видит и карту, и динамические препятствия.
class ObstacleTracer : public IMoveTracer
{
public:
    ObstacleTracer(const ICollisionWorld& world, HullIndex hull) : m_world(world), m_hull(hull) {}
    void Trace(const Vec3& from, const Vec3& to, HullTrace& out) override { m_world.TraceHull(from, to, m_hull, out); }
private:
    const ICollisionWorld& m_world;
    HullIndex m_hull;
};

struct Walk
{
    bool arrived = false;
    Vec3 feet;
    float time = 0.0f;
    float maxSide = 0.0f;
    float maxTurnPerStep = 0.0f; // наибольший излом направления за один шаг, градусы
    int jumps = 0;
};

static Walk WalkTo(const ICollisionWorld& world, const NavGrid& grid, Vec3 feet, const Vec3& to,
                   float jumpHeight = 0.0f, float speed = 200.0f)
{
    Walk result;
    NavPathPlanner planner;
    PlanConfig config;
    config.cost.maxJump = jumpHeight;
    PathFollower follower;
    ObstacleTracer tracer(world, HullIndex::Standing);
    FollowState follow;
    follow.jumpHeight = jumpHeight;
    MoverState mover;

    int plans = 0;
    auto plan = [&]() {
        if (++plans > 6) return false;
        const PlanResult p = planner.Plan(grid, &world, feet, to, HullIndex::Standing, config);
        if (!p.HasPath()) return false;
        follow.SetPath(p.points);
        return true;
    };
    if (!plan()) { result.feet = feet; return result; }

    Vec3 lastDir;
    const float dt = 0.02f;
    while (result.time < 40.0f)
    {
        const FollowStep step = follower.Update(follow, tracer, mover, feet, speed, dt);
        const Vec3 moved(step.feet.x - feet.x, step.feet.y - feet.y, 0.0f);
        if (moved.Length2D() > speed * dt * 0.5f)
        {
            const Vec3 dir = moved.Normalized();
            if (lastDir.LengthSq() > 0.5f)
            {
                const float c = std::max(-1.0f, std::min(1.0f, Dot(dir, lastDir)));
                result.maxTurnPerStep = std::max(result.maxTurnPerStep, std::acos(c) * 57.2958f);
            }
            lastDir = dir;
        }
        feet = step.feet;
        result.time += dt;
        result.maxSide = std::max(result.maxSide, std::fabs(feet.y));
        if (step.jumped) ++result.jumps;
        if (step.result == FollowResult::Arrived) { result.arrived = true; break; }
        if (step.result != FollowResult::Moving && !plan()) break;
    }
    result.feet = feet;
    return result;
}

// ---------- прыжки ----------

static void Test_JumpEdges_AreBuiltOnlyUpToMaxJump()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    world.AddSolid(Vec3(64, -256, 0), Vec3(256, 256, 40)); // уступ 40: шагом не взять

    NavGrid noJump;
    BuildNav(world, noJump, HullIndex::Standing, { Vec3(-128, 0, 10) });
    GN_CHECK(noJump.FindNearestNode(Vec3(128, 0, 40), 1, 16.0f) == kInvalidNodeId);

    NavGrid lowJump;
    BuildNav(world, lowJump, HullIndex::Standing, { Vec3(-128, 0, 10) }, JumpConfig(32.0f));
    GN_CHECK(lowJump.FindNearestNode(Vec3(128, 0, 40), 1, 16.0f) == kInvalidNodeId);

    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(-128, 0, 10) }, JumpConfig(64.0f));
    GN_CHECK(grid.FindNearestNode(Vec3(128, 0, 40), 1, 16.0f) != kInvalidNodeId);

    int jumps = 0;
    for (uint32_t id = 0; id < grid.NodeCount(); ++id)
    {
        for (int dir = 0; dir < kNavDirCount; ++dir)
        {
            if (grid.Node(id).edgeType[dir] != kEdgeJump) continue;
            ++jumps;
            GN_CHECK(!NavDirIsDiagonal(dir));
            GN_CHECK(std::fabs(NavDecodeHeight(grid.Node(id).edgeHeight[dir]) - 40.0f) <= 2.0f);
        }
    }
    GN_CHECK(jumps > 0);
}

static void Test_JumpHeight_IsPerEntity()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    world.AddSolid(Vec3(64, -256, 0), Vec3(256, 256, 40));
    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(-128, 0, 10) }, JumpConfig(64.0f));

    NavPathPlanner planner;
    const Vec3 from(-128, 0, 0), to(160, 0, 40);

    PlanConfig cannot; // по умолчанию entity не прыгает
    GN_CHECK(planner.Plan(grid, &world, from, to, HullIndex::Standing, cannot).status == PlanStatus::Partial);

    PlanConfig weak;
    weak.cost.maxJump = 30.0f;
    GN_CHECK(planner.Plan(grid, &world, from, to, HullIndex::Standing, weak).status == PlanStatus::Partial);

    PlanConfig strong;
    strong.cost.maxJump = 45.0f;
    GN_CHECK(planner.Plan(grid, &world, from, to, HullIndex::Standing, strong).status == PlanStatus::Found);
}

static void Test_Follower_JumpsOntoLedge()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    world.AddSolid(Vec3(64, -256, 0), Vec3(256, 256, 40));
    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(-128, 0, 10) }, JumpConfig(64.0f));

    const Vec3 to(170, 30, 40);
    const Walk walk = WalkTo(world, grid, Vec3(-150, -20, 0.03125f), to, 45.0f);
    GN_CHECK(walk.arrived);
    GN_CHECK(Distance2D(walk.feet, to) < 3.0f);
    GN_CHECK(std::fabs(walk.feet.z - 40.0f) < 1.0f);
    GN_CHECK(walk.jumps >= 1 && walk.jumps <= 2);

    // Тот же маршрут без умения прыгать: подходит к уступу и останавливается внизу.
    const Walk grounded = WalkTo(world, grid, Vec3(-150, -20, 0.03125f), to, 0.0f);
    GN_CHECK(grounded.jumps == 0);
    GN_CHECK(std::fabs(grounded.feet.z) < 1.0f);
}

static void Test_MaxDrop_IsPerEntity()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    world.AddSolid(Vec3(0, -256, 0), Vec3(256, 256, 100));
    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(128, 0, 110) });

    NavPathPlanner planner;
    const Vec3 from(128, 0, 100), to(-128, 0, 0);

    PlanConfig brave;
    GN_CHECK(planner.Plan(grid, &world, from, to, HullIndex::Standing, brave).status == PlanStatus::Found);

    PlanConfig careful;
    careful.cost.maxDrop = 60.0f;
    GN_CHECK(planner.Plan(grid, &world, from, to, HullIndex::Standing, careful).status == PlanStatus::Partial);
}

// ---------- динамические препятствия ----------

static void Test_ObstacleSet_ClipsTraces()
{
    BoxWorld base;
    AddRoom(base, 256.0f);
    ObstacleSet set;
    ObstacleWorld world(base, set);

    HullTrace tr;
    world.TraceHull(Vec3(-100, 0, 1), Vec3(100, 0, 1), HullIndex::Standing, tr);
    GN_CHECK(tr.Clear());

    set.Set(7, Vec3(-10, -10, 0), Vec3(10, 10, 40));
    world.TraceHull(Vec3(-100, 0, 1), Vec3(100, 0, 1), HullIndex::Standing, tr);
    GN_CHECK(tr.Hit());
    GN_CHECK(tr.entity == 7);
    GN_CHECK(std::fabs(tr.endPos.x - (-26.0f)) < 0.1f); // грань -10 минус полуширина хулла 16
    GN_CHECK(tr.normal.x < -0.9f);

    world.TraceHull(Vec3(0, 0, 1), Vec3(0, 0, 1), HullIndex::Standing, tr);
    GN_CHECK(tr.startSolid);
    GN_CHECK(world.PointContents(Vec3(0, 0, 10)) == NavContents::Solid);

    GN_CHECK(set.Remove(7));
    GN_CHECK(!set.Remove(7));
    world.TraceHull(Vec3(-100, 0, 1), Vec3(100, 0, 1), HullIndex::Standing, tr);
    GN_CHECK(tr.Clear());
}

static void Test_PlacedBox_IsAvoided_AndFreedWhenRemoved()
{
    BoxWorld base;
    AddRoom(base, 256.0f);
    ObstacleSet set;
    ObstacleWorld world(base, set);

    NavGrid grid;
    NavMeshGenerator generator;
    generator.Begin(grid, world, NavLayer::Medium, NavGenConfig());
    generator.AddSeed(Vec3(0, 0, 10));
    generator.RunToCompletion();
    const size_t nodesBefore = grid.NodeCount();

    const Vec3 from(-200, 0, 0.03125f), to(200, 0, 0);

    const Walk direct = WalkTo(world, grid, from, to);
    GN_CHECK(direct.arrived);
    GN_CHECK(direct.maxSide < 8.0f); // по прямой

    // Ставим ящик 64x64 прямо на дороге.
    const Vec3 boxMins(-32, -32, 0), boxMaxs(32, 32, 64);
    set.Set(5, boxMins, boxMaxs);
    GN_CHECK(generator.InvalidateRegion(boxMins, boxMaxs) > 0);
    GN_CHECK(!generator.IsIdle());
    generator.RunToCompletion();

    GN_CHECK(grid.NodeCount() == nodesBefore);                                   // лишних узлов не появилось
    GN_CHECK(grid.FindNearestNode(Vec3(8, 8, 0), 1, 20.0f) == kInvalidNodeId);  // под ящиком навмеша больше нет

    NavPathPlanner planner;
    const PlanResult around = planner.Plan(grid, &world, from, to, HullIndex::Standing, PlanConfig());
    GN_CHECK(around.status == PlanStatus::Found);

    const Walk detour = WalkTo(world, grid, from, to);
    GN_CHECK(detour.arrived);
    GN_CHECK(detour.maxSide > 48.0f);  // обошли ящик (32 + полуширина хулла 16)
    GN_CHECK(detour.maxSide < 110.0f); // с небольшим запасом, без крюка

    // Ящик убрали — путь снова прямой.
    GN_CHECK(set.Remove(5));
    generator.InvalidateRegion(boxMins, boxMaxs);
    generator.RunToCompletion();
    GN_CHECK(grid.FindNearestNode(Vec3(8, 8, 0), 1, 20.0f) != kInvalidNodeId);

    const Walk again = WalkTo(world, grid, from, to);
    GN_CHECK(again.arrived);
    GN_CHECK(again.maxSide < 8.0f);
}

static void Test_Door_ClosedBlocks_OpenLetsThrough()
{
    // Стена с проёмом; "дверь" — коробка в проёме (в боевом коде её видит движковая
    // трасса, здесь её роль играет ObstacleSet — механизм обновления графа тот же).
    BoxWorld base;
    AddRoom(base, 256.0f);
    base.AddSolid(Vec3(-8, -256, 0), Vec3(8, -32, 256));
    base.AddSolid(Vec3(-8, 32, 0), Vec3(8, 256, 256));
    ObstacleSet set;
    ObstacleWorld world(base, set);

    const Vec3 doorMins(-4, -32, 0), doorMaxs(4, 32, 128);
    set.Set(9, doorMins, doorMaxs); // закрыта на момент генерации

    NavGrid grid;
    NavMeshGenerator generator;
    generator.Begin(grid, world, NavLayer::Medium, NavGenConfig());
    generator.AddSeed(Vec3(-128, 0, 10));
    generator.RunToCompletion();

    // За закрытой дверью заливка ничего не нашла.
    GN_CHECK(grid.FindNearestNode(Vec3(128, 0, 0), 1, 16.0f) == kInvalidNodeId);

    // Дверь открылась: участок перепроверяется, заливка проходит в соседнюю комнату.
    set.Remove(9);
    generator.InvalidateRegion(doorMins, doorMaxs);
    generator.RunToCompletion();
    GN_CHECK(grid.FindNearestNode(Vec3(128, 0, 0), 1, 16.0f) != kInvalidNodeId);

    const Vec3 from(-150, 100, 0.03125f), to(150, -100, 0);
    const Walk through = WalkTo(world, grid, from, to);
    GN_CHECK(through.arrived);

    // Закрылась снова: прохода нет, идём "как можно ближе" и остаёмся на своей стороне.
    set.Set(9, doorMins, doorMaxs);
    generator.InvalidateRegion(doorMins, doorMaxs);
    generator.RunToCompletion();

    NavPathPlanner planner;
    GN_CHECK(planner.Plan(grid, &world, from, to, HullIndex::Standing, PlanConfig()).status == PlanStatus::Partial);
}

static void Test_NodeOnTopOfRemovedBox_Dies()
{
    BoxWorld base;
    AddRoom(base, 256.0f);
    ObstacleSet set;
    ObstacleWorld world(base, set);

    // Низкий помост (16) — на него заходят обычным шагом.
    const Vec3 mins(-48, -48, 0), maxs(48, 48, 16);
    set.Set(3, mins, maxs);

    NavGrid grid;
    NavMeshGenerator generator;
    generator.Begin(grid, world, NavLayer::Medium, NavGenConfig());
    generator.AddSeed(Vec3(-200, 0, 10));
    generator.RunToCompletion();

    const uint32_t onTop = grid.FindNearestNode(Vec3(8, 8, 16), 1, 6.0f);
    GN_CHECK(onTop != kInvalidNodeId);

    set.Remove(3);
    generator.InvalidateRegion(mins, maxs);
    generator.RunToCompletion();

    // Узел "в воздухе" умер, на его месте — узел на полу.
    GN_CHECK(grid.Node(onTop).variantMask == 0);
    const uint32_t onFloor = grid.FindNearestNode(Vec3(8, 8, 0), 1, 6.0f);
    GN_CHECK(onFloor != kInvalidNodeId && onFloor != onTop);

    const Walk walk = WalkTo(world, grid, Vec3(-200, 0, 0.03125f), Vec3(200, 0, 0));
    GN_CHECK(walk.arrived);
}

// ---------- плавность ----------

static void Test_Turns_AreSmoothArcs()
{
    BoxWorld world;
    AddRoom(world, 256.0f);
    world.AddSolid(Vec3(-8, -256, 0), Vec3(8, 96, 256)); // обход торца стены: два поворота

    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(-128, 0, 10) });

    const Vec3 to(128, -100, 0);
    const Walk walk = WalkTo(world, grid, Vec3(-128, -100, 0.03125f), to);
    GN_CHECK(walk.arrived);
    GN_CHECK(Distance2D(walk.feet, to) < 3.0f);

    // Ни одного резкого излома: направление движения за шаг (0.02 с) меняется плавно.
    // Без инерции здесь были бы изломы по 40-90 градусов.
    GN_CHECK(walk.maxTurnPerStep < 12.0f);
}

static void Test_Acceleration_RampsUpFromStandstill()
{
    BoxWorld world;
    AddRoom(world, 512.0f);
    NavGrid grid;
    BuildNav(world, grid, HullIndex::Standing, { Vec3(0, 0, 10) });

    NavPathPlanner planner;
    PathFollower follower;
    BoxMoveTracer tracer(world, HullIndex::Standing);
    FollowState follow;
    MoverState mover;
    Vec3 feet(-400, 0, 0.03125f);
    follow.SetPath(planner.Plan(grid, &world, feet, Vec3(400, 0, 0), HullIndex::Standing, PlanConfig()).points);

    const float speed = 200.0f, dt = 0.02f;
    const FollowStep first = follower.Update(follow, tracer, mover, feet, speed, dt);
    GN_CHECK(first.speed < speed * 0.25f); // первый шаг — не с места в карьер

    feet = first.feet;
    float reached = 0.0f;
    for (int i = 0; i < 25; ++i) // полсекунды
    {
        const FollowStep step = follower.Update(follow, tracer, mover, feet, speed, dt);
        feet = step.feet;
        reached = step.speed;
    }
    GN_CHECK(std::fabs(reached - speed) < speed * 0.03f); // вышли ровно на заданную скорость
}

static void RunAllTests()
{
    GN_RUN(Test_JumpEdges_AreBuiltOnlyUpToMaxJump);
    GN_RUN(Test_JumpHeight_IsPerEntity);
    GN_RUN(Test_Follower_JumpsOntoLedge);
    GN_RUN(Test_MaxDrop_IsPerEntity);
    GN_RUN(Test_ObstacleSet_ClipsTraces);
    GN_RUN(Test_PlacedBox_IsAvoided_AndFreedWhenRemoved);
    GN_RUN(Test_Door_ClosedBlocks_OpenLetsThrough);
    GN_RUN(Test_NodeOnTopOfRemovedBox_Dies);
    GN_RUN(Test_Turns_AreSmoothArcs);
    GN_RUN(Test_Acceleration_RampsUpFromStandstill);
}

GN_TEST_MAIN()
