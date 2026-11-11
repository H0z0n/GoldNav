// Общие помощники тестов: сборка навмеша по коробочному миру и симуляция
// "entity идёт из A в B" тем же конвейером, что и боевой модуль:
// планировщик -> PathFollower -> GroundMove, с перестроением пути при Stuck/OffPath.
#pragma once

#include <vector>
#include "box_world.h"
#include "navmesh/nav_grid.h"
#include "navmesh/nav_generator.h"
#include "navmesh/nav_path_planner.h"
#include "pathfollower/path_follower.h"

namespace goldnav_test {

inline void BuildNav(const BoxWorld& world, goldnav::NavGrid& grid, goldnav::HullIndex hull,
                     const std::vector<goldnav::Vec3>& seeds,
                     const goldnav::NavGenConfig& config = goldnav::NavGenConfig())
{
    goldnav::NavMeshGenerator generator;
    generator.Begin(grid, world, goldnav::LayerOfHull(hull), config);
    for (const goldnav::Vec3& seed : seeds)
    {
        generator.AddSeed(seed);
    }
    generator.RunToCompletion();
}

struct SimResult
{
    bool arrived = false;
    bool gaveUp = false;
    goldnav::Vec3 finalFeet;
    float time = 0.0f;
    int plans = 0;
    float maxZ = -1e30f;
    float minZ = 1e30f;
    goldnav::PlanStatus lastStatus = goldnav::PlanStatus::NoPath;
    std::vector<goldnav::Vec3> trail; // позиции раз в ~0.1 с
};

inline SimResult SimulateMove(const BoxWorld& world, const goldnav::NavGrid& grid,
                              const goldnav::Vec3& from, const goldnav::Vec3& to,
                              goldnav::HullIndex hull, float speed = 200.0f, float dt = 0.02f,
                              float maxTime = 60.0f, int maxPlans = 6)
{
    using namespace goldnav;

    SimResult sim;
    NavPathPlanner planner;
    PlanConfig planConfig;
    PathFollower follower;
    BoxMoveTracer tracer(world, hull);

    FollowState follow;
    MoverState mover;
    Vec3 feet = from;
    float trailTimer = 0.0f;

    auto replan = [&]() -> bool {
        if (sim.plans >= maxPlans)
        {
            return false;
        }
        ++sim.plans;
        const PlanResult plan = planner.Plan(grid, &world, feet, to, hull, planConfig);
        sim.lastStatus = plan.status;
        if (!plan.HasPath())
        {
            return false;
        }
        follow.SetPath(plan.points);
        return true;
    };

    if (!replan())
    {
        sim.gaveUp = true;
        sim.finalFeet = feet;
        return sim;
    }

    while (sim.time < maxTime)
    {
        const FollowStep step = follower.Update(follow, tracer, mover, feet, speed, dt);
        feet = step.feet;
        sim.time += dt;
        sim.maxZ = std::max(sim.maxZ, feet.z);
        sim.minZ = std::min(sim.minZ, feet.z);

        trailTimer += dt;
        if (trailTimer >= 0.1f)
        {
            trailTimer = 0.0f;
            sim.trail.push_back(feet);
        }

        if (step.result == FollowResult::Arrived)
        {
            sim.arrived = true;
            break;
        }
        if (step.result == FollowResult::Stuck || step.result == FollowResult::OffPath ||
            step.result == FollowResult::NoPath)
        {
            if (!replan())
            {
                sim.gaveUp = true;
                break;
            }
        }
    }

    sim.finalFeet = feet;
    return sim;
}

// Комната: пол z=0 на [-half, half]^2 и стены высотой wallHeight по периметру.
inline void AddRoom(BoxWorld& world, float half, float wallHeight = 256.0f, float floorZ = 0.0f)
{
    using goldnav::Vec3;
    world.AddSolid(Vec3(-half - 64, -half - 64, floorZ - 64), Vec3(half + 64, half + 64, floorZ));
    world.AddSolid(Vec3(-half - 64, -half - 64, floorZ), Vec3(-half, half + 64, floorZ + wallHeight));
    world.AddSolid(Vec3(half, -half - 64, floorZ), Vec3(half + 64, half + 64, floorZ + wallHeight));
    world.AddSolid(Vec3(-half - 64, -half - 64, floorZ), Vec3(half + 64, -half, floorZ + wallHeight));
    world.AddSolid(Vec3(-half - 64, half, floorZ), Vec3(half + 64, half + 64, floorZ + wallHeight));
}

} // namespace goldnav_test
