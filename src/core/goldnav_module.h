// GoldNavModule — синглтон, владеющий слоями навмеша, генераторами, планировщиком и
// "агентами" (entity, которых GoldNav сейчас ведёт). Единственная точка входа для
// metamod/hooks.cpp и для реализации публичного C API (см. goldnav_module.cpp).
#pragma once

#include <vector>
#include <unordered_map>

#include <extdll.h>

#include "goldnav_api.h"
#include "navmesh/nav_grid.h"
#include "navmesh/nav_generator.h"
#include "navmesh/nav_path_planner.h"
#include "pathfollower/path_follower.h"
#include "metamod/engine_collision_world.h"

namespace goldnav {

struct NavLayerData
{
    NavGrid grid;
    NavMeshGenerator generator;
    bool started = false;     // генерация запускалась
    bool initialDone = false; // первичная заливка от спавнов завершена
    bool wasBusy = false;
    double startedAtMs = 0.0;
};

class GoldNavModule
{
public:
    static GoldNavModule& Instance();

    void OnServerActivate();
    void OnServerDeactivate();
    void OnStartFrame(float frameTime);

    bool IsReady() const { return Layer(NavLayer::Medium).initialDone; }
    bool IsHullReady(int hullType);

    bool FindPath(const Vec3& from, const Vec3& to, int hullType, std::vector<Vec3>& outPath);

    bool MoveEntityTo(edict_t* ent, const Vec3& goal, float speed, int hullType, int flags);
    void SetEntityPath(edict_t* ent, const std::vector<Vec3>& path, float speed);
    void StopEntity(edict_t* ent);
    int GetEntityState(edict_t* ent) const;

    void SetMoveCallback(GoldNavMoveCallback callback) { m_callback = callback; }

    void RequestRebuild();

    const NavLayerData& Layer(NavLayer layer) const { return m_layers[static_cast<size_t>(layer)]; }

    // Для отладочной отрисовки: оставшиеся точки путей всех идущих entity.
    void CollectActivePaths(std::vector<std::vector<Vec3>>& outPaths) const;
    size_t AgentCount() const { return m_agents.size(); }

private:
    GoldNavModule() = default;

    struct Agent
    {
        int serial = 0; // edict_t::serialnumber на момент запроса: защита от переиспользования слота
        int state = GOLDNAV_STATE_IDLE;
        Vec3 goal;
        float speed = 0.0f;
        int hullType = GOLDNAV_HULL_AUTO;
        int flags = 0;

        FollowState follow;
        MoverState mover;
        Vec3 lastFeet;
        bool hasLastFeet = false;

        bool startSeeded = false;
        bool goalSeeded = false;
        int replans = 0;
        float sinceReplan = 0.0f;
        float pendingTime = 0.0f;
        float touchCooldown = 0.0f;
        bool removeRequested = false;
    };

    struct FinishedEvent
    {
        int entityIndex;
        int state;
    };

    NavLayerData& LayerMutable(NavLayer layer) { return m_layers[static_cast<size_t>(layer)]; }
    NavLayerData& EnsureLayer(NavLayer layer);
    void StartLayer(NavLayerData& data, NavLayer layer);
    void TickGeneration();

    PlanConfig MakePlanConfig(int flags) const;
    void UpdateAgent(int entIndex, Agent& agent, float frameTime);
    void PlanAgent(edict_t* ent, Agent& agent, const EntityHullInfo& hull);
    void FinishAgent(int entIndex, edict_t* ent, Agent& agent, int state);

    Agent* BeginAgent(edict_t* ent, int& outIndex);
    static bool IsValidEntity(const edict_t* ent);

    NavLayerData m_layers[static_cast<size_t>(NavLayer::Count)];
    EngineCollisionWorld m_world;
    NavPathPlanner m_planner;
    PathFollower m_follower;

    std::unordered_map<int, Agent> m_agents; // ключ — ENTINDEX
    std::vector<int> m_updateOrder;
    std::vector<FinishedEvent> m_finished;
    bool m_updating = false;
    bool m_active = false;
    GoldNavMoveCallback m_callback = nullptr;
};

} // namespace goldnav
