#include "goldnav_module.h"

#include <algorithm>
#include <cmath>
#include <meta_api.h>
#include <util.h> // ENTINDEX/INDEXENT

#include "logger.h"
#include "config.h"
#include "platform/platform_time.h"
#include "metamod/entity_follow_bridge.h"

namespace goldnav {

namespace {

constexpr int kMaxReplans = 6;            // подряд, без заметного продвижения между ними
constexpr float kReplanForgetTime = 4.0f; // столько спокойной ходьбы обнуляет счётчик
constexpr float kPendingTimeout = 30.0f;
constexpr float kTouchInterval = 0.25f;
constexpr float kTeleportDistance = 96.0f; // entity "переставили" извне
constexpr float kCloseEnoughToGoal = 32.0f;

Vec3 FromEngine(const Vector& v) { return Vec3(v.x, v.y, v.z); }

const char* LayerName(NavLayer layer)
{
    switch (layer)
    {
        case NavLayer::Point:  return "point";
        case NavLayer::Medium: return "32x32";
        case NavLayer::Large:  return "64x64";
        default:               return "?";
    }
}

} // namespace

GoldNavModule& GoldNavModule::Instance()
{
    static GoldNavModule instance;
    return instance;
}

bool GoldNavModule::IsValidEntity(const edict_t* ent)
{
    return ent && !ent->free && ent->pvPrivateData && !(ent->v.flags & FL_KILLME);
}

// --- Генерация навмеша ---

void GoldNavModule::StartLayer(NavLayerData& data, NavLayer layer)
{
    NavGenConfig config;
    config.cellSize = ConfigCellSize();
    config.probe.stepHeight = ConfigStepSize();
    config.probe.maxDrop = ConfigMaxDrop();

    data.generator.Begin(data.grid, m_world, layer, config);
    data.started = true;
    data.initialDone = false;
    data.wasBusy = true;
    data.startedAtMs = PlatformNowMs();

    std::vector<Vec3> seeds;
    CollectSeedPoints(seeds);

    size_t accepted = 0;
    for (const Vec3& seed : seeds)
    {
        if (data.generator.AddSeed(seed))
        {
            ++accepted;
        }
    }

    GOLDNAV_LOG_INFO("navmesh [%s] generation started: cell=%.0f, seeds %u/%u",
                     LayerName(layer), config.cellSize,
                     static_cast<unsigned>(accepted), static_cast<unsigned>(seeds.size()));
}

NavLayerData& GoldNavModule::EnsureLayer(NavLayer layer)
{
    NavLayerData& data = LayerMutable(layer);
    if (!data.started && m_active)
    {
        StartLayer(data, layer);
    }
    return data;
}

void GoldNavModule::TickGeneration()
{
    int busy = 0;
    for (NavLayerData& data : m_layers)
    {
        if (data.started && !data.generator.IsIdle())
        {
            ++busy;
        }
    }

    const double budget = static_cast<double>(ConfigGenerationBudgetMs());

    for (size_t i = 0; i < static_cast<size_t>(NavLayer::Count); ++i)
    {
        NavLayerData& data = m_layers[i];
        if (!data.started)
        {
            continue;
        }

        if (!data.generator.IsIdle())
        {
            data.wasBusy = true;
            data.generator.Tick(budget / (busy > 0 ? busy : 1));
        }

        if (data.generator.IsIdle() && data.wasBusy)
        {
            data.wasBusy = false;
            const double elapsed = PlatformNowMs() - data.startedAtMs;
            if (!data.initialDone)
            {
                data.initialDone = true;
                GOLDNAV_LOG_INFO("navmesh [%s] ready: %u nodes in %u columns, %.1f s%s",
                                 LayerName(static_cast<NavLayer>(i)),
                                 static_cast<unsigned>(data.grid.NodeCount()),
                                 static_cast<unsigned>(data.grid.ColumnCount()), elapsed / 1000.0,
                                 data.generator.HitNodeLimit() ? " (node limit reached!)" : "");
            }
            else
            {
                GOLDNAV_LOG_INFO("navmesh [%s] extended: %u nodes", LayerName(static_cast<NavLayer>(i)),
                                 static_cast<unsigned>(data.grid.NodeCount()));
            }
        }
        else if (!data.generator.IsIdle() && data.initialDone && !data.wasBusy)
        {
            data.wasBusy = true;
        }
    }
}

void GoldNavModule::OnServerActivate()
{
    m_agents.clear();
    m_finished.clear();
    for (NavLayerData& data : m_layers)
    {
        data = NavLayerData();
    }

    m_active = true;
    // Основной слой (хуллы 32x32) нужен почти всем — строим сразу. Остальные — по
    // первому запросу.
    EnsureLayer(NavLayer::Medium);
}

void GoldNavModule::OnServerDeactivate()
{
    m_active = false;
    m_agents.clear();
    m_finished.clear();
    for (NavLayerData& data : m_layers)
    {
        data = NavLayerData();
    }
}

void GoldNavModule::RequestRebuild()
{
    if (!m_active)
    {
        return;
    }

    GOLDNAV_LOG_INFO("navmesh rebuild requested");
    for (size_t i = 0; i < static_cast<size_t>(NavLayer::Count); ++i)
    {
        if (m_layers[i].started)
        {
            StartLayer(m_layers[i], static_cast<NavLayer>(i));
        }
    }
    EnsureLayer(NavLayer::Medium);

    // Пути построены по старому графу — пусть идущие entity перепланируют.
    for (auto& entry : m_agents)
    {
        Agent& agent = entry.second;
        if (agent.state == GOLDNAV_STATE_MOVING || agent.state == GOLDNAV_STATE_PENDING)
        {
            agent.state = GOLDNAV_STATE_PENDING;
            agent.follow.Reset();
            agent.pendingTime = 0.0f;
            agent.startSeeded = false;
            agent.goalSeeded = false;
        }
    }
}

bool GoldNavModule::IsHullReady(int hullType)
{
    if (!m_active)
    {
        return false;
    }
    return EnsureLayer(LayerOfHull(ClampHullIndex(hullType))).initialDone;
}

// --- Поиск пути ---

PlanConfig GoldNavModule::MakePlanConfig(int flags) const
{
    PlanConfig config;
    config.probe.stepHeight = ConfigStepSize();
    config.probe.maxDrop = ConfigMaxDrop();
    config.cost.allowDrops = (flags & GOLDNAV_MOVE_NO_DROP) == 0;
    return config;
}

bool GoldNavModule::FindPath(const Vec3& from, const Vec3& to, int hullType, std::vector<Vec3>& outPath)
{
    outPath.clear();
    if (!m_active)
    {
        return false;
    }

    const HullIndex hull = ClampHullIndex(hullType);
    NavLayerData& layer = EnsureLayer(LayerOfHull(hull));
    if (!layer.initialDone)
    {
        return false;
    }

    PlanConfig config = MakePlanConfig(0);
    config.allowPartial = false;

    PlanResult plan = m_planner.Plan(layer.grid, &m_world, from, to, hull, config);
    if (plan.status == PlanStatus::NoStartNode || plan.status == PlanStatus::NoGoalNode)
    {
        // Рядом со стартом/целью навмеша нет — досеиваем туда; следующий запрос
        // (когда генерация дойдёт) уже найдёт путь.
        layer.generator.AddSeed(plan.status == PlanStatus::NoStartNode ? from : to);
        return false;
    }
    if (plan.status != PlanStatus::Found)
    {
        return false;
    }

    outPath.swap(plan.points);
    return true;
}

// --- Агенты ---

GoldNavModule::Agent* GoldNavModule::BeginAgent(edict_t* ent, int& outIndex)
{
    if (!m_active || !IsValidEntity(ent))
    {
        return nullptr;
    }

    outIndex = ENTINDEX(ent);
    if (outIndex <= 0)
    {
        return nullptr;
    }
    if (outIndex <= gpGlobals->maxClients || (ent->v.flags & FL_CLIENT))
    {
        // Игроков двигает их собственный код движения (usercmd/pmove) — подмена origin
        // на сервере ломала бы клиентское предсказание. Для ботов используйте
        // GoldNav_FindPath и ведите их своими командами.
        static bool warned = false;
        if (!warned)
        {
            warned = true;
            GOLDNAV_LOG_WARN("players/bots cannot be moved by GoldNav (entity #%d); use FindPath for them", outIndex);
        }
        return nullptr;
    }

    Agent& agent = m_agents[outIndex];
    agent = Agent();
    agent.serial = ent->serialnumber;
    return &agent;
}

bool GoldNavModule::MoveEntityTo(edict_t* ent, const Vec3& goal, float speed, int hullType, int flags)
{
    int index = 0;
    Agent* agent = BeginAgent(ent, index);
    if (!agent)
    {
        return false;
    }

    agent->goal = goal;
    agent->speed = speed > 1.0f ? speed : 1.0f;
    agent->hullType = hullType;
    agent->flags = flags;
    agent->state = GOLDNAV_STATE_PENDING;
    return true;
}

void GoldNavModule::SetEntityPath(edict_t* ent, const std::vector<Vec3>& path, float speed)
{
    if (path.empty())
    {
        return;
    }

    int index = 0;
    Agent* agent = BeginAgent(ent, index);
    if (!agent)
    {
        return;
    }

    agent->goal = path.back();
    agent->speed = speed > 1.0f ? speed : 1.0f;
    agent->hullType = GOLDNAV_HULL_AUTO;
    agent->flags = 0;
    agent->follow.SetPath(path);
    agent->state = GOLDNAV_STATE_MOVING;
}

void GoldNavModule::StopEntity(edict_t* ent)
{
    if (!ent)
    {
        return;
    }

    const int index = ENTINDEX(ent);
    const auto it = m_agents.find(index);
    if (it == m_agents.end())
    {
        return;
    }

    if (it->second.state == GOLDNAV_STATE_MOVING)
    {
        HaltEntity(ent);
        if (it->second.mover.initialized && !it->second.mover.onGround)
        {
            ent->v.flags &= ~FL_ONGROUND;
        }
    }

    if (m_updating)
    {
        // Нас позвали из обработчика Touch/колбэка посреди обхода агентов — удалять
        // запись сейчас нельзя, на неё могут держать ссылку выше по стеку.
        it->second.state = GOLDNAV_STATE_IDLE;
        it->second.follow.Reset();
        it->second.removeRequested = true;
    }
    else
    {
        m_agents.erase(it);
    }
}

int GoldNavModule::GetEntityState(edict_t* ent) const
{
    if (!ent)
    {
        return GOLDNAV_STATE_IDLE;
    }

    const auto it = m_agents.find(ENTINDEX(ent));
    if (it == m_agents.end() || it->second.serial != ent->serialnumber)
    {
        return GOLDNAV_STATE_IDLE;
    }
    return it->second.state;
}

void GoldNavModule::CollectActivePaths(std::vector<std::vector<Vec3>>& outPaths) const
{
    outPaths.clear();
    for (const auto& entry : m_agents)
    {
        const Agent& agent = entry.second;
        if (agent.state != GOLDNAV_STATE_MOVING || !agent.follow.active)
        {
            continue;
        }

        edict_t* ent = INDEXENT(entry.first);
        if (!IsValidEntity(ent))
        {
            continue;
        }

        std::vector<Vec3> points;
        points.push_back(FromEngine(ent->v.origin) + ComputeEntityHull(ent).feetOffset);
        for (size_t i = agent.follow.currentIndex; i < agent.follow.path.size(); ++i)
        {
            points.push_back(agent.follow.path[i]);
        }
        outPaths.push_back(points);
    }
}

void GoldNavModule::FinishAgent(int entIndex, edict_t* ent, Agent& agent, int state)
{
    agent.state = state;
    agent.follow.Reset();
    if (ent)
    {
        HaltEntity(ent);
        // Бросили entity в воздухе — снимаем "стою на земле", чтобы её уронил движок
        // (пока вели, флаг держали мы сами, см. entity_follow_bridge.h).
        if (agent.mover.initialized && !agent.mover.onGround)
        {
            ent->v.flags &= ~FL_ONGROUND;
        }
    }
    m_finished.push_back(FinishedEvent{ entIndex, state });
}

void GoldNavModule::PlanAgent(edict_t* ent, Agent& agent, const EntityHullInfo& hull)
{
    const int entIndex = ENTINDEX(ent);

    // Каким хуллом планировать: заданным явно или тем, которым entity реально
    // сталкивается с миром.
    HullIndex planHull = hull.hull;
    if (agent.hullType >= 0 && agent.hullType < static_cast<int>(HullIndex::Count))
    {
        planHull = static_cast<HullIndex>(agent.hullType);
    }

    NavLayerData& layer = EnsureLayer(LayerOfHull(planHull));
    if (!layer.initialDone)
    {
        agent.state = GOLDNAV_STATE_PENDING;
        return;
    }

    const Vec3 feet = FromEngine(ent->v.origin) + hull.feetOffset;

    PlanConfig config = MakePlanConfig(agent.flags);
    PlanResult plan = m_planner.Plan(layer.grid, &m_world, feet, agent.goal, planHull, config);

    if (plan.status == PlanStatus::NoStartNode || plan.status == PlanStatus::NoGoalNode)
    {
        // Навмеш сюда не дошёл (entity заспавнили на изолированном островке, цель в
        // неисследованной зоне) — досеиваем и ждём, пока генератор дорастит граф.
        bool& seeded = (plan.status == PlanStatus::NoStartNode) ? agent.startSeeded : agent.goalSeeded;
        if (!seeded)
        {
            seeded = true;
            layer.generator.AddSeed(plan.status == PlanStatus::NoStartNode ? feet : agent.goal);
        }

        if (layer.generator.IsIdle())
        {
            GOLDNAV_LOG_WARN("entity #%d: no navmesh near %s (%.0f %.0f %.0f)", entIndex,
                             plan.status == PlanStatus::NoStartNode ? "start" : "goal",
                             plan.status == PlanStatus::NoStartNode ? feet.x : agent.goal.x,
                             plan.status == PlanStatus::NoStartNode ? feet.y : agent.goal.y,
                             plan.status == PlanStatus::NoStartNode ? feet.z : agent.goal.z);
            FinishAgent(entIndex, ent, agent, GOLDNAV_STATE_FAILED);
        }
        else
        {
            agent.state = GOLDNAV_STATE_PENDING;
        }
        return;
    }

    if (plan.status == PlanStatus::Partial && !agent.goalSeeded)
    {
        // Цель недостижима по текущему графу. Возможно, граф туда просто ещё не дорос —
        // один раз досеиваем от цели и перепланируем, когда генератор закончит.
        agent.goalSeeded = true;
        layer.generator.AddSeed(agent.goal);
        if (!layer.generator.IsIdle())
        {
            agent.state = GOLDNAV_STATE_PENDING;
            return;
        }
    }

    if (!plan.HasPath() || (plan.status == PlanStatus::Partial && (agent.flags & GOLDNAV_MOVE_EXACT)))
    {
        FinishAgent(entIndex, ent, agent, GOLDNAV_STATE_FAILED);
        return;
    }

    agent.follow.SetPath(plan.points);
    agent.state = GOLDNAV_STATE_MOVING;
    agent.pendingTime = 0.0f;
}

void GoldNavModule::UpdateAgent(int entIndex, Agent& agent, float frameTime)
{
    if (agent.state != GOLDNAV_STATE_PENDING && agent.state != GOLDNAV_STATE_MOVING)
    {
        return;
    }

    edict_t* ent = INDEXENT(entIndex);
    if (!IsValidEntity(ent) || ent->serialnumber != agent.serial)
    {
        // Entity удалили (или слот уже занят другой) — молча забываем.
        agent.state = GOLDNAV_STATE_IDLE;
        agent.removeRequested = true;
        return;
    }

    const EntityHullInfo hull = ComputeEntityHull(ent);
    const Vec3 feet = FromEngine(ent->v.origin) + hull.feetOffset;

    // Entity переставили извне (телепорт, плагин) — наше представление о её
    // положении и путь больше не годятся.
    if (agent.hasLastFeet && Distance(feet, agent.lastFeet) > kTeleportDistance)
    {
        agent.mover = MoverState();
        if (agent.state == GOLDNAV_STATE_MOVING)
        {
            agent.state = GOLDNAV_STATE_PENDING;
            agent.follow.Reset();
        }
    }

    if (agent.state == GOLDNAV_STATE_PENDING)
    {
        agent.pendingTime += frameTime;
        if (agent.pendingTime > kPendingTimeout)
        {
            GOLDNAV_LOG_WARN("entity #%d: gave up waiting for a path", entIndex);
            FinishAgent(entIndex, ent, agent, GOLDNAV_STATE_FAILED);
            return;
        }

        PlanAgent(ent, agent, hull);
        if (agent.state != GOLDNAV_STATE_MOVING)
        {
            agent.lastFeet = feet;
            agent.hasLastFeet = true;
            return;
        }
    }

    // --- Движение ---
    PathFollowerConfig& followerConfig = m_follower.Config();
    followerConfig.mover.stepHeight = ConfigStepSize();
    followerConfig.mover.gravity = ConfigGravity() * (ent->v.gravity != 0.0f ? ent->v.gravity : 1.0f);

    EntityMoveTracer tracer(ent, hull.feetOffset);
    const FollowStep step = m_follower.Update(agent.follow, tracer, agent.mover, feet, agent.speed, frameTime);

    if (step.result != FollowResult::NoPath)
    {
        ApplyFollowStep(ent, hull, step, agent.mover, (agent.flags & GOLDNAV_MOVE_NO_TURN) == 0, frameTime);
    }
    agent.lastFeet = step.feet;
    agent.hasLastFeet = true;

    agent.sinceReplan += frameTime;
    if (agent.sinceReplan > kReplanForgetTime)
    {
        agent.replans = 0;
    }

    switch (step.result)
    {
        case FollowResult::Moving:
            break;

        case FollowResult::Arrived:
            FinishAgent(entIndex, ent, agent, GOLDNAV_STATE_ARRIVED);
            break;

        case FollowResult::Stuck:
        case FollowResult::OffPath:
        case FollowResult::NoPath:
            if (ConfigDebugDrawEnabled())
            {
                const bool hasTarget = agent.follow.currentIndex < agent.follow.path.size();
                const Vec3 target = hasTarget ? agent.follow.path[agent.follow.currentIndex] : Vec3();
                GOLDNAV_LOG_INFO("entity #%d: replan #%d (%s) at (%.1f %.1f %.1f), waypoint %u/%u (%.1f %.1f %.1f), onGround=%d, blocked=%d by #%d normal (%.2f %.2f %.2f)",
                                 entIndex, agent.replans + 1,
                                 step.result == FollowResult::Stuck ? "stuck" : (step.result == FollowResult::OffPath ? "off path" : "no path"),
                                 step.feet.x, step.feet.y, step.feet.z,
                                 static_cast<unsigned>(agent.follow.currentIndex), static_cast<unsigned>(agent.follow.path.size()),
                                 target.x, target.y, target.z, agent.mover.onGround ? 1 : 0,
                                 step.blocked ? 1 : 0, step.blockEntity, step.blockNormal.x, step.blockNormal.y, step.blockNormal.z);
            }
            ++agent.replans;
            agent.sinceReplan = 0.0f;
            if (agent.replans > kMaxReplans)
            {
                // Не дошли совсем чуть-чуть (цель у самой стены, на ней стоит игрок) —
                // это "пришли", а не провал.
                const bool closeEnough = Distance2D(step.feet, agent.goal) <= kCloseEnoughToGoal &&
                                         std::fabs(step.feet.z - agent.goal.z) <= 48.0f;
                if (!closeEnough)
                {
                    GOLDNAV_LOG_WARN("entity #%d: stuck at (%.0f %.0f %.0f), giving up", entIndex,
                                     step.feet.x, step.feet.y, step.feet.z);
                }
                FinishAgent(entIndex, ent, agent, closeEnough ? GOLDNAV_STATE_ARRIVED : GOLDNAV_STATE_FAILED);
            }
            else
            {
                agent.state = GOLDNAV_STATE_PENDING;
                PlanAgent(ent, agent, hull);
            }
            break;
    }

    // Touch — в самом конце: обработчик в игровой DLL/плагине может вызвать наши же
    // натусы (остановить эту entity, отправить её в другое место) или удалить её.
    agent.touchCooldown -= frameTime;
    if (step.touchedEntity > 0 && agent.touchCooldown <= 0.0f && (agent.flags & GOLDNAV_MOVE_NO_TOUCH) == 0)
    {
        agent.touchCooldown = kTouchInterval;
        EmulateTouch(ent, step.touchedEntity);
    }
}

void GoldNavModule::OnStartFrame(float frameTime)
{
    if (!m_active)
    {
        return;
    }

    TickGeneration();

    if (!m_agents.empty())
    {
        // Обходим по снимку ключей: Touch-обработчики могут добавлять агентов
        // (перехеширование таблицы инвалидирует итераторы, но не ссылки на элементы).
        m_updateOrder.clear();
        for (const auto& entry : m_agents)
        {
            m_updateOrder.push_back(entry.first);
        }

        m_updating = true;
        for (int entIndex : m_updateOrder)
        {
            const auto it = m_agents.find(entIndex);
            if (it != m_agents.end())
            {
                UpdateAgent(entIndex, it->second, frameTime);
            }
        }
        m_updating = false;

        for (auto it = m_agents.begin(); it != m_agents.end(); )
        {
            if (it->second.removeRequested)
            {
                it = m_agents.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    if (!m_finished.empty())
    {
        // Колбэки — после обхода: обработчик вправе тут же отправить entity дальше.
        std::vector<FinishedEvent> events;
        events.swap(m_finished);
        if (m_callback)
        {
            for (const FinishedEvent& event : events)
            {
                m_callback(event.entityIndex, event.state);
            }
        }
    }
}

} // namespace goldnav

// --- Публичный C API (см. goldnav_api.h) ---

using namespace goldnav;

namespace {

bool WritePath(const std::vector<Vec3>& path, Vector* outPath, int outPathMaxLen, int* outPathLen)
{
    const int count = std::min(static_cast<int>(path.size()), outPathMaxLen);
    for (int i = 0; i < count; ++i)
    {
        outPath[i] = Vector(path[i].x, path[i].y, path[i].z);
    }
    if (outPathLen) *outPathLen = count;
    return count > 0;
}

} // namespace

bool GoldNav_FindPath(const Vector* from, const Vector* to,
                       Vector* outPath, int outPathMaxLen, int* outPathLen,
                       int hullType)
{
    if (outPathLen) *outPathLen = 0;
    if (!from || !to || !outPath || outPathMaxLen <= 0)
    {
        return false;
    }

    std::vector<Vec3> path;
    if (!GoldNavModule::Instance().FindPath(Vec3(from->x, from->y, from->z), Vec3(to->x, to->y, to->z),
                                            hullType == GOLDNAV_HULL_AUTO ? GOLDNAV_HULL_STANDING : hullType, path))
    {
        return false;
    }
    return WritePath(path, outPath, outPathMaxLen, outPathLen);
}

bool GoldNav_FindPathToEntity(edict_t* entity, const Vector* from,
                               Vector* outPath, int outPathMaxLen, int* outPathLen,
                               int hullType)
{
    if (outPathLen) *outPathLen = 0;
    if (!entity || entity->free)
    {
        return false;
    }

    const Vector to = entity->v.origin;
    return GoldNav_FindPath(from, &to, outPath, outPathMaxLen, outPathLen, hullType);
}

bool GoldNav_MoveEntityTo(edict_t* entity, const Vector* goal, float speed, int hullType, int flags)
{
    if (!entity || !goal)
    {
        return false;
    }
    return GoldNavModule::Instance().MoveEntityTo(entity, Vec3(goal->x, goal->y, goal->z), speed, hullType, flags);
}

void GoldNav_MoveEntityAlongPath(edict_t* entity, const Vector* path, int pathLen, float speed)
{
    if (!entity || !path || pathLen <= 0)
    {
        return;
    }

    std::vector<Vec3> vpath;
    vpath.reserve(static_cast<size_t>(pathLen));
    for (int i = 0; i < pathLen; ++i)
    {
        vpath.emplace_back(path[i].x, path[i].y, path[i].z);
    }

    GoldNavModule::Instance().SetEntityPath(entity, vpath, speed);
}

void GoldNav_StopFollowing(edict_t* entity)
{
    GoldNavModule::Instance().StopEntity(entity);
}

int GoldNav_GetEntityState(edict_t* entity)
{
    return GoldNavModule::Instance().GetEntityState(entity);
}

bool GoldNav_IsNavMeshReady(void)
{
    return GoldNavModule::Instance().IsReady();
}

bool GoldNav_IsHullReady(int hullType)
{
    return GoldNavModule::Instance().IsHullReady(hullType == GOLDNAV_HULL_AUTO ? GOLDNAV_HULL_STANDING : hullType);
}

int GoldNav_GetVersion(void)
{
    constexpr int kMajor = 1;
    constexpr int kMinor = 0;
    constexpr int kPatch = 0;
    return kMajor * 10000 + kMinor * 100 + kPatch;
}

void GoldNav_SetMoveCallback(GoldNavMoveCallback callback)
{
    GoldNavModule::Instance().SetMoveCallback(callback);
}

edict_t* GoldNav_EdictByIndex(int index)
{
    if (!gpGlobals || index < 0 || index >= gpGlobals->maxEntities)
    {
        return nullptr;
    }
    edict_t* ent = INDEXENT(index);
    if (!ent || ent->free)
    {
        return nullptr;
    }
    return ent;
}
