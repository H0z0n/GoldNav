#include "natives_navmesh.h"

// amxxmodule.h — главный заголовок AMXX SDK (public/sdk). USE_METAMOD НЕ определён
// (см. moduleconfig.h) — этот модуль не хукает Metamod/движок сам, поэтому не имеет
// своего g_engfuncs; для entity index -> edict_t* используется GoldNav_EdictByIndex
// из goldnav_mm (единственное место с реальным доступом к движку).
#include <amxxmodule.h>

// Из goldnav_api.h нужны только типы HLSDK (Vector/edict_t) и константы. goldnav_amxx
// НЕ линкуется на goldnav_mm: функции GoldNav_* ищутся в уже загруженном goldnav_mm
// при первом вызове натива (см. BindApi ниже).
#include "core/goldnav_api.h"

#include <vector>

#if !defined(_WIN32)
    #include <dlfcn.h>
#endif
// На Windows windows.h уже подключён через extdll.h (goldnav_api.h).

namespace goldnav {

namespace {

// Причина отказа от линковки на goldnav_mm: AMXX загружает модули РАНЬШЕ, чем Metamod
// подключает goldnav_mm, и не знает, где на конкретном сервере лежит goldnav_mm.so —
// при DT_NEEDED-зависимости dlopen() модуля падал (воспроизведено на живом сервере).
// Вместо этого функции берутся через dlsym/GetProcAddress из уже загруженного
// goldnav_mm в момент первого вызова натива.
struct GoldNavApi
{
    bool (*FindPath)(const Vector*, const Vector*, Vector*, int, int*, int) = nullptr;
    bool (*FindPathToEntity)(edict_t*, const Vector*, Vector*, int, int*, int) = nullptr;
    bool (*MoveEntityTo)(edict_t*, const Vector*, float, int, int) = nullptr;
    void (*MoveEntityAlongPath)(edict_t*, const Vector*, int, float) = nullptr;
    void (*StopFollowing)(edict_t*) = nullptr;
    int (*GetEntityState)(edict_t*) = nullptr;
    bool (*IsNavMeshReady)() = nullptr;
    bool (*IsHullReady)(int) = nullptr;
    void (*SetMoveCallback)(GoldNavMoveCallback) = nullptr;
    edict_t* (*EdictByIndex)(int) = nullptr;
};

GoldNavApi g_api;
bool g_apiBound = false;
int g_fwdMoveFinished = -1;

#if defined(_WIN32)
void* ResolveSymbol(const char* name)
{
    HMODULE module = GetModuleHandleA("goldnav_mm.dll");
    if (!module)
    {
        module = GetModuleHandleA("libgoldnav_mm.dll"); // mingw-сборка добавляет префикс lib
    }
    return module ? reinterpret_cast<void*>(GetProcAddress(module, name)) : nullptr;
}
#else
void* ResolveSymbol(const char* name)
{
    static void* handle = nullptr;
    if (!handle)
    {
        // RTLD_NOLOAD: только найти уже загруженный Metamod-ом goldnav_mm (по SONAME
        // "goldnav_mm.so", независимо от директории), ничего не загружая самим.
        handle = dlopen("goldnav_mm.so", RTLD_NOW | RTLD_NOLOAD);
    }
    return handle ? dlsym(handle, name) : nullptr;
}
#endif

template <typename T>
bool Resolve(T& target, const char* name)
{
    target = reinterpret_cast<T>(ResolveSymbol(name));
    return target != nullptr;
}

// Вызывается goldnav_mm, когда ведение entity закончилось -> forward в плагины.
void OnMoveFinished(int entityIndex, int state)
{
    if (g_fwdMoveFinished >= 0)
    {
        MF_ExecuteForward(g_fwdMoveFinished, static_cast<cell>(entityIndex), static_cast<cell>(state));
    }
}

bool BindApi()
{
    if (g_apiBound)
    {
        return true;
    }

    GoldNavApi api;
    bool ok = true;
    ok = Resolve(api.FindPath, "GoldNav_FindPath") && ok;
    ok = Resolve(api.FindPathToEntity, "GoldNav_FindPathToEntity") && ok;
    ok = Resolve(api.MoveEntityTo, "GoldNav_MoveEntityTo") && ok;
    ok = Resolve(api.MoveEntityAlongPath, "GoldNav_MoveEntityAlongPath") && ok;
    ok = Resolve(api.StopFollowing, "GoldNav_StopFollowing") && ok;
    ok = Resolve(api.GetEntityState, "GoldNav_GetEntityState") && ok;
    ok = Resolve(api.IsNavMeshReady, "GoldNav_IsNavMeshReady") && ok;
    ok = Resolve(api.IsHullReady, "GoldNav_IsHullReady") && ok;
    ok = Resolve(api.SetMoveCallback, "GoldNav_SetMoveCallback") && ok;
    ok = Resolve(api.EdictByIndex, "GoldNav_EdictByIndex") && ok;
    if (!ok)
    {
        return false;
    }

    g_api = api;
    g_apiBound = true;
    g_api.SetMoveCallback(OnMoveFinished);
    return true;
}

// Для натусов, которые не могут вернуть осмысленный результат без goldnav_mm.
bool RequireApi(AMX* amx, const char* nativeName)
{
    if (BindApi())
    {
        return true;
    }
    MF_LogError(amx, AMX_ERR_NATIVE,
                "%s: goldnav_mm is not loaded by Metamod, or its version does not match goldnav_amxx "
                "(check plugins.ini / `meta list`)", nativeName);
    return false;
}

edict_t* RequireEntity(AMX* amx, const char* nativeName, cell index)
{
    edict_t* entity = g_api.EdictByIndex(static_cast<int>(index));
    if (!entity)
    {
        MF_LogError(amx, AMX_ERR_NATIVE, "%s: invalid entity index %d", nativeName, static_cast<int>(index));
    }
    return entity;
}

Vector ReadVector(AMX* amx, cell param)
{
    cell* addr = MF_GetAmxAddr(amx, param);
    return Vector(amx_ctof(addr[0]), amx_ctof(addr[1]), amx_ctof(addr[2]));
}

// Верхний предел длины пути на один вызов — соответствует NavMesh::kMaxPathLength.
constexpr int kNativeMaxPathLength = 256;

// Общая часть find_path/find_path_to_entity: вызвать поиск и разложить путь в плоский
// cell-массив x0,y0,z0, x1,y1,z1, ...
template <typename TSearch>
cell RunPathSearch(AMX* amx, cell outPathParam, cell maxLenParam, cell outLenParam, TSearch search)
{
    cell* outLenAddr = MF_GetAmxAddr(amx, outLenParam);
    *outLenAddr = 0;

    const int requested = static_cast<int>(maxLenParam);
    const int maxLen = requested > kNativeMaxPathLength ? kNativeMaxPathLength : requested;
    if (maxLen <= 0)
    {
        return 0;
    }

    std::vector<Vector> path(static_cast<size_t>(maxLen));
    int actualLen = 0;
    if (!search(path.data(), maxLen, &actualLen))
    {
        return 0;
    }

    cell* outPathAddr = MF_GetAmxAddr(amx, outPathParam);
    for (int i = 0; i < actualLen; ++i)
    {
        outPathAddr[i * 3 + 0] = amx_ftoc(path[static_cast<size_t>(i)].x);
        outPathAddr[i * 3 + 1] = amx_ftoc(path[static_cast<size_t>(i)].y);
        outPathAddr[i * 3 + 2] = amx_ftoc(path[static_cast<size_t>(i)].z);
    }
    *outLenAddr = actualLen;
    return 1;
}

// native bool:goldnav_find_path(const Float:from[3], const Float:to[3], Float:outPath[], maxLen, &outLen, hull = GOLDNAV_HULL_STANDING);
// outPath — ПЛОСКИЙ массив длиной maxLen*3, НЕ Float[][3]: 2D-массивы как параметры
// натусов в Pawn устроены через таблицу смещений, плоский массив этого избегает.
cell AMX_NATIVE_CALL native_goldnav_find_path(AMX* amx, cell* params)
{
    enum { arg_from = 1, arg_to, arg_outpath, arg_maxlen, arg_outlen, arg_hull };

    if (!RequireApi(amx, "goldnav_find_path"))
    {
        *MF_GetAmxAddr(amx, params[arg_outlen]) = 0;
        return 0;
    }

    const Vector from = ReadVector(amx, params[arg_from]);
    const Vector to = ReadVector(amx, params[arg_to]);
    const int hull = static_cast<int>(params[arg_hull]);

    return RunPathSearch(amx, params[arg_outpath], params[arg_maxlen], params[arg_outlen],
                         [&](Vector* out, int maxLen, int* outLen) {
                             return g_api.FindPath(&from, &to, out, maxLen, outLen, hull);
                         });
}

// native bool:goldnav_find_path_to_entity(entity, const Float:from[3], Float:outPath[], maxLen, &outLen, hull = GOLDNAV_HULL_STANDING);
cell AMX_NATIVE_CALL native_goldnav_find_path_to_entity(AMX* amx, cell* params)
{
    enum { arg_entity = 1, arg_from, arg_outpath, arg_maxlen, arg_outlen, arg_hull };

    *MF_GetAmxAddr(amx, params[arg_outlen]) = 0;
    if (!RequireApi(amx, "goldnav_find_path_to_entity"))
    {
        return 0;
    }

    edict_t* entity = RequireEntity(amx, "goldnav_find_path_to_entity", params[arg_entity]);
    if (!entity)
    {
        return 0;
    }

    const Vector from = ReadVector(amx, params[arg_from]);
    const int hull = static_cast<int>(params[arg_hull]);

    return RunPathSearch(amx, params[arg_outpath], params[arg_maxlen], params[arg_outlen],
                         [&](Vector* out, int maxLen, int* outLen) {
                             return g_api.FindPathToEntity(entity, &from, out, maxLen, outLen, hull);
                         });
}

// native bool:goldnav_move_to(entity, const Float:goal[3], Float:speed = 200.0, hull = GOLDNAV_HULL_AUTO, flags = 0);
cell AMX_NATIVE_CALL native_goldnav_move_to(AMX* amx, cell* params)
{
    enum { arg_entity = 1, arg_goal, arg_speed, arg_hull, arg_flags };

    if (!RequireApi(amx, "goldnav_move_to"))
    {
        return 0;
    }

    edict_t* entity = RequireEntity(amx, "goldnav_move_to", params[arg_entity]);
    if (!entity)
    {
        return 0;
    }

    const Vector goal = ReadVector(amx, params[arg_goal]);
    return g_api.MoveEntityTo(entity, &goal, amx_ctof(params[arg_speed]),
                              static_cast<int>(params[arg_hull]), static_cast<int>(params[arg_flags])) ? 1 : 0;
}

// native goldnav_move_along_path(entity, const Float:path[], pathLen, Float:speed);
// path — плоский массив длиной pathLen*3 (см. комментарий над find_path).
cell AMX_NATIVE_CALL native_goldnav_move_along_path(AMX* amx, cell* params)
{
    enum { arg_entity = 1, arg_path, arg_pathlen, arg_speed };

    if (!RequireApi(amx, "goldnav_move_along_path"))
    {
        return 0;
    }

    edict_t* entity = RequireEntity(amx, "goldnav_move_along_path", params[arg_entity]);
    if (!entity)
    {
        return 0;
    }

    const int pathLen = static_cast<int>(params[arg_pathlen]);
    if (pathLen <= 0 || pathLen > 4096)
    {
        return 0;
    }

    cell* pathAddr = MF_GetAmxAddr(amx, params[arg_path]);
    std::vector<Vector> path(static_cast<size_t>(pathLen));
    for (int i = 0; i < pathLen; ++i)
    {
        path[static_cast<size_t>(i)] = Vector(
            amx_ctof(pathAddr[i * 3 + 0]),
            amx_ctof(pathAddr[i * 3 + 1]),
            amx_ctof(pathAddr[i * 3 + 2]));
    }

    g_api.MoveEntityAlongPath(entity, path.data(), pathLen, amx_ctof(params[arg_speed]));
    return 1;
}

// native goldnav_stop_following(entity);
cell AMX_NATIVE_CALL native_goldnav_stop_following(AMX* amx, cell* params)
{
    enum { arg_entity = 1 };

    if (!RequireApi(amx, "goldnav_stop_following"))
    {
        return 0;
    }

    // Entity могли уже удалить — это не ошибка: останавливать просто нечего.
    edict_t* entity = g_api.EdictByIndex(static_cast<int>(params[arg_entity]));
    if (!entity)
    {
        return 0;
    }

    g_api.StopFollowing(entity);
    return 1;
}

// native goldnav_get_state(entity);
cell AMX_NATIVE_CALL native_goldnav_get_state(AMX* /*amx*/, cell* params)
{
    enum { arg_entity = 1 };

    if (!BindApi())
    {
        return GOLDNAV_STATE_IDLE;
    }

    edict_t* entity = g_api.EdictByIndex(static_cast<int>(params[arg_entity]));
    return entity ? g_api.GetEntityState(entity) : GOLDNAV_STATE_IDLE;
}

// native bool:goldnav_is_ready(hull = GOLDNAV_HULL_STANDING);
cell AMX_NATIVE_CALL native_goldnav_is_ready(AMX* /*amx*/, cell* params)
{
    // Без goldnav_mm (ещё не загружен/не подключён) — просто "не готов", не ошибка:
    // плагины опрашивают этот натив перед использованием остальных.
    if (!BindApi())
    {
        return 0;
    }

    // Старые плагины вызывают goldnav_is_ready() без аргументов.
    const int argCount = static_cast<int>(params[0] / sizeof(cell));
    if (argCount < 1)
    {
        return g_api.IsNavMeshReady() ? 1 : 0;
    }
    return g_api.IsHullReady(static_cast<int>(params[1])) ? 1 : 0;
}

AMX_NATIVE_INFO g_NavmeshNatives[] = {
    { "goldnav_find_path",           native_goldnav_find_path },
    { "goldnav_find_path_to_entity", native_goldnav_find_path_to_entity },
    { "goldnav_move_to",             native_goldnav_move_to },
    { "goldnav_move_along_path",     native_goldnav_move_along_path },
    { "goldnav_stop_following",      native_goldnav_stop_following },
    { "goldnav_get_state",           native_goldnav_get_state },
    { "goldnav_is_ready",            native_goldnav_is_ready },
    { nullptr, nullptr },
};

} // namespace

void RegisterNavmeshNatives()
{
    MF_AddNatives(g_NavmeshNatives);
}

void RegisterNavmeshForwards()
{
    // forward goldnav_move_finished(entity, state);
    g_fwdMoveFinished = MF_RegisterForward("goldnav_move_finished", ET_IGNORE, FP_CELL, FP_CELL, FP_DONE);
}

void InvalidateNavmeshForwards()
{
    g_fwdMoveFinished = -1;
}

void ShutdownNavmeshNatives()
{
    g_fwdMoveFinished = -1;
    if (g_apiBound && g_api.SetMoveCallback)
    {
        // goldnav_mm переживает этот модуль — не оставляем ему указатель в выгруженный код.
        g_api.SetMoveCallback(nullptr);
    }
    g_apiBound = false;
}

} // namespace goldnav
