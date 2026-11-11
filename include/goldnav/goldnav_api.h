// Публичный C API GoldNav — каноническая версия. То, что подключают ДРУГИЕ
// Metamod-плагины/DLL (либо копируют этот файл, либо ссылаются на него из дерева
// репозитория — src/core/goldnav_api.h делает именно это, чтобы не дублировать
// содержимое; при релизе синхронизировать с docs/public_api.md).
//
// ABI-контракт: только extern "C" функции с сырыми указателями/массивами и явной
// длиной. Это НАМЕРЕННО не std::vector<Vector>& — STL-типы не переносимы через
// границу DLL между разными компиляторами/версиями STL (MSVC/MinGW/разные gcc).
//
// Самодостаточен относительно ДЕРЕВА GOLDNAV: не подключает внутренние заголовки
// GoldNav, чтобы его можно было скопировать в другой проект без остального src/.
// Требует HLSDK на include-path вызывающего (Vector/edict_t).
#pragma once

#include <extdll.h>

// GOLDNAV_BUILDING_SHARED определяется только при сборке самого goldnav_mm
// (см. CMakeLists.txt) — тогда это export, иначе (у потребителей API) import.
#if defined(_WIN32)
    #if defined(GOLDNAV_BUILDING_SHARED)
        #define GOLDNAV_API extern "C" __declspec(dllexport)
    #else
        #define GOLDNAV_API extern "C" __declspec(dllimport)
    #endif
#else
    #if defined(GOLDNAV_BUILDING_SHARED)
        #define GOLDNAV_API extern "C" __attribute__((visibility("default")))
    #else
        #define GOLDNAV_API extern "C"
    #endif
#endif

// --- Хуллы ---
// Движок GoldSource сталкивает entity с картой только четырьмя размерами; навмеш
// строится под них. AUTO — взять тот, которым движок реально сталкивает эту entity.
#define GOLDNAV_HULL_AUTO     (-1)
#define GOLDNAV_HULL_POINT    0   // entity шириной до 8 юнитов (точка в origin)
#define GOLDNAV_HULL_DUCKING  1   // 32x32x36
#define GOLDNAV_HULL_STANDING 2   // 32x32x72
#define GOLDNAV_HULL_LARGE    3   // 64x64x64

// --- Состояние entity, которую ведёт GoldNav (GoldNav_GetEntityState) ---
#define GOLDNAV_STATE_IDLE    0   // GoldNav эту entity не ведёт
#define GOLDNAV_STATE_PENDING 1   // ждёт навмеш/путь (генерация ещё идёт)
#define GOLDNAV_STATE_MOVING  2   // идёт
#define GOLDNAV_STATE_ARRIVED 3   // дошла (или подошла так близко, как возможно)
#define GOLDNAV_STATE_FAILED  4   // пути нет / застряла окончательно

// --- Флаги GoldNav_MoveEntityTo ---
#define GOLDNAV_MOVE_NO_TURN  (1 << 0) // не поворачивать entity (angles.y) по ходу движения
#define GOLDNAV_MOVE_NO_DROP  (1 << 1) // не спрыгивать с уступов выше ступеньки
#define GOLDNAV_MOVE_NO_TOUCH (1 << 2) // не вызывать Touch у entity, в которые упёрлись
#define GOLDNAV_MOVE_EXACT    (1 << 3) // цель недостижима -> FAILED, а не "подойти поближе"

extern "C" {

// Ищет путь между двумя точками. outPath — буфер вызывающего на outPathMaxLen точек;
// в него пишутся точки, по которым идти (положение "ног": XY — центр хулла, Z — пол),
// без стартовой. from/to — произвольные точки: старт привязывается к полу под from,
// цель — к ближайшему проходимому месту.
// false — пути нет, либо навмеш этого хулла ещё не сгенерирован.
GOLDNAV_API bool GoldNav_FindPath(const Vector* from, const Vector* to,
                                   Vector* outPath, int outPathMaxLen, int* outPathLen,
                                   int hullType);

// Путь до текущей позиции произвольного entity.
GOLDNAV_API bool GoldNav_FindPathToEntity(edict_t* entity, const Vector* from,
                                           Vector* outPath, int outPathMaxLen, int* outPathLen,
                                           int hullType);

// Главная функция: отправить entity в точку. GoldNav сам находит путь, ведёт entity
// по нему каждый кадр (ступеньки, обход стен и других entity), перестраивает путь,
// если entity столкнули или она застряла, и останавливает её в цели.
// Если навмеш ещё генерируется, запрос ждёт (состояние PENDING) и стартует сам.
// hullType — GOLDNAV_HULL_*; flags — GOLDNAV_MOVE_*.
// false — entity невалидна или это игрок (игроков GoldNav не двигает).
GOLDNAV_API bool GoldNav_MoveEntityTo(edict_t* entity, const Vector* goal, float speed,
                                       int hullType, int flags);

// Вести entity по готовому списку точек (обычно — из GoldNav_FindPath). Последняя
// точка считается целью: если entity собьётся с пути, GoldNav перестроит его к ней.
GOLDNAV_API void GoldNav_MoveEntityAlongPath(edict_t* entity, const Vector* path, int pathLen, float speed);

// Останавливает ведение entity (без побочных эффектов, если её никто не ведёт).
GOLDNAV_API void GoldNav_StopFollowing(edict_t* entity);

// GOLDNAV_STATE_* для entity.
GOLDNAV_API int GoldNav_GetEntityState(edict_t* entity);

// Готов ли навмеш основного слоя (хуллы 32x32: ducking/standing).
GOLDNAV_API bool GoldNav_IsNavMeshReady(void);

// Готов ли навмеш под конкретный хулл. Слои point и large строятся по первому
// требованию — этот вызов заодно запускает их генерацию.
GOLDNAV_API bool GoldNav_IsHullReady(int hullType);

GOLDNAV_API int GoldNav_GetVersion(void); // major*10000 + minor*100 + patch

// Вызывается, когда ведение entity закончилось: state — GOLDNAV_STATE_ARRIVED или
// GOLDNAV_STATE_FAILED. Один обработчик на весь модуль; nullptr — снять.
typedef void (*GoldNavMoveCallback)(int entityIndex, int state);
GOLDNAV_API void GoldNav_SetMoveCallback(GoldNavMoveCallback callback);

// Конвертирует индекс entity (как его видит Pawn/AMXX-скрипт — ENTINDEX) в edict_t*.
// nullptr для невалидного индекса/свободного слота.
GOLDNAV_API edict_t* GoldNav_EdictByIndex(int index);

} // extern "C"

#if defined(__cplusplus)

#include <vector>

// Инлайн-обёртки с std::vector. БЕЗОПАСНЫ ТОЛЬКО при статической линковке в том же
// тулчейне (см. предупреждение выше про STL через границу DLL).
namespace NavMesh {

constexpr int kMaxPathLength = 256;

inline bool FindPath(const Vector& from, const Vector& to, std::vector<Vector>& outPath, int hullType = GOLDNAV_HULL_STANDING)
{
    outPath.resize(kMaxPathLength);
    int actualLen = 0;
    const bool found = GoldNav_FindPath(&from, &to, outPath.data(), kMaxPathLength, &actualLen, hullType);
    outPath.resize(found ? actualLen : 0);
    return found;
}

inline bool FindPathToEntity(edict_t* entity, const Vector& from, std::vector<Vector>& outPath, int hullType = GOLDNAV_HULL_STANDING)
{
    outPath.resize(kMaxPathLength);
    int actualLen = 0;
    const bool found = GoldNav_FindPathToEntity(entity, &from, outPath.data(), kMaxPathLength, &actualLen, hullType);
    outPath.resize(found ? actualLen : 0);
    return found;
}

inline bool MoveEntityTo(edict_t* entity, const Vector& goal, float speed, int hullType = GOLDNAV_HULL_AUTO, int flags = 0)
{
    return GoldNav_MoveEntityTo(entity, &goal, speed, hullType, flags);
}

inline void MoveEntityAlongPath(edict_t* entity, const std::vector<Vector>& path, float speed)
{
    GoldNav_MoveEntityAlongPath(entity, path.data(), static_cast<int>(path.size()), speed);
}

} // namespace NavMesh

#endif // __cplusplus
