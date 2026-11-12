#include "config.h"

#include <extdll.h>
#include <meta_api.h>
#include <util.h> // CVAR_REGISTER/CVAR_GET_FLOAT (dlls/enginecallback.h, included transitively)

namespace goldnav {

namespace {
// cvar_t::string — это `char*` (не `const char*`) в HLSDK, поэтому литералы
// приводим явно — стандартная для GoldSource-кодовой базы идиома.
cvar_t gCvarCellSize        = { "goldnav_cell_size", (char*)"16", FCVAR_SERVER, 0.0f, nullptr };
cvar_t gCvarGenBudgetMs     = { "goldnav_gen_budget_ms", (char*)"8", FCVAR_SERVER, 0.0f, nullptr };
cvar_t gCvarMaxDrop         = { "goldnav_max_drop", (char*)"150", FCVAR_SERVER, 0.0f, nullptr };
cvar_t gCvarMaxJump         = { "goldnav_max_jump", (char*)"64", FCVAR_SERVER, 0.0f, nullptr };
cvar_t gCvarDynamic         = { "goldnav_dynamic", (char*)"1", FCVAR_SERVER, 0.0f, nullptr };
cvar_t gCvarDebug           = { "goldnav_debug", (char*)"0", FCVAR_SERVER, 0.0f, nullptr };
} // namespace

void RegisterConfigCvars()
{
    CVAR_REGISTER(&gCvarCellSize);
    CVAR_REGISTER(&gCvarGenBudgetMs);
    CVAR_REGISTER(&gCvarMaxDrop);
    CVAR_REGISTER(&gCvarMaxJump);
    CVAR_REGISTER(&gCvarDynamic);
    CVAR_REGISTER(&gCvarDebug);
}

float ConfigCellSize()
{
    // Меньше 8 — сетка взрывается по числу узлов, больше 32 — теряются дверные проёмы.
    const float value = CVAR_GET_FLOAT("goldnav_cell_size");
    if (value < 8.0f) return value > 0.0f ? 8.0f : 16.0f;
    return value > 32.0f ? 32.0f : value;
}

float ConfigGenerationBudgetMs()
{
    const float value = CVAR_GET_FLOAT("goldnav_gen_budget_ms");
    return value > 0.0f ? value : 8.0f;
}

float ConfigMaxDrop()
{
    const float value = CVAR_GET_FLOAT("goldnav_max_drop");
    return value >= 0.0f ? value : 150.0f;
}

float ConfigMaxJump()
{
    const float value = CVAR_GET_FLOAT("goldnav_max_jump");
    if (value <= 0.0f) return 0.0f;
    return value > 200.0f ? 200.0f : value;
}

bool ConfigDynamicEnabled()
{
    return CVAR_GET_FLOAT("goldnav_dynamic") != 0.0f;
}

float ConfigGravity()
{
    const float value = CVAR_GET_FLOAT("sv_gravity");
    return value > 0.0f ? value : 800.0f;
}

float ConfigStepSize()
{
    const float value = CVAR_GET_FLOAT("sv_stepsize");
    return value > 0.0f ? value : 18.0f;
}

bool ConfigDebugDrawEnabled()
{
    return CVAR_GET_FLOAT("goldnav_debug") != 0.0f;
}

} // namespace goldnav
