// Meta_Query/Meta_Attach/Meta_Detach + таблица META_FUNCTIONS. Структура сверена
// построчно с официальным примером плагина Metamod-R
// (third_party/metamod-r/metamod/extra/example/meta_api.cpp) — в отличие от него,
// здесь все 8 функций таблицы явно forward-declared перед использованием (пример
// полагается на порядок компиляции всех .cpp его собственного Makefile; мы этого
// не гарантируем и объявляем сигнатуры сами).
#include <extdll.h>
#include <meta_api.h>

#include "core/config.h"
#include "core/logger.h"
#include "metamod/admin_commands.h"

// dllapi.cpp
extern "C" int GetEntityAPI2(DLL_FUNCTIONS* pFunctionTable, int* interfaceVersion);
extern "C" int GetEntityAPI2_Post(DLL_FUNCTIONS* pFunctionTable, int* interfaceVersion);
extern "C" int GetNewDLLFunctions(NEW_DLL_FUNCTIONS* pNewFunctionTable, int* interfaceVersion);
extern "C" int GetNewDLLFunctions_Post(NEW_DLL_FUNCTIONS* pNewFunctionTable, int* interfaceVersion);

// engine_api.cpp
extern "C" int GetEngineFunctions(enginefuncs_t* pengfuncsFromEngine, int* interfaceVersion);
extern "C" int GetEngineFunctions_Post(enginefuncs_t* pengfuncsFromEngine, int* interfaceVersion);

meta_globals_t* gpMetaGlobals;
gamedll_funcs_t* gpGamedllFuncs;
mutil_funcs_t* gpMetaUtilFuncs;

plugin_info_t Plugin_info = {
    META_INTERFACE_VERSION,
    "GoldNav",
    "1.0.0-beta",
    __DATE__,
    "Hozon",
    "https://t.me/AmxM0dX",
    "GOLDNAV",
    PT_ANYTIME,
    PT_ANYTIME,
};

C_DLLEXPORT int Meta_Query(char* /*interfaceVersion*/, plugin_info_t** plinfo, mutil_funcs_t* pMetaUtilFuncs)
{
    *plinfo = &Plugin_info;
    gpMetaUtilFuncs = pMetaUtilFuncs;
    return TRUE;
}

META_FUNCTIONS gMetaFunctionTable = {
    nullptr,                      // pfnGetEntityAPI — не используется, GoldNav только POST
    nullptr,                      // pfnGetEntityAPI_Post — устаревший путь, не используется
    GetEntityAPI2,
    GetEntityAPI2_Post,
    GetNewDLLFunctions,
    GetNewDLLFunctions_Post,
    GetEngineFunctions,
    GetEngineFunctions_Post,
};

C_DLLEXPORT int Meta_Attach(PLUG_LOADTIME /*now*/, META_FUNCTIONS* pFunctionTable,
                             meta_globals_t* pMGlobals, gamedll_funcs_t* pGamedllFuncs)
{
    if (!pMGlobals || !pFunctionTable)
    {
        return FALSE;
    }

    gpMetaGlobals = pMGlobals;
    gpGamedllFuncs = pGamedllFuncs;

    memcpy(pFunctionTable, &gMetaFunctionTable, sizeof(META_FUNCTIONS));

    goldnav::RegisterConfigCvars();
    goldnav::RegisterAdminCommands();

    GOLDNAV_LOG_INFO("attached");
    return TRUE;
}

C_DLLEXPORT int Meta_Detach(PLUG_LOADTIME /*now*/, PL_UNLOAD_REASON /*reason*/)
{
    GOLDNAV_LOG_INFO("detached");
    return TRUE;
}
