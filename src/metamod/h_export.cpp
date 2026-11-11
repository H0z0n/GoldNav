// Первая точка входа, которую движок/Metamod-R вызывает у плагина — раздаёт
// enginefuncs_t/globalvars_t. Структура и порядок операций сверены построчно с
// официальным примером плагина Metamod-R
// (third_party/metamod-r/metamod/extra/example/h_export.cpp).
#include <extdll.h>
#include <meta_api.h>

enginefuncs_t g_engfuncs;
globalvars_t* gpGlobals;

C_DLLEXPORT void WINAPI GiveFnptrsToDll(enginefuncs_t* pengfuncsFromEngine, globalvars_t* pGlobals)
{
    memcpy(&g_engfuncs, pengfuncsFromEngine, sizeof(enginefuncs_t));
    gpGlobals = pGlobals;
}
