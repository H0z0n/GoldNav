// GetEngineFunctions/_Post — GoldNav не хукает движковые функции напрямую (вся
// работа с движком идёт через g_engfuncs, заполненный в h_export.cpp), поэтому
// отдаёт полностью пустые таблицы. Присутствие этих функций в META_FUNCTIONS
// требуется Metamod-R (см. официальный пример плагина: meta_api.cpp всегда
// заполняет все 8 полей таблицы, включая GetEngineFunctions/_Post).
#include <extdll.h>
#include <meta_api.h>

C_DLLEXPORT int GetEngineFunctions(enginefuncs_t* pengfuncsFromEngine, int* interfaceVersion)
{
    if (!pengfuncsFromEngine)
    {
        return FALSE;
    }
    if (*interfaceVersion != ENGINE_INTERFACE_VERSION)
    {
        *interfaceVersion = ENGINE_INTERFACE_VERSION;
        return FALSE;
    }

    memset(pengfuncsFromEngine, 0, sizeof(enginefuncs_t));
    return TRUE;
}

C_DLLEXPORT int GetEngineFunctions_Post(enginefuncs_t* pengfuncsFromEngine, int* interfaceVersion)
{
    if (!pengfuncsFromEngine)
    {
        return FALSE;
    }
    if (*interfaceVersion != ENGINE_INTERFACE_VERSION)
    {
        *interfaceVersion = ENGINE_INTERFACE_VERSION;
        return FALSE;
    }

    memset(pengfuncsFromEngine, 0, sizeof(enginefuncs_t));
    return TRUE;
}
