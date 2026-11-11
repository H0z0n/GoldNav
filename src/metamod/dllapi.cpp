// GetEntityAPI2/_Post и GetNewDLLFunctions/_Post — таблицы dllapi-хуков, которые
// Metamod-R запрашивает у плагина через META_FUNCTIONS (см. meta_api.cpp). GoldNav
// использует только POST-хуки (см. hooks.cpp) — GetEntityAPI2 (pre) и
// GetNewDLLFunctions/_Post отдают полностью пустые таблицы.
#include <extdll.h>
#include <meta_api.h>

#include "metamod/hooks.h"

C_DLLEXPORT int GetEntityAPI2(DLL_FUNCTIONS* pFunctionTable, int* interfaceVersion)
{
    if (!pFunctionTable)
    {
        return FALSE;
    }
    if (*interfaceVersion != INTERFACE_VERSION)
    {
        *interfaceVersion = INTERFACE_VERSION;
        return FALSE;
    }

    memset(pFunctionTable, 0, sizeof(DLL_FUNCTIONS));
    return TRUE;
}

C_DLLEXPORT int GetEntityAPI2_Post(DLL_FUNCTIONS* pFunctionTable, int* interfaceVersion)
{
    if (!pFunctionTable)
    {
        return FALSE;
    }
    if (*interfaceVersion != INTERFACE_VERSION)
    {
        *interfaceVersion = INTERFACE_VERSION;
        return FALSE;
    }

    memset(pFunctionTable, 0, sizeof(DLL_FUNCTIONS));
    // Только POST: ServerActivate — после того как мир/энтити заспавнены движком;
    // StartFrame — после стандартной обработки кадра.
    pFunctionTable->pfnServerActivate = goldnav::Hook_ServerActivate_Post;
    pFunctionTable->pfnServerDeactivate = goldnav::Hook_ServerDeactivate_Post;
    pFunctionTable->pfnStartFrame = goldnav::Hook_StartFrame_Post;
    return TRUE;
}

C_DLLEXPORT int GetNewDLLFunctions(NEW_DLL_FUNCTIONS* pNewFunctionTable, int* interfaceVersion)
{
    if (!pNewFunctionTable)
    {
        return FALSE;
    }
    if (*interfaceVersion != NEW_DLL_FUNCTIONS_VERSION)
    {
        *interfaceVersion = NEW_DLL_FUNCTIONS_VERSION;
        return FALSE;
    }

    memset(pNewFunctionTable, 0, sizeof(NEW_DLL_FUNCTIONS));
    return TRUE;
}

C_DLLEXPORT int GetNewDLLFunctions_Post(NEW_DLL_FUNCTIONS* pNewFunctionTable, int* interfaceVersion)
{
    if (!pNewFunctionTable)
    {
        return FALSE;
    }
    if (*interfaceVersion != NEW_DLL_FUNCTIONS_VERSION)
    {
        *interfaceVersion = NEW_DLL_FUNCTIONS_VERSION;
        return FALSE;
    }

    memset(pNewFunctionTable, 0, sizeof(NEW_DLL_FUNCTIONS));
    return TRUE;
}
