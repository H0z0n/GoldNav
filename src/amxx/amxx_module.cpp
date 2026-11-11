// Точка входа AMXX-модуля. Основная логика GoldNav живёт в goldnav_mm (Metamod
// plugin) — этот модуль лишь маршаллит AMX-параметры в вызовы его публичного C API
// (см. natives_navmesh.cpp) и пробрасывает событие "entity дошла/не дошла" в плагины.
#include "moduleconfig.h"
#include <amxxmodule.h>

#include "natives_navmesh.h"

void OnAmxxAttach()
{
    goldnav::RegisterNavmeshNatives();
}

void OnAmxxDetach()
{
    goldnav::ShutdownNavmeshNatives();
}

void OnPluginsLoaded()
{
    goldnav::RegisterNavmeshForwards();
}

void OnPluginsUnloading()
{
    goldnav::InvalidateNavmeshForwards();
}
