// Глобальные указатели на движковые интерфейсы, полученные в GiveFnptrsToDll
// (см. meta_api.cpp). Единая точка объявления, чтобы logger.cpp, nav_trace_sampler.cpp
// и hooks.cpp не дублировали extern-декларации.
#pragma once

#include <extdll.h>
#include <meta_api.h>

namespace goldnav {

// g_engfuncs/gpGlobals уже объявлены как extern в extdll.h/eiface.h (стандартный
// HLSDK-паттерн: enginefuncs_t g_engfuncs; globalvars_t* gpGlobals;), здесь ничего
// переопределять не нужно — файл существует как явная точка документирования того,
// какие движковые глобалы GoldNav использует.

} // namespace goldnav
