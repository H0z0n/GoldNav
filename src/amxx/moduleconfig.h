// Локальная конфигурация AMXX-модуля — сверено построчно с реальным шаблоном
// third_party/amxmodx/public/sdk/moduleconfig.in.h.
#pragma once

#define MODULE_NAME     "GoldNav"
#define MODULE_VERSION  "1.0.0-beta"
#define MODULE_AUTHOR   "Hozon"
#define MODULE_URL      "https://t.me/AmxM0dX"
#define MODULE_LOGTAG   "GOLDNAV"
// Имя библиотеки — то, что плагин указывает в `#pragma reqlib goldnav` (см.
// pawn/include/goldnav.inc): по нему AMXX сам подгружает goldnav_amxx, даже если
// модуль не прописан в modules.ini.
#define MODULE_LIBRARY  "goldnav"
#define MODULE_LIBCLASS ""

// MODULE_RELOAD_ON_MAPCHANGE не определён: модулю нечего сбрасывать между картами, а
// лишняя выгрузка/загрузка DLL только создаёт окно, когда goldnav_mm держал бы
// указатель на колбэк в выгруженном коде.

// USE_METAMOD сознательно НЕ определён: goldnav_amxx не хукает Metamod/движок сам —
// он лишь вызывает публичный C API goldnav_mm (см. src/amxx/natives_navmesh.cpp).
// Определение USE_METAMOD потребовало бы <extdll.h>/<meta_api.h>/"osdep.h" (последнего
// нет в дереве amxmodx для standalone-модулей) — не нужно для тонкой natives-обёртки.

#define FN_AMXX_ATTACH OnAmxxAttach
#define FN_AMXX_DETACH OnAmxxDetach
#define FN_AMXX_PLUGINSLOADED OnPluginsLoaded
#define FN_AMXX_PLUGINSUNLOADING OnPluginsUnloading
