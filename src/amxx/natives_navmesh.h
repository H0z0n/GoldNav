// AMXX-натусы GoldNav — тонкая обёртка над публичным C API goldnav_mm (см.
// core/goldnav_api.h). Маршаллинг AMX cell-массивов <-> Vector делает
// natives_navmesh.cpp; сама навигационная логика здесь не реализуется — только
// перекладывание данных через границу Pawn VM <-> C API.
#pragma once

namespace goldnav {

// Регистрирует AMX_NATIVE_INFO-таблицу через MF_AddNatives (см. amxx_module.cpp).
void RegisterNavmeshNatives();

// Регистрирует forward goldnav_move_finished — когда плагины уже загружены.
void RegisterNavmeshForwards();

// Плагины выгружаются (смена карты) — хэндл forward-а больше не валиден.
void InvalidateNavmeshForwards();

// Модуль выгружается: снять колбэк, зарегистрированный в goldnav_mm.
void ShutdownNavmeshNatives();

} // namespace goldnav
