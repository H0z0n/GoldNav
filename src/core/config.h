// cvar-backed настройки GoldNav. Регистрируются один раз в Meta_Attach, читаются
// каждый кадр из hooks.cpp/nav_generator.cpp — не кэшируем значения в статиках,
// т.к. админ может менять cvar-ы в рантайме через goldnav_status для диагностики.
#pragma once

namespace goldnav {

void RegisterConfigCvars();

// Размер клетки навигационной сетки в юнитах движка (goldnav_cell_size).
float ConfigCellSize();

// Бюджет генерации на один StartFrame-тик в миллисекундах (goldnav_gen_budget_ms).
float ConfigGenerationBudgetMs();

// С какой высоты entity ещё может спрыгнуть с уступа (goldnav_max_drop).
float ConfigMaxDrop();

// Показывать ли отладочную визуализацию (goldnav_debug).
bool ConfigDebugDrawEnabled();

// Движковые sv_gravity / sv_stepsize (с разумными значениями по умолчанию).
float ConfigGravity();
float ConfigStepSize();

} // namespace goldnav
