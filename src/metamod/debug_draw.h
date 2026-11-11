// Отладочная визуализация навигационной сетки через tempentity-лучи (TE_BEAMPOINTS),
// включается cvar-ом goldnav_debug (см. core/config.h). Троттлится, чтобы не
// заспамить сеть на больших сетках — рисует только рёбра рядом с игроком(ами).
#pragma once

namespace goldnav {

// Однократная подготовка (precache спрайта луча) — вызывается из
// Hook_ServerActivate_Post до первого возможного DebugDraw_Tick.
void DebugDraw_Precache();

// Вызывается из Hook_StartFrame_Post, когда goldnav_debug включён. Сам решает,
// нужно ли в этом кадре реально рисовать (троттлинг по частоте).
void DebugDraw_Tick();

} // namespace goldnav
