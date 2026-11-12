// NavMeshGenerator — строит NavGrid "заливкой" (flood fill) от посевных точек и
// поддерживает его в актуальном состоянии.
//
// От каждого узла пробуется шаг во все восемь соседних клеток той же логикой, которой
// потом ходит entity (см. nav_probe.h). Куда шаг удался — там появляется узел и ребро,
// новый узел встаёт в очередь на раскрытие. Что это даёт по сравнению со сканированием
// карты сверху вниз:
//   - не нужны границы мира (у world-энтити GoldSource их и нет: absmin/absmax = ±1);
//   - многоэтажность получается сама собой: узел — это (клетка, высота пола);
//   - в граф попадает только то, куда реально можно дойти.
//
// Работа размазывается по кадрам: Tick(budgetMs) раскрывает узлы, пока не выйдет
// бюджет. Новые посевы можно добавлять в любой момент.
//
// Динамика: когда на карте что-то изменилось (открылась дверь, поставили ящик),
// InvalidateRegion() ставит узлы вокруг этого места на повторное раскрытие — их рёбра
// пересчитываются по текущей геометрии, остальной граф не трогается.
#pragma once

#include <deque>
#include <vector>
#include "nav_grid.h"
#include "nav_probe.h"
#include "collision_world.h"

namespace goldnav {

struct NavGenConfig
{
    float cellSize = 16.0f;
    ProbeConfig probe;
    float maxJump = 0.0f;      // до какой высоты строить рёбра-прыжки (0 — не строить)
    size_t maxNodes = 400000;  // предохранитель по памяти/времени
    float maxCoord = 16384.0f; // дальше этого по любой оси не ходим
};

class NavMeshGenerator
{
public:
    // Сбрасывает grid и готовит генератор. world и grid должны пережить генератор.
    void Begin(NavGrid& grid, const ICollisionWorld& world, NavLayer layer, const NavGenConfig& config);

    bool IsStarted() const { return m_grid != nullptr; }

    // Посев: ищет под точкой пол, привязывает её к сетке и ставит новые узлы в очередь.
    // false — в точке нет пола/места для хулла (или рядом уже есть узел: тогда true,
    // но работы не добавляется).
    bool AddSeed(const Vec3& position);

    // Геометрия внутри коробки изменилась: узлы, которых это могло коснуться, будут
    // перепроверены и раскрыты заново. Возвращает число таких узлов.
    size_t InvalidateRegion(const Vec3& mins, const Vec3& maxs);

    // Раскрывает узлы из очереди, пока не исчерпан бюджет (мс реального времени).
    // true — очередь пуста, граф согласован (wallDist пересчитан).
    bool Tick(double budgetMs);

    // Раскрыть всё без ограничения по времени (для тестов и синхронных ребилдов).
    void RunToCompletion();

    bool IsIdle() const { return m_queue.empty() && !m_clearanceDirty; }
    size_t PendingNodes() const { return m_queue.size(); }
    size_t ExpandedNodes() const { return m_expandedCount; }
    bool HitNodeLimit() const { return m_hitNodeLimit; }

    const NavGenConfig& Config() const { return m_config; }

private:
    void ExpandNode(uint32_t nodeId);
    bool RevalidateNode(uint32_t nodeId);
    uint8_t FitMask(const Vec3& center) const;
    uint32_t FindOrCreateNode(int gx, int gy, const Vec3& feet, bool& outQueue);
    void Finish();

    NavGrid* m_grid = nullptr;
    const ICollisionWorld* m_world = nullptr;
    NavGenConfig m_config;

    std::deque<uint32_t> m_queue;
    size_t m_expandedCount = 0;
    bool m_clearanceDirty = false;
    bool m_hitNodeLimit = false;
};

} // namespace goldnav
