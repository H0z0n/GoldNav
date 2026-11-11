// Сглаживание пути: жадное "натягивание нити" (string pulling) по сырому пути A*.
// Из пути выбрасываются промежуточные точки, если между оставшимися соседями можно
// пройти по прямой. Сама проверка "можно ли по прямой" передаётся снаружи —
// в боевом коде это NavGrid::LineOfSight (обход клеток сетки по Walk-рёбрам, без
// обращения к движку), в тестах — мок. Работает с индексами точек, а не с
// координатами, чтобы предикат мог пользоваться id узлов.
#pragma once

#include <vector>
#include <functional>
#include <cstddef>

namespace goldnav {

// true, если от точки пути с индексом from можно идти по прямой к точке to (from < to).
using DirectPathPredicate = std::function<bool(size_t from, size_t to)>;

// Возвращает индексы точек сырого пути, которые остаются в сглаженном (всегда
// включает первую и последнюю). maxLookahead ограничивает, как далеко вперёд по
// пути пробуем срезать от каждой опорной точки (0 — без ограничения).
std::vector<size_t> SmoothPathIndices(size_t pointCount, const DirectPathPredicate& canGoDirect,
                                      size_t maxLookahead = 0);

} // namespace goldnav
