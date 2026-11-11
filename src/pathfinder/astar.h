// A* по абстрактному IGraph. Engine-agnostic, без единой зависимости от GoldSource —
// юнит-тестируется на синтетических графах без SDK.
#pragma once

#include <vector>
#include <cstdint>
#include "graph_interface.h"
#include "binary_heap.h"

namespace goldnav {

enum class AStarResult
{
    Found,    // путь до цели
    Partial,  // цель недостижима; путь до достижимого узла, ближайшего к цели
    NotFound, // цель недостижима (или невалидные узлы)
};

struct AStarOptions
{
    // Если цель недостижима — вернуть путь до ближайшего к ней достижимого узла
    // ("подойти как можно ближе"), а не отказ.
    bool allowPartial = false;
    // Предохранитель: сколько узлов можно раскрыть за один поиск (0 — без ограничения).
    size_t maxExpansions = 0;
};

class AStarPathfinder
{
public:
    // Scratch-буферы переиспользуются между вызовами: вместо очистки gScore/cameFrom
    // на каждый запрос используется generation-counter — узел считается "непосещённым
    // в этом поиске", если его visitedGen != m_currentGen.
    AStarResult Search(const IGraph& graph, uint32_t startNode, uint32_t goalNode,
                       std::vector<uint32_t>& outNodePath, const AStarOptions& options = AStarOptions());

    // Строгий вариант: true только если найден путь до самой цели.
    bool FindPath(const IGraph& graph, uint32_t startNode, uint32_t goalNode,
                  std::vector<uint32_t>& outNodePath)
    {
        return Search(graph, startNode, goalNode, outNodePath) == AStarResult::Found;
    }

    // Сколько узлов раскрыл последний поиск (диагностика).
    size_t LastExpansions() const { return m_lastExpansions; }

private:
    struct NodeState
    {
        float gScore = 0.0f;
        uint32_t cameFrom = kInvalidNodeId;
        uint32_t visitedGen = 0;
        bool closed = false;
    };

    std::vector<NodeState> m_state;
    uint32_t m_currentGen = 0;
    IndexedBinaryHeap<float> m_openSet;
    size_t m_lastExpansions = 0;

    NodeState& StateOf(uint32_t nodeId);
    bool IsVisitedThisSearch(uint32_t nodeId) const;
    void BuildPath(uint32_t endNode, std::vector<uint32_t>& outNodePath) const;
};

} // namespace goldnav
