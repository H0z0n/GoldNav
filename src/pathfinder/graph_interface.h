// IGraph — шов между navmesh (представление сетки) и pathfinder (алгоритмы).
// astar.cpp/path_smoother.cpp знают только этот интерфейс и никогда не видят
// GoldSource-типы или конкретное представление NavGrid — это то, что позволяет
// pathfinder быть engine-agnostic и юнит-тестируемым без SDK.
#pragma once

#include <cstdint>
#include <cstddef>
#include "core/gn_math.h"

namespace goldnav {

constexpr uint32_t kInvalidNodeId = 0xFFFFFFFFu;
constexpr int kMaxNeighborsPerNode = 8;

struct Neighbor
{
    uint32_t nodeId = kInvalidNodeId;
    float cost = 0.0f;
};

// Небольшой стековый буфер соседей — избегаем аллокации на каждый GetNeighbors().
struct NeighborList
{
    Neighbor items[kMaxNeighborsPerNode];
    int count = 0;

    void Push(uint32_t nodeId, float cost)
    {
        if (count < kMaxNeighborsPerNode)
        {
            items[count].nodeId = nodeId;
            items[count].cost = cost;
            ++count;
        }
    }
};

class IGraph
{
public:
    virtual ~IGraph() = default;

    virtual size_t NodeCount() const = 0;
    virtual Vec3 NodePosition(uint32_t nodeId) const = 0;
    virtual void GetNeighbors(uint32_t nodeId, NeighborList& outNeighbors) const = 0;

    // Ближайший узел графа к произвольной точке пространства (или kInvalidNodeId,
    // если граф пуст/точка вне разумного радиуса поиска). Нужен пути от Vector до Vector.
    virtual uint32_t FindNearestNode(const Vec3& position) const = 0;
};

} // namespace goldnav
