// NavGraphView — IGraph-обёртка над NavGrid для конкретного варианта хулла. Отдаёт
// A* только те рёбра, по которым этот вариант проходит, и назначает им стоимость:
// расстояние плюс "человеческие" надбавки — держаться подальше от стен и краёв
// уступов, не лезть в воду без нужды, не прыгать с высоты, если есть обход.
// Engine-agnostic (часть goldnav_core).
#pragma once

#include "nav_grid.h"
#include "pathfinder/graph_interface.h"

namespace goldnav {

struct NavCostConfig
{
    // Надбавка к длине шага в долях, по wallDist целевого узла (0 — вплотную к стене).
    float wallPenalty[kMaxWallDist + 1] = { 1.0f, 0.3f, 0.1f, 0.0f };
    float waterPenalty = 1.5f;
    float deepWaterPenalty = 6.0f;
    // Спрыгивание: фиксированная цена + цена за каждый юнит высоты.
    float dropBaseCost = 48.0f;
    float dropCostPerUnit = 1.0f;
    bool allowDrops = true;
    float maxDrop = 100000.0f; // выше этого конкретная entity не спрыгивает
    // Прыжок вверх: по умолчанию entity не прыгает (maxJump = 0).
    float maxJump = 0.0f;
    float jumpBaseCost = 64.0f;
    float jumpCostPerUnit = 1.5f;
};

class NavGraphView : public IGraph
{
public:
    NavGraphView(const NavGrid& grid, int variant, const NavCostConfig& cost = NavCostConfig())
        : m_grid(grid), m_variant(variant), m_bit(VariantBit(variant)), m_cost(cost) {}

    size_t NodeCount() const override { return m_grid.NodeCount(); }

    Vec3 NodePosition(uint32_t nodeId) const override
    {
        return nodeId < m_grid.NodeCount() ? m_grid.Node(nodeId).pos : Vec3();
    }

    void GetNeighbors(uint32_t nodeId, NeighborList& outNeighbors) const override
    {
        outNeighbors.count = 0;
        if (nodeId >= m_grid.NodeCount())
        {
            return;
        }

        const NavNode& node = m_grid.Node(nodeId);
        if ((node.variantMask & m_bit) == 0)
        {
            return;
        }

        for (int dir = 0; dir < kNavDirCount; ++dir)
        {
            const uint8_t type = node.edgeType[dir];
            if (type == kEdgeNone || (node.edgeMask[dir] & m_bit) == 0)
            {
                continue;
            }
            const float edgeHeight = NavDecodeHeight(node.edgeHeight[dir]);
            if (type == kEdgeDrop && (!m_cost.allowDrops || edgeHeight > m_cost.maxDrop + 2.0f))
            {
                continue;
            }
            if (type == kEdgeJump && edgeHeight > m_cost.maxJump + 0.5f)
            {
                continue;
            }

            const NavNode& target = m_grid.Node(node.edgeTarget[dir]);

            // Стоимость никогда не меньше 3D-расстояния — евклидова эвристика A*
            // остаётся допустимой, путь оптимален относительно этих весов.
            const float dist = Distance(node.pos, target.pos);
            float factor = 1.0f + m_cost.wallPenalty[target.wallDist[m_variant] <= kMaxWallDist
                                                          ? target.wallDist[m_variant] : kMaxWallDist];
            if (target.flags & kNodeDeepWater)
            {
                factor += m_cost.deepWaterPenalty;
            }
            else if (target.flags & kNodeWater)
            {
                factor += m_cost.waterPenalty;
            }

            float cost = dist * factor;
            if (type == kEdgeDrop)
            {
                cost += m_cost.dropBaseCost + m_cost.dropCostPerUnit * (node.pos.z - target.pos.z);
            }
            else if (type == kEdgeJump)
            {
                cost += m_cost.jumpBaseCost + m_cost.jumpCostPerUnit * edgeHeight;
            }

            outNeighbors.Push(node.edgeTarget[dir], cost);
        }
    }

    uint32_t FindNearestNode(const Vec3& position) const override
    {
        return m_grid.FindNearestNode(position, m_variant, 1024.0f);
    }

private:
    const NavGrid& m_grid;
    int m_variant;
    uint8_t m_bit;
    NavCostConfig m_cost;
};

} // namespace goldnav
