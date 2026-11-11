// Синтетический graph-фикстура для тестов pathfinder — ручная 2D-сетка с
// заблокированными клетками, без единой зависимости от NavGrid/SDK.
#pragma once

#include <vector>
#include <string>
#include <utility>
#include "pathfinder/graph_interface.h"

namespace goldnav_test {

// Строит граф из ASCII-карты: '.' = проходимо, '#' = стена. 4-connected.
// Каждая строка map[] должна иметь одинаковую длину.
class MockGridGraph : public goldnav::IGraph
{
public:
    explicit MockGridGraph(const std::vector<std::string>& asciiMap)
    {
        m_height = static_cast<int>(asciiMap.size());
        m_width = m_height > 0 ? static_cast<int>(asciiMap[0].size()) : 0;
        m_walkable.assign(static_cast<size_t>(m_width) * m_height, false);
        m_nodeIdOf.assign(static_cast<size_t>(m_width) * m_height, goldnav::kInvalidNodeId);

        for (int y = 0; y < m_height; ++y)
        {
            for (int x = 0; x < m_width; ++x)
            {
                const bool walkable = asciiMap[y][x] == '.';
                m_walkable[Index(x, y)] = walkable;
                if (walkable)
                {
                    m_nodeIdOf[Index(x, y)] = static_cast<uint32_t>(m_positions.size());
                    m_positions.push_back(goldnav::Vec3(static_cast<float>(x), static_cast<float>(y), 0.0f));
                    m_gridOfNode.push_back({x, y});
                }
            }
        }
    }

    uint32_t NodeIdAt(int x, int y) const
    {
        if (x < 0 || y < 0 || x >= m_width || y >= m_height) return goldnav::kInvalidNodeId;
        return m_nodeIdOf[Index(x, y)];
    }

    size_t NodeCount() const override { return m_positions.size(); }

    goldnav::Vec3 NodePosition(uint32_t nodeId) const override
    {
        return nodeId < m_positions.size() ? m_positions[nodeId] : goldnav::Vec3();
    }

    void GetNeighbors(uint32_t nodeId, goldnav::NeighborList& outNeighbors) const override
    {
        outNeighbors.count = 0;
        if (nodeId >= m_gridOfNode.size()) return;

        const auto [x, y] = m_gridOfNode[nodeId];
        static const int dx[4] = {1, -1, 0, 0};
        static const int dy[4] = {0, 0, 1, -1};

        for (int i = 0; i < 4; ++i)
        {
            const uint32_t neighborId = NodeIdAt(x + dx[i], y + dy[i]);
            if (neighborId != goldnav::kInvalidNodeId)
            {
                outNeighbors.Push(neighborId, 1.0f);
            }
        }
    }

    uint32_t FindNearestNode(const goldnav::Vec3& position) const override
    {
        const int x = static_cast<int>(position.x + 0.5f);
        const int y = static_cast<int>(position.y + 0.5f);
        return NodeIdAt(x, y);
    }

private:
    int Index(int x, int y) const { return y * m_width + x; }

    int m_width = 0;
    int m_height = 0;
    std::vector<bool> m_walkable;
    std::vector<uint32_t> m_nodeIdOf;
    std::vector<goldnav::Vec3> m_positions;
    std::vector<std::pair<int, int>> m_gridOfNode;
};

} // namespace goldnav_test
