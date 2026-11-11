#include "astar.h"

#include <algorithm>
#include <limits>

namespace goldnav {

AStarPathfinder::NodeState& AStarPathfinder::StateOf(uint32_t nodeId)
{
    if (nodeId >= m_state.size())
    {
        m_state.resize(static_cast<size_t>(nodeId) + 1);
    }

    NodeState& state = m_state[nodeId];
    if (state.visitedGen != m_currentGen)
    {
        state.gScore = std::numeric_limits<float>::infinity();
        state.cameFrom = kInvalidNodeId;
        state.visitedGen = m_currentGen;
        state.closed = false;
    }
    return state;
}

bool AStarPathfinder::IsVisitedThisSearch(uint32_t nodeId) const
{
    return nodeId < m_state.size() && m_state[nodeId].visitedGen == m_currentGen;
}

void AStarPathfinder::BuildPath(uint32_t endNode, std::vector<uint32_t>& outNodePath) const
{
    outNodePath.clear();
    uint32_t node = endNode;
    while (node != kInvalidNodeId)
    {
        outNodePath.push_back(node);
        node = m_state[node].cameFrom;
    }
    std::reverse(outNodePath.begin(), outNodePath.end());
}

AStarResult AStarPathfinder::Search(const IGraph& graph, uint32_t startNode, uint32_t goalNode,
                                    std::vector<uint32_t>& outNodePath, const AStarOptions& options)
{
    outNodePath.clear();
    m_lastExpansions = 0;

    const size_t nodeCount = graph.NodeCount();
    if (startNode == kInvalidNodeId || goalNode == kInvalidNodeId ||
        startNode >= nodeCount || goalNode >= nodeCount)
    {
        return AStarResult::NotFound;
    }

    if (startNode == goalNode)
    {
        outNodePath.push_back(startNode);
        return AStarResult::Found;
    }

    // Новая генерация вместо очистки m_state. При переполнении счётчика (2^32 поисков)
    // сбрасываем всё честно, иначе старые записи "ожили" бы как посещённые.
    ++m_currentGen;
    if (m_currentGen == 0)
    {
        std::fill(m_state.begin(), m_state.end(), NodeState());
        m_currentGen = 1;
    }
    m_openSet.Clear();
    m_openSet.Reserve(nodeCount);

    const Vec3 goalPos = graph.NodePosition(goalNode);

    NodeState& startState = StateOf(startNode);
    startState.gScore = 0.0f;
    const float startH = Distance(graph.NodePosition(startNode), goalPos);
    m_openSet.Push(startNode, startH);

    // Лучший кандидат для частичного пути — раскрытый узел с минимальной эвристикой.
    uint32_t bestNode = startNode;
    float bestH = startH;

    NeighborList neighbors;

    while (!m_openSet.Empty())
    {
        const uint32_t current = m_openSet.PopMin();

        if (current == goalNode)
        {
            BuildPath(current, outNodePath);
            return AStarResult::Found;
        }

        StateOf(current).closed = true;
        ++m_lastExpansions;

        const float currentH = Distance(graph.NodePosition(current), goalPos);
        if (currentH < bestH)
        {
            bestH = currentH;
            bestNode = current;
        }

        if (options.maxExpansions != 0 && m_lastExpansions >= options.maxExpansions)
        {
            break;
        }

        graph.GetNeighbors(current, neighbors);
        const float currentG = StateOf(current).gScore;

        for (int i = 0; i < neighbors.count; ++i)
        {
            const uint32_t neighborId = neighbors.items[i].nodeId;
            if (neighborId == kInvalidNodeId || neighborId >= nodeCount)
            {
                continue;
            }
            if (IsVisitedThisSearch(neighborId) && m_state[neighborId].closed)
            {
                continue;
            }

            const float tentativeG = currentG + neighbors.items[i].cost;
            NodeState& neighborState = StateOf(neighborId);

            if (tentativeG < neighborState.gScore)
            {
                neighborState.gScore = tentativeG;
                neighborState.cameFrom = current;

                const float h = Distance(graph.NodePosition(neighborId), goalPos);
                m_openSet.Push(neighborId, tentativeG + h);
            }
        }
    }

    if (options.allowPartial)
    {
        BuildPath(bestNode, outNodePath);
        return AStarResult::Partial;
    }

    return AStarResult::NotFound;
}

} // namespace goldnav
