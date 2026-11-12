#include "nav_grid.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace goldnav {

//                                   +x  -x  +y  -y  ++  +-  -+  --
const int kNavDirDx[kNavDirCount] = { 1, -1,  0,  0,  1,  1, -1, -1 };
const int kNavDirDy[kNavDirCount] = { 0,  0,  1, -1,  1, -1,  1, -1 };

int NavDirFromDelta(int dx, int dy)
{
    for (int i = 0; i < kNavDirCount; ++i)
    {
        if (kNavDirDx[i] == dx && kNavDirDy[i] == dy)
        {
            return i;
        }
    }
    return -1;
}

int NavOppositeDir(int dir)
{
    static const int kOpposite[kNavDirCount] = { 1, 0, 3, 2, 7, 6, 5, 4 };
    return kOpposite[dir];
}

void NavGrid::Initialize(NavLayer layer, float cellSize)
{
    Clear();
    m_layer = layer;
    m_cellSize = cellSize >= 4.0f ? cellSize : 16.0f;
}

void NavGrid::Clear()
{
    m_nodes.clear();
    m_columnHead.clear();
    ++m_revision;
}

void NavGrid::WorldToCell(const Vec3& pos, int& outGx, int& outGy) const
{
    outGx = static_cast<int>(std::floor(pos.x / m_cellSize));
    outGy = static_cast<int>(std::floor(pos.y / m_cellSize));
}

uint32_t NavGrid::ColumnHead(int gx, int gy) const
{
    const auto it = m_columnHead.find(CellKey(gx, gy));
    return it != m_columnHead.end() ? it->second : kInvalidNodeId;
}

uint32_t NavGrid::FindNodeInColumn(int gx, int gy, float z, float tolerance) const
{
    uint32_t best = kInvalidNodeId;
    float bestDiff = tolerance;

    for (uint32_t id = ColumnHead(gx, gy); id != kInvalidNodeId; id = m_nodes[id].nextInColumn)
    {
        const float diff = std::fabs(m_nodes[id].pos.z - z);
        if (diff <= bestDiff)
        {
            bestDiff = diff;
            best = id;
        }
    }
    return best;
}

uint32_t NavGrid::AddNode(int gx, int gy, float z, uint8_t variantMask, uint8_t flags)
{
    NavNode node;
    node.gx = gx;
    node.gy = gy;
    node.pos = Vec3(CellCenter(gx), CellCenter(gy), z);
    node.variantMask = variantMask;
    node.flags = flags;

    const uint32_t id = static_cast<uint32_t>(m_nodes.size());
    const int64_t key = CellKey(gx, gy);
    const auto it = m_columnHead.find(key);
    if (it != m_columnHead.end())
    {
        node.nextInColumn = it->second;
        it->second = id;
    }
    else
    {
        m_columnHead.emplace(key, id);
    }

    m_nodes.push_back(node);
    ++m_revision;
    return id;
}

void NavGrid::SetEdge(uint32_t from, int dir, uint32_t to, uint8_t variantMask, uint8_t type, float height)
{
    if (from >= m_nodes.size() || dir < 0 || dir >= kNavDirCount)
    {
        return;
    }

    NavNode& node = m_nodes[from];
    node.edgeTarget[dir] = to;
    node.edgeMask[dir] = variantMask;
    node.edgeType[dir] = (to == kInvalidNodeId || variantMask == 0) ? static_cast<uint8_t>(kEdgeNone) : type;
    node.edgeHeight[dir] = NavEncodeHeight(height);
    ++m_revision;
}

void NavGrid::ClearEdges(uint32_t nodeId)
{
    if (nodeId >= m_nodes.size())
    {
        return;
    }
    NavNode& node = m_nodes[nodeId];
    for (int dir = 0; dir < kNavDirCount; ++dir)
    {
        node.edgeTarget[dir] = kInvalidNodeId;
        node.edgeMask[dir] = 0;
        node.edgeType[dir] = kEdgeNone;
        node.edgeHeight[dir] = 0;
    }
    ++m_revision;
}

float NavGrid::NodeDistance(const NavNode& node, const Vec3& pos, float zWeight) const
{
    const float dx = node.pos.x - pos.x;
    const float dy = node.pos.y - pos.y;
    const float dz = (node.pos.z - pos.z) * zWeight;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void NavGrid::FindNearbyNodes(const Vec3& pos, int variant, float maxDistance, float zWeight,
                              size_t maxCount, std::vector<uint32_t>& outNodes) const
{
    outNodes.clear();
    if (m_nodes.empty() || maxCount == 0)
    {
        return;
    }

    struct Candidate
    {
        float dist;
        uint32_t id;
    };
    std::vector<Candidate> found;

    const uint8_t bit = VariantBit(variant);
    int cx = 0;
    int cy = 0;
    WorldToCell(pos, cx, cy);

    const int maxRadius = static_cast<int>(std::ceil(maxDistance / m_cellSize)) + 1;

    for (int radius = 0; radius <= maxRadius; ++radius)
    {
        for (int gx = cx - radius; gx <= cx + radius; ++gx)
        {
            const bool edgeColumn = (gx == cx - radius || gx == cx + radius);
            // На боковых сторонах кольца перебираем всю сторону, иначе — только две клетки.
            const int step = edgeColumn ? 1 : (radius * 2);
            for (int gy = cy - radius; gy <= cy + radius; gy += (step > 0 ? step : 1))
            {
                for (uint32_t id = ColumnHead(gx, gy); id != kInvalidNodeId; id = m_nodes[id].nextInColumn)
                {
                    const NavNode& node = m_nodes[id];
                    if ((node.variantMask & bit) == 0)
                    {
                        continue;
                    }
                    const float dist = NodeDistance(node, pos, zWeight);
                    if (dist <= maxDistance)
                    {
                        found.push_back(Candidate{ dist, id });
                    }
                }
                if (radius == 0)
                {
                    break;
                }
            }
        }

        // Клетки следующего кольца не ближе radius*cellSize по XY: если уже набрали
        // maxCount кандидатов ближе этого — дальше искать незачем.
        if (found.size() >= maxCount)
        {
            std::sort(found.begin(), found.end(),
                      [](const Candidate& a, const Candidate& b) { return a.dist < b.dist; });
            if (found[maxCount - 1].dist <= static_cast<float>(radius) * m_cellSize)
            {
                break;
            }
        }
    }

    std::sort(found.begin(), found.end(),
              [](const Candidate& a, const Candidate& b) { return a.dist < b.dist; });
    const size_t count = std::min(found.size(), maxCount);
    outNodes.reserve(count);
    for (size_t i = 0; i < count; ++i)
    {
        outNodes.push_back(found[i].id);
    }
}

uint32_t NavGrid::FindNearestNode(const Vec3& pos, int variant, float maxDistance, float zWeight) const
{
    std::vector<uint32_t> nodes;
    FindNearbyNodes(pos, variant, maxDistance, zWeight, 1, nodes);
    return nodes.empty() ? kInvalidNodeId : nodes.front();
}

void NavGrid::ComputeClearance()
{
    const int variantCount = Spec().variantCount;
    std::vector<uint32_t> queue;
    queue.reserve(m_nodes.size());

    for (int variant = 0; variant < variantCount; ++variant)
    {
        const uint8_t bit = VariantBit(variant);
        queue.clear();

        for (uint32_t id = 0; id < m_nodes.size(); ++id)
        {
            NavNode& node = m_nodes[id];
            bool boundary = (node.variantMask & bit) == 0;
            if (!boundary)
            {
                // Граница — если хотя бы в одну из четырёх сторон нельзя просто шагнуть.
                for (int dir = 0; dir < 4; ++dir)
                {
                    if (node.edgeType[dir] != kEdgeWalk || (node.edgeMask[dir] & bit) == 0)
                    {
                        boundary = true;
                        break;
                    }
                }
            }

            if (boundary)
            {
                node.wallDist[variant] = 0;
                queue.push_back(id);
            }
            else
            {
                node.wallDist[variant] = kMaxWallDist;
            }
        }

        // Многоисточниковый BFS от границ внутрь, по всем восьми направлениям.
        for (size_t head = 0; head < queue.size(); ++head)
        {
            const NavNode& node = m_nodes[queue[head]];
            const uint8_t nextDist = static_cast<uint8_t>(node.wallDist[variant] + 1);
            if (nextDist >= kMaxWallDist)
            {
                continue;
            }

            for (int dir = 0; dir < kNavDirCount; ++dir)
            {
                const uint32_t target = node.edgeTarget[dir];
                if (target == kInvalidNodeId || node.edgeType[dir] != kEdgeWalk)
                {
                    continue;
                }
                NavNode& other = m_nodes[target];
                if (other.wallDist[variant] > nextDist)
                {
                    other.wallDist[variant] = nextDist;
                    queue.push_back(target);
                }
            }
        }
    }
}

bool NavGrid::LineOfSight(uint32_t a, uint32_t b, int variant, uint8_t minWallDist, uint8_t avoidFlags) const
{
    if (a >= m_nodes.size() || b >= m_nodes.size())
    {
        return false;
    }
    if (a == b)
    {
        return true;
    }

    const NavNode& nodeA = m_nodes[a];
    const NavNode& nodeB = m_nodes[b];
    const int dx = nodeB.gx - nodeA.gx;
    const int dy = nodeB.gy - nodeA.gy;
    if (dx == 0 && dy == 0)
    {
        return false; // одна колонка, разные этажи
    }

    const int nx = std::abs(dx);
    const int ny = std::abs(dy);
    const int sx = dx > 0 ? 1 : (dx < 0 ? -1 : 0);
    const int sy = dy > 0 ? 1 : (dy < 0 ? -1 : 0);

    // Линия вдоль оси или точной диагонали идёт ровно по рёбрам сетки, каждое из
    // которых уже проверено свипом хулла — отступ от стен для неё не нужен.
    const bool pureLine = (nx == 0 || ny == 0 || nx == ny);
    const uint8_t bit = VariantBit(variant);

    uint32_t current = a;
    int ix = 0;
    int iy = 0;

    while (ix < nx || iy < ny)
    {
        // Обход клеток, которые пересекает отрезок между центрами (supercover).
        const int64_t lhs = static_cast<int64_t>(1 + 2 * ix) * ny;
        const int64_t rhs = static_cast<int64_t>(1 + 2 * iy) * nx;

        int stepX = 0;
        int stepY = 0;
        if (lhs == rhs)
        {
            stepX = sx;
            stepY = sy;
            ++ix;
            ++iy;
        }
        else if (lhs < rhs)
        {
            stepX = sx;
            ++ix;
        }
        else
        {
            stepY = sy;
            ++iy;
        }

        const int dir = NavDirFromDelta(stepX, stepY);
        const NavNode& node = m_nodes[current];
        if (dir < 0 || node.edgeType[dir] != kEdgeWalk || (node.edgeMask[dir] & bit) == 0)
        {
            return false;
        }

        current = node.edgeTarget[dir];
        if (current == kInvalidNodeId)
        {
            return false;
        }

        if ((m_nodes[current].flags & avoidFlags) != 0)
        {
            return false;
        }

        const bool isLast = !(ix < nx || iy < ny);
        if (!isLast && !pureLine && m_nodes[current].wallDist[variant] < minWallDist)
        {
            return false;
        }
    }

    return current == b;
}

} // namespace goldnav
