// NavGrid — граф одного слоя навмеша (см. NavLayer в nav_types.h).
//
// Узел = место, где хулл слоя может стоять: центр клетки сетки по XY + высота пола.
// В одной колонке XY может быть несколько узлов на разной высоте (этажи, мосты,
// тоннели) — они связаны в список через nextInColumn. Мировых границ сетка не
// требует: координаты клеток считаются от нуля мира, узлы появляются там, куда
// дошёл генератор (см. nav_generator.h).
//
// Рёбра хранятся по восьми направлениям сетки. У каждого ребра — маска вариантов
// хулла, которые могут по нему пройти, и тип: Walk (обычный шаг, в т.ч. ступенька)
// или Drop (односторонний спрыгивание с уступа).
#pragma once

#include <vector>
#include <unordered_map>
#include <cstdint>
#include "nav_types.h"

namespace goldnav {

constexpr int kNavDirCount = 8;
extern const int kNavDirDx[kNavDirCount];
extern const int kNavDirDy[kNavDirCount];

// Индекс направления по смещению клетки (-1..1 по каждой оси), -1 для (0,0).
int NavDirFromDelta(int dx, int dy);
int NavOppositeDir(int dir);
inline bool NavDirIsDiagonal(int dir) { return dir >= 4; }

enum : uint8_t
{
    kEdgeNone = 0,
    kEdgeWalk = 1,
    kEdgeDrop = 2,
    kEdgeJump = 3, // запрыгнуть на уступ выше ступеньки
};

enum : uint8_t
{
    kNodeWater = 1 << 0,     // пол под водой
    kNodeDeepWater = 1 << 1, // вода выше пояса
};

constexpr uint8_t kMaxWallDist = 3;

inline uint8_t NavEncodeHeight(float height)
{
    const float half = height * 0.5f + 0.5f;
    return half <= 0.0f ? 0 : (half >= 255.0f ? 255 : static_cast<uint8_t>(half));
}

inline float NavDecodeHeight(uint8_t encoded)
{
    return static_cast<float>(encoded) * 2.0f;
}

struct NavNode
{
    Vec3 pos; // feet: центр клетки по XY, высота пола по Z
    int32_t gx = 0;
    int32_t gy = 0;
    uint32_t nextInColumn = kInvalidNodeId;

    uint32_t edgeTarget[kNavDirCount];
    uint8_t edgeMask[kNavDirCount];
    uint8_t edgeType[kNavDirCount];
    // Для Drop — высота спрыгивания, для Jump — высота подъёма (в двойных юнитах, см.
    // NavEncodeHeight): разные entity прыгают по-разному, фильтр — на этапе поиска пути.
    uint8_t edgeHeight[kNavDirCount];

    uint8_t variantMask = 0; // какие варианты хулла здесь помещаются
    uint8_t flags = 0;
    // Расстояние в клетках до ближайшей "границы" (стена, обрыв, низкий потолок)
    // для каждого варианта, не больше kMaxWallDist. 0 — узел вплотную к границе.
    uint8_t wallDist[kMaxLayerVariants] = { 0, 0 };
    bool expanded = false;
    bool revalidate = false; // узел уже раскрывался: при повторном раскрытии перепроверить его самого

    NavNode()
    {
        for (int i = 0; i < kNavDirCount; ++i)
        {
            edgeTarget[i] = kInvalidNodeId;
            edgeMask[i] = 0;
            edgeType[i] = kEdgeNone;
            edgeHeight[i] = 0;
        }
    }
};

class NavGrid
{
public:
    void Initialize(NavLayer layer, float cellSize);
    void Clear();

    NavLayer Layer() const { return m_layer; }
    const LayerSpec& Spec() const { return GetLayerSpec(m_layer); }
    float CellSize() const { return m_cellSize; }

    size_t NodeCount() const { return m_nodes.size(); }
    size_t ColumnCount() const { return m_columnHead.size(); }
    const NavNode& Node(uint32_t id) const { return m_nodes[id]; }
    NavNode& Node(uint32_t id) { return m_nodes[id]; }

    void WorldToCell(const Vec3& pos, int& outGx, int& outGy) const;
    float CellCenter(int cell) const { return (static_cast<float>(cell) + 0.5f) * m_cellSize; }

    uint32_t ColumnHead(int gx, int gy) const;

    // Узел колонки, чей пол отличается от z не больше чем на tolerance (ближайший).
    uint32_t FindNodeInColumn(int gx, int gy, float z, float tolerance) const;

    uint32_t AddNode(int gx, int gy, float z, uint8_t variantMask, uint8_t flags);

    void SetEdge(uint32_t from, int dir, uint32_t to, uint8_t variantMask, uint8_t type, float height = 0.0f);
    void ClearEdges(uint32_t nodeId);

    // Ближайший к точке узел, где помещается вариант variant. zWeight > 1 делает
    // разницу по высоте "дороже" (чтобы не прилипать к этажу выше/ниже).
    // maxDistance ограничивает и XY-радиус поиска, и итоговое расстояние.
    uint32_t FindNearestNode(const Vec3& pos, int variant, float maxDistance, float zWeight = 1.0f) const;

    // До maxCount ближайших узлов, отсортированных по возрастанию расстояния.
    void FindNearbyNodes(const Vec3& pos, int variant, float maxDistance, float zWeight,
                         size_t maxCount, std::vector<uint32_t>& outNodes) const;

    // Пересчёт wallDist для всех узлов. Вызывается генератором, когда очередь пуста.
    void ComputeClearance();

    // Можно ли пройти по прямой от узла a до узла b, двигаясь только по Walk-рёбрам
    // варианта variant. Для линий, не совпадающих с осью/диагональю сетки, все
    // промежуточные клетки обязаны иметь wallDist >= minWallDist (иначе хулл, идущий
    // не по центрам клеток, может зацепить угол).
    // avoidFlags: узлы с любым из этих флагов (вода и т.п.) прямая пересекать не должна.
    bool LineOfSight(uint32_t a, uint32_t b, int variant, uint8_t minWallDist, uint8_t avoidFlags = 0) const;

    // Версия графа: растёт при добавлении узлов/рёбер. Нужна, чтобы понять, что
    // кэшированный результат (например, "пути нет") устарел.
    uint32_t Revision() const { return m_revision; }

private:
    static int64_t CellKey(int gx, int gy)
    {
        return (static_cast<int64_t>(gx) << 32) | static_cast<int64_t>(static_cast<uint32_t>(gy));
    }

    float NodeDistance(const NavNode& node, const Vec3& pos, float zWeight) const;

    NavLayer m_layer = NavLayer::Medium;
    float m_cellSize = 16.0f;
    uint32_t m_revision = 0;

    std::vector<NavNode> m_nodes;
    std::unordered_map<int64_t, uint32_t> m_columnHead;
};

} // namespace goldnav
