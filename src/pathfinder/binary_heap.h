// Индексированная бинарная min-heap с O(log n) DecreaseKey.
// Ключи — произвольный uint32_t id (в pathfinder это nodeId), приоритет — TPriority
// (в A* — f-score). Индексация по id позволяет находить и уменьшать приоритет
// уже вставленного элемента без линейного скана открытого множества.
#pragma once

#include <vector>
#include <cstdint>
#include <cstddef>
#include <limits>
#include <algorithm>

namespace goldnav {

template <typename TPriority>
class IndexedBinaryHeap
{
public:
    // capacityHint — общее число возможных id (обычно NodeCount графа).
    explicit IndexedBinaryHeap(size_t capacityHint = 0)
    {
        Reserve(capacityHint);
    }

    // Только наращивает таблицу позиций — уже лежащие в куче элементы не трогает.
    void Reserve(size_t capacityHint)
    {
        if (capacityHint > m_positionOf.size())
        {
            m_positionOf.resize(capacityHint, kNotInHeap);
        }
    }

    // O(размер кучи), а не O(число id): сбрасываются позиции только тех элементов,
    // что реально лежат в куче. Важно, когда поиск пути на графе в сотни тысяч узлов
    // затрагивает лишь малую его часть.
    void Clear()
    {
        for (const Entry& entry : m_heap)
        {
            m_positionOf[entry.id] = kNotInHeap;
        }
        m_heap.clear();
    }

    bool Empty() const { return m_heap.empty(); }
    size_t Size() const { return m_heap.size(); }

    bool Contains(uint32_t id) const
    {
        return id < m_positionOf.size() && m_positionOf[id] != kNotInHeap;
    }

    void Push(uint32_t id, TPriority priority)
    {
        EnsureCapacity(id);

        if (Contains(id))
        {
            DecreaseKey(id, priority);
            return;
        }

        const size_t pos = m_heap.size();
        m_heap.push_back(Entry{id, priority});
        m_positionOf[id] = pos;
        SiftUp(pos);
    }

    // Не увеличивает приоритет — если newPriority хуже текущего, вызов игнорируется
    // (в A* это соответствует "нашли путь не лучше уже известного").
    void DecreaseKey(uint32_t id, TPriority newPriority)
    {
        if (!Contains(id))
        {
            return;
        }
        const size_t pos = m_positionOf[id];
        if (!(newPriority < m_heap[pos].priority))
        {
            return;
        }
        m_heap[pos].priority = newPriority;
        SiftUp(pos);
    }

    uint32_t PopMin()
    {
        const uint32_t result = m_heap.front().id;
        m_positionOf[result] = kNotInHeap;

        m_heap.front() = m_heap.back();
        m_heap.pop_back();

        if (!m_heap.empty())
        {
            m_positionOf[m_heap.front().id] = 0;
            SiftDown(0);
        }

        return result;
    }

private:
    struct Entry
    {
        uint32_t id;
        TPriority priority;
    };

    static constexpr size_t kNotInHeap = static_cast<size_t>(-1);

    std::vector<Entry> m_heap;
    std::vector<size_t> m_positionOf;

    void EnsureCapacity(uint32_t id)
    {
        if (id >= m_positionOf.size())
        {
            m_positionOf.resize(static_cast<size_t>(id) + 1, kNotInHeap);
        }
    }

    void SiftUp(size_t pos)
    {
        while (pos > 0)
        {
            const size_t parent = (pos - 1) / 2;
            if (!(m_heap[pos].priority < m_heap[parent].priority))
            {
                break;
            }
            Swap(pos, parent);
            pos = parent;
        }
    }

    void SiftDown(size_t pos)
    {
        const size_t n = m_heap.size();
        for (;;)
        {
            const size_t left = pos * 2 + 1;
            const size_t right = pos * 2 + 2;
            size_t smallest = pos;

            if (left < n && m_heap[left].priority < m_heap[smallest].priority)
            {
                smallest = left;
            }
            if (right < n && m_heap[right].priority < m_heap[smallest].priority)
            {
                smallest = right;
            }
            if (smallest == pos)
            {
                break;
            }
            Swap(pos, smallest);
            pos = smallest;
        }
    }

    void Swap(size_t a, size_t b)
    {
        std::swap(m_heap[a], m_heap[b]);
        m_positionOf[m_heap[a].id] = a;
        m_positionOf[m_heap[b].id] = b;
    }
};

} // namespace goldnav
