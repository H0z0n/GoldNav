#include "micro_test.h"
#include "pathfinder/binary_heap.h"
#include <vector>
#include <random>
#include <algorithm>

using goldnav::IndexedBinaryHeap;

static void Test_PushPopOrder()
{
    IndexedBinaryHeap<float> heap;
    heap.Push(3, 3.0f);
    heap.Push(1, 1.0f);
    heap.Push(2, 2.0f);
    heap.Push(0, 0.5f);

    GN_CHECK(heap.Size() == 4);
    GN_CHECK(heap.PopMin() == 0);
    GN_CHECK(heap.PopMin() == 1);
    GN_CHECK(heap.PopMin() == 2);
    GN_CHECK(heap.PopMin() == 3);
    GN_CHECK(heap.Empty());
}

static void Test_DecreaseKeyReordersCorrectly()
{
    IndexedBinaryHeap<float> heap;
    heap.Push(10, 100.0f);
    heap.Push(20, 50.0f);
    heap.Push(30, 75.0f);

    // 20 сейчас минимум; понижаем 30 ниже него.
    heap.DecreaseKey(30, 10.0f);
    GN_CHECK(heap.PopMin() == 30);
    GN_CHECK(heap.PopMin() == 20);
    GN_CHECK(heap.PopMin() == 10);
}

static void Test_DecreaseKeyIgnoresWorsePriority()
{
    IndexedBinaryHeap<float> heap;
    heap.Push(1, 5.0f);
    heap.DecreaseKey(1, 10.0f); // хуже текущего — должно быть проигнорировано
    GN_CHECK(heap.Size() == 1);
    // Всё ещё 5.0, а не 10.0 — проверяем косвенно через порядок с другим элементом.
    heap.Push(2, 7.0f);
    GN_CHECK(heap.PopMin() == 1);
}

static void Test_PushExistingIdActsAsDecreaseKey()
{
    IndexedBinaryHeap<float> heap;
    heap.Push(5, 100.0f);
    heap.Push(5, 1.0f); // повторный Push того же id = DecreaseKey
    GN_CHECK(heap.Size() == 1);
    GN_CHECK(heap.PopMin() == 5);
}

static void Test_StressRandomOrderIsSorted()
{
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> dist(0.0f, 100000.0f);

    IndexedBinaryHeap<float> heap;
    std::vector<float> priorities;
    constexpr uint32_t kCount = 2000;

    for (uint32_t id = 0; id < kCount; ++id)
    {
        const float p = dist(rng);
        priorities.push_back(p);
        heap.Push(id, p);
    }

    std::vector<float> sortedPriorities = priorities;
    std::sort(sortedPriorities.begin(), sortedPriorities.end());

    std::vector<float> poppedInOrder;
    while (!heap.Empty())
    {
        const uint32_t id = heap.PopMin();
        poppedInOrder.push_back(priorities[id]);
    }

    GN_CHECK(poppedInOrder.size() == kCount);
    bool sorted = true;
    for (size_t i = 1; i < poppedInOrder.size(); ++i)
    {
        if (poppedInOrder[i] < poppedInOrder[i - 1])
        {
            sorted = false;
            break;
        }
    }
    GN_CHECK(sorted);
}

static void RunAllTests()
{
    GN_RUN(Test_PushPopOrder);
    GN_RUN(Test_DecreaseKeyReordersCorrectly);
    GN_RUN(Test_DecreaseKeyIgnoresWorsePriority);
    GN_RUN(Test_PushExistingIdActsAsDecreaseKey);
    GN_RUN(Test_StressRandomOrderIsSorted);
}

GN_TEST_MAIN()
