#include "micro_test.h"
#include "fixtures/mock_graph.h"
#include "pathfinder/astar.h"

using goldnav::AStarPathfinder;
using goldnav_test::MockGridGraph;

static void Test_SimpleOpenGrid_FindsShortestPath()
{
    MockGridGraph graph({
        "......",
        "......",
        "......",
    });

    AStarPathfinder astar;
    std::vector<uint32_t> path;
    const uint32_t start = graph.NodeIdAt(0, 0);
    const uint32_t goal = graph.NodeIdAt(5, 2);

    const bool found = astar.FindPath(graph, start, goal, path);
    GN_CHECK(found);
    // Манхэттенское расстояние 5+2=7 -> 8 узлов на 4-connected сетке.
    GN_CHECK(path.size() == 8);
    GN_CHECK(!path.empty() && path.front() == start);
    GN_CHECK(!path.empty() && path.back() == goal);
}

static void Test_MazeWithWalls_GoesAroundObstacle()
{
    MockGridGraph graph({
        ".....",
        ".###.",
        ".....",
    });

    AStarPathfinder astar;
    std::vector<uint32_t> path;
    const uint32_t start = graph.NodeIdAt(0, 1);
    const uint32_t goal = graph.NodeIdAt(4, 1);

    const bool found = astar.FindPath(graph, start, goal, path);
    GN_CHECK(found);
    // Стена шириной 3 клетки (col 1..3) в row1: подняться на row0 (1 шаг), пройти
    // col0->col4 (4 шага), спуститься на row1 (1 шаг) = 6 шагов = 7 узлов.
    GN_CHECK(path.size() == 7);
}

static void Test_UnreachableGoal_ReturnsFalseWithoutHanging()
{
    MockGridGraph graph({
        "..#..",
        "..#..",
        "..#..",
    });

    AStarPathfinder astar;
    std::vector<uint32_t> path;
    const uint32_t start = graph.NodeIdAt(0, 0);
    const uint32_t goal = graph.NodeIdAt(4, 0);

    const bool found = astar.FindPath(graph, start, goal, path);
    GN_CHECK(!found);
    GN_CHECK(path.empty());
}

static void Test_SingleNodePath_StartEqualsGoal()
{
    MockGridGraph graph({"..."});

    AStarPathfinder astar;
    std::vector<uint32_t> path;
    const uint32_t node = graph.NodeIdAt(1, 0);

    const bool found = astar.FindPath(graph, node, node, path);
    GN_CHECK(found);
    GN_CHECK(path.size() == 1);
    GN_CHECK(path[0] == node);
}

static void Test_ReusedAcrossCalls_DoesNotLeakStateFromPreviousSearch()
{
    // Один и тот же AStarPathfinder используется дважды подряд с разными графами —
    // это проверяет, что generation-counter в scratch-буферах реально изолирует поиски.
    MockGridGraph graphA({"....."});
    MockGridGraph graphB({
        "..#..",
        "..#..",
    });

    AStarPathfinder astar;
    std::vector<uint32_t> path;

    GN_CHECK(astar.FindPath(graphA, graphA.NodeIdAt(0, 0), graphA.NodeIdAt(4, 0), path));
    GN_CHECK(path.size() == 5);

    GN_CHECK(!astar.FindPath(graphB, graphB.NodeIdAt(0, 0), graphB.NodeIdAt(4, 0), path));
    GN_CHECK(path.empty());
}

static void Test_Partial_GoesAsCloseAsPossible()
{
    MockGridGraph graph({
        "....#..",
        "....#..",
        "....#..",
    });

    AStarPathfinder astar;
    std::vector<uint32_t> path;
    goldnav::AStarOptions options;
    options.allowPartial = true;

    const uint32_t start = graph.NodeIdAt(0, 1);
    const uint32_t goal = graph.NodeIdAt(6, 1);
    const goldnav::AStarResult result = astar.Search(graph, start, goal, path, options);

    GN_CHECK(result == goldnav::AStarResult::Partial);
    // Упёрлись в стену напротив цели: (3,1) — ближайшая к цели достижимая клетка.
    GN_CHECK(!path.empty() && path.front() == start);
    GN_CHECK(!path.empty() && path.back() == graph.NodeIdAt(3, 1));

    // Без allowPartial — честный отказ.
    GN_CHECK(astar.Search(graph, start, goal, path) == goldnav::AStarResult::NotFound);
    GN_CHECK(path.empty());
}

static void Test_ExpansionLimit_StopsSearch()
{
    MockGridGraph graph({
        "..........",
        "..........",
        "..........",
        "..........",
    });

    AStarPathfinder astar;
    std::vector<uint32_t> path;
    goldnav::AStarOptions options;
    options.maxExpansions = 3;

    GN_CHECK(astar.Search(graph, graph.NodeIdAt(0, 0), graph.NodeIdAt(9, 3), path, options) == goldnav::AStarResult::NotFound);
    GN_CHECK(astar.LastExpansions() <= 3);
}

static void Test_InvalidNodes_AreRejected()
{
    MockGridGraph graph({"..."});
    AStarPathfinder astar;
    std::vector<uint32_t> path;
    GN_CHECK(!astar.FindPath(graph, goldnav::kInvalidNodeId, 0, path));
    GN_CHECK(!astar.FindPath(graph, 0, 999, path));
}

static void RunAllTests()
{
    GN_RUN(Test_Partial_GoesAsCloseAsPossible);
    GN_RUN(Test_ExpansionLimit_StopsSearch);
    GN_RUN(Test_InvalidNodes_AreRejected);
    GN_RUN(Test_SimpleOpenGrid_FindsShortestPath);
    GN_RUN(Test_MazeWithWalls_GoesAroundObstacle);
    GN_RUN(Test_UnreachableGoal_ReturnsFalseWithoutHanging);
    GN_RUN(Test_SingleNodePath_StartEqualsGoal);
    GN_RUN(Test_ReusedAcrossCalls_DoesNotLeakStateFromPreviousSearch);
}

GN_TEST_MAIN()
