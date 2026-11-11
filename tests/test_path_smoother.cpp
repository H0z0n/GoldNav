#include "micro_test.h"
#include "pathfinder/path_smoother.h"

using goldnav::SmoothPathIndices;

static void Test_NoPredicate_KeepsEverything()
{
    const auto kept = SmoothPathIndices(4, nullptr);
    GN_CHECK(kept.size() == 4);
    GN_CHECK(kept.front() == 0 && kept.back() == 3);
}

static void Test_AllDirect_CollapsesToEndpoints()
{
    const auto kept = SmoothPathIndices(10, [](size_t, size_t) { return true; });
    GN_CHECK(kept.size() == 2);
    GN_CHECK(kept[0] == 0 && kept[1] == 9);
}

static void Test_NothingDirect_KeepsEverything()
{
    const auto kept = SmoothPathIndices(5, [](size_t, size_t) { return false; });
    GN_CHECK(kept.size() == 5);
    for (size_t i = 0; i < kept.size(); ++i) GN_CHECK(kept[i] == i);
}

static void Test_Corner_KeepsTheCornerPoint()
{
    // Угол в точке 3: напрямую видно только в пределах своей "стороны" угла.
    const auto direct = [](size_t from, size_t to) { return (from < 3 && to <= 3) || (from >= 3); };
    const auto kept = SmoothPathIndices(8, direct);
    GN_CHECK(kept.size() == 3);
    GN_CHECK(kept[0] == 0 && kept[1] == 3 && kept[2] == 7);
}

static void Test_GapInVisibility_StillTakesTheFarthest()
{
    // От 0 не видно точку 2, но видно 3 и 4 — должны срезать сразу до 4.
    const auto direct = [](size_t from, size_t to) { return !(from == 0 && to == 2); };
    const auto kept = SmoothPathIndices(5, direct);
    GN_CHECK(kept.size() == 2);
    GN_CHECK(kept[1] == 4);
}

static void Test_Lookahead_LimitsHowFarWeTry()
{
    const auto kept = SmoothPathIndices(10, [](size_t, size_t) { return true; }, 3);
    GN_CHECK(kept.size() == 4); // 0, 3, 6, 9
    GN_CHECK(kept[1] == 3 && kept[2] == 6 && kept[3] == 9);
}

static void Test_TinyPaths()
{
    GN_CHECK(SmoothPathIndices(0, nullptr).empty());
    GN_CHECK(SmoothPathIndices(1, [](size_t, size_t) { return true; }).size() == 1);
    GN_CHECK(SmoothPathIndices(2, [](size_t, size_t) { return true; }).size() == 2);
}

static void RunAllTests()
{
    GN_RUN(Test_NoPredicate_KeepsEverything);
    GN_RUN(Test_AllDirect_CollapsesToEndpoints);
    GN_RUN(Test_NothingDirect_KeepsEverything);
    GN_RUN(Test_Corner_KeepsTheCornerPoint);
    GN_RUN(Test_GapInVisibility_StillTakesTheFarthest);
    GN_RUN(Test_Lookahead_LimitsHowFarWeTry);
    GN_RUN(Test_TinyPaths);
}

GN_TEST_MAIN()
