// Минимальный header-only test-harness — без внешних зависимостей (никакого
// gtest/Catch2), т.к. проект договорился не тянуть тяжёлые библиотеки.
#pragma once

#include <cstdio>

namespace goldnav_test {
inline int g_failures = 0;
inline int g_checks = 0;
} // namespace goldnav_test

#define GN_CHECK(cond) \
    do { \
        ++goldnav_test::g_checks; \
        if (!(cond)) { \
            std::fprintf(stderr, "  [FAIL] %s:%d: CHECK(%s)\n", __FILE__, __LINE__, #cond); \
            ++goldnav_test::g_failures; \
        } \
    } while (0)

#define GN_RUN(testFn) \
    do { \
        std::fprintf(stderr, "[RUN ] %s\n", #testFn); \
        testFn(); \
    } while (0)

#define GN_TEST_MAIN() \
    int main() { \
        RunAllTests(); \
        if (goldnav_test::g_failures == 0) { \
            std::fprintf(stderr, "[OK] %d checks passed\n", goldnav_test::g_checks); \
            return 0; \
        } \
        std::fprintf(stderr, "[FAILED] %d/%d checks failed\n", goldnav_test::g_failures, goldnav_test::g_checks); \
        return 1; \
    }
