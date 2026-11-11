#include "platform_time.h"

#if defined(_WIN32)
    #define WIN32_LEAN_AND_MEAN
    #include <windows.h>
#else
    #include <time.h>
#endif

namespace goldnav {

#if defined(_WIN32)

double PlatformNowMs()
{
    static LARGE_INTEGER frequency{};
    static const bool haveFrequency = QueryPerformanceFrequency(&frequency) != 0;

    LARGE_INTEGER counter;
    if (!haveFrequency || !QueryPerformanceCounter(&counter))
    {
        return 0.0;
    }

    return (static_cast<double>(counter.QuadPart) / static_cast<double>(frequency.QuadPart)) * 1000.0;
}

#else

double PlatformNowMs()
{
    struct timespec ts{};
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
    {
        return 0.0;
    }
    return static_cast<double>(ts.tv_sec) * 1000.0 + static_cast<double>(ts.tv_nsec) / 1'000'000.0;
}

#endif

} // namespace goldnav
