// Размеры стандартных hull'ов GoldSource (см. gHullMins/gHullMaxs в движке и
// pm_shared.c в HLSDK). Движок знает только эти четыре размера — см. nav_types.h.
#pragma once

namespace goldnav
{
    constexpr float kHullPointHalfWidth = 0.0f;
    constexpr float kHullPointHeight = 0.0f;

    constexpr float kHullDuckingHalfWidth = 16.0f;   // 32x32x36 (head_hull)
    constexpr float kHullDuckingHeight = 36.0f;

    constexpr float kHullStandingHalfWidth = 16.0f;  // 32x32x72 (human_hull)
    constexpr float kHullStandingHeight = 72.0f;

    constexpr float kHullLargeHalfWidth = 32.0f;     // 64x64x64 (large_hull)
    constexpr float kHullLargeHeight = 64.0f;

    constexpr float kDefaultStepHeight = 18.0f;      // sv_stepsize по умолчанию
}
