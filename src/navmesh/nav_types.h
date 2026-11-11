// Базовые типы навигации: хуллы движка и "слои" навмеша.
//
// GoldSource сталкивает entity с BSP-миром НЕ по её реальному bbox, а по одному из
// четырёх предвычисленных хуллов карты (см. SV_HullForBsp в движке):
//   ширина <= 8            -> point hull (точка в origin)
//   ширина <= 36, h <= 36  -> head hull   32x32x36
//   ширина <= 36, h >  36  -> human hull  32x32x72
//   ширина >  36           -> large hull  64x64x64
// Поэтому навмеш строится ровно под эти четыре размера, а не под произвольные.
//
// Хуллы с разной ШИРИНОЙ опираются на пол по-разному (широкий хулл "висит" на краю
// уступа там, где точка уже падает вниз), поэтому у каждой ширины свой граф — "слой".
// Хуллы одной ширины, но разной высоты (ducking/standing) делят один граф и
// различаются битом варианта на узлах/рёбрах.
#pragma once

#include <cstdint>
#include "core/gn_math.h"
#include "pathfinder/graph_interface.h"

namespace goldnav {

// Значения совпадают с параметром hullType публичного API — не переставлять.
enum class HullIndex : uint8_t
{
    Point = 0,
    Ducking = 1,
    Standing = 2,
    Large = 3,
    Count = 4,
};

struct HullDef
{
    const char* name;
    float halfWidth; // половина стороны квадратного основания
    float height;    // полная высота
    HullIndex index;
};

const HullDef& GetHullDef(HullIndex index);

inline HullIndex ClampHullIndex(int hullType)
{
    if (hullType < 0) return HullIndex::Point;
    if (hullType >= static_cast<int>(HullIndex::Count)) return HullIndex::Standing;
    return static_cast<HullIndex>(hullType);
}

enum class NavLayer : uint8_t
{
    Point = 0,  // точечный хулл
    Medium = 1, // основание 32x32: ducking + standing
    Large = 2,  // основание 64x64
    Count = 3,
};

constexpr int kMaxLayerVariants = 2;

struct LayerSpec
{
    NavLayer layer;
    int variantCount;
    // От низкого к высокому: variants[0] используется для поиска пола.
    HullIndex variants[kMaxLayerVariants];
};

const LayerSpec& GetLayerSpec(NavLayer layer);

// В каком слое живёт хулл и какой у него номер варианта (бит = 1 << variant).
NavLayer LayerOfHull(HullIndex hull);
int VariantOfHull(HullIndex hull);

inline uint8_t VariantBit(int variant)
{
    return static_cast<uint8_t>(1u << variant);
}

} // namespace goldnav
