#include "nav_types.h"
#include "nav_hull_defs.h"

namespace goldnav {

const HullDef& GetHullDef(HullIndex index)
{
    static const HullDef kDefs[static_cast<size_t>(HullIndex::Count)] = {
        { "point",    kHullPointHalfWidth,    kHullPointHeight,    HullIndex::Point },
        { "ducking",  kHullDuckingHalfWidth,  kHullDuckingHeight,  HullIndex::Ducking },
        { "standing", kHullStandingHalfWidth, kHullStandingHeight, HullIndex::Standing },
        { "large",    kHullLargeHalfWidth,    kHullLargeHeight,    HullIndex::Large },
    };

    const size_t i = static_cast<size_t>(index);
    return kDefs[i < static_cast<size_t>(HullIndex::Count) ? i : static_cast<size_t>(HullIndex::Standing)];
}

const LayerSpec& GetLayerSpec(NavLayer layer)
{
    static const LayerSpec kSpecs[static_cast<size_t>(NavLayer::Count)] = {
        { NavLayer::Point,  1, { HullIndex::Point,   HullIndex::Point } },
        { NavLayer::Medium, 2, { HullIndex::Ducking, HullIndex::Standing } },
        { NavLayer::Large,  1, { HullIndex::Large,   HullIndex::Large } },
    };

    const size_t i = static_cast<size_t>(layer);
    return kSpecs[i < static_cast<size_t>(NavLayer::Count) ? i : static_cast<size_t>(NavLayer::Medium)];
}

NavLayer LayerOfHull(HullIndex hull)
{
    switch (hull)
    {
        case HullIndex::Point: return NavLayer::Point;
        case HullIndex::Large: return NavLayer::Large;
        default:               return NavLayer::Medium;
    }
}

int VariantOfHull(HullIndex hull)
{
    return hull == HullIndex::Standing ? 1 : 0;
}

} // namespace goldnav
