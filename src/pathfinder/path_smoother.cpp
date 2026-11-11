#include "path_smoother.h"

namespace goldnav {

std::vector<size_t> SmoothPathIndices(size_t pointCount, const DirectPathPredicate& canGoDirect,
                                      size_t maxLookahead)
{
    std::vector<size_t> result;
    if (pointCount == 0)
    {
        return result;
    }

    result.reserve(pointCount);
    result.push_back(0);

    if (!canGoDirect || pointCount < 3)
    {
        for (size_t i = 1; i < pointCount; ++i)
        {
            result.push_back(i);
        }
        return result;
    }

    size_t anchor = 0;
    while (anchor < pointCount - 1)
    {
        // Соседняя точка достижима всегда — это ребро сырого пути.
        size_t farthest = anchor + 1;

        size_t limit = pointCount - 1;
        if (maxLookahead != 0 && anchor + maxLookahead < limit)
        {
            limit = anchor + maxLookahead;
        }

        // Не останавливаемся на первом отказе: за углом прямая может оборваться,
        // а чуть дальше — снова появиться, и срезать выгоднее именно туда.
        for (size_t candidate = anchor + 2; candidate <= limit; ++candidate)
        {
            if (canGoDirect(anchor, candidate))
            {
                farthest = candidate;
            }
        }

        result.push_back(farthest);
        anchor = farthest;
    }

    return result;
}

} // namespace goldnav
