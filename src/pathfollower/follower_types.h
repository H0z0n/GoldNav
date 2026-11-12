#pragma once

#include <vector>
#include "core/gn_math.h"

namespace goldnav {

enum class FollowResult
{
    Moving,  // идём
    Arrived, // дошли до последней точки пути
    Stuck,   // нет продвижения — путь надо перестроить
    OffPath, // оказались не там, где ждал путь (упали, столкнули) — путь надо перестроить
    NoPath,  // пути нет
};

// Состояние следования по пути для одного entity.
struct FollowState
{
    std::vector<Vec3> path; // feet-позиции
    size_t currentIndex = 0;
    bool active = false;

    // Откуда начали идти к текущей точке — чтобы понять, что её "проскочили".
    Vec3 segmentStart;
    bool hasSegmentStart = false;

    // Окно контроля продвижения.
    float windowTime = 0.0f;     // сколько длится текущее окно
    float windowIntended = 0.0f; // сколько собирались пройти за окно
    float windowStartDist = 0.0f; // расстояние до текущей точки в начале окна
    size_t windowIndex = 0;      // индекс точки в начале окна
    bool windowValid = false;
    float windowSidestep = 0.0f; // сколько времени окна ушло на обход боком
    int stuckStrikes = 0;        // сколько окон подряд без продвижения
    int idleWindows = 0;         // сколько окон подряд ушло на обход без продвижения

    // Текущая горизонтальная скорость (для инерции). Переживает смену пути.
    Vec3 velocity;

    // На какую высоту entity умеет запрыгивать (0 — не прыгает) и пауза до следующей попытки.
    float jumpHeight = 0.0f;
    float jumpCooldown = 0.0f;

    // Обход препятствия боком.
    float sidestepTimeLeft = 0.0f;
    Vec3 sidestepDir;
    int sidestepSign = 1;

    void SetPath(const std::vector<Vec3>& newPath)
    {
        path = newPath;
        currentIndex = 0;
        active = !path.empty();
        hasSegmentStart = false;
        windowValid = false;
        stuckStrikes = 0;
        idleWindows = 0;
        sidestepTimeLeft = 0.0f;
    }

    void Reset()
    {
        SetPath(std::vector<Vec3>());
    }
};

} // namespace goldnav
