// PathFollower — ведёт entity по пути: выбирает текущую точку, задаёт направление и
// скорость, двигает через GroundMove, следит за продвижением. Engine-agnostic.
//
// Что здесь "как у человека":
//   - точка пути считается пройденной и по радиусу, и когда её проскочили — никакого
//     кружения вокруг неё;
//   - высота учитывается: точка этажом выше/ниже не засчитывается;
//   - перед целью — плавное торможение и остановка точно в ней;
//   - упёрлись в другую entity — обходим её боком, а не толкаемся в лоб;
//   - нет продвижения — не стоим вечно, а сообщаем наверх, чтобы путь перестроили;
//   - скорость меняется с инерцией: плавный разгон и повороты по дуге;
//   - если entity умеет прыгать (FollowState::jumpHeight), запрыгивает на уступы.
#pragma once

#include "follower_types.h"
#include "ground_mover.h"

namespace goldnav {

struct PathFollowerConfig
{
    float waypointRadius = 6.0f;        // минимальный радиус "дошли" для промежуточной точки
    float turnInPerSpeed = 0.06f;       // на скорости радиус растёт: speed * это (поворот начинается заранее)
    float maxTurnInRadius = 16.0f;      // ...но не больше этого
    float acceleration = 6.0f;          // инерция: за секунду скорость меняется не больше чем на speed * это (0 — без инерции)
    float jumpMargin = 10.0f;           // прыгаем на столько юнитов выше, чем нужно
    float jumpCooldown = 0.6f;          // пауза между попытками прыжка, с
    float jumpMaxDistance = 48.0f;      // прыгаем, только если точка на уступе не дальше этого
    float waypointHeightTolerance = 16.0f; // и допустимая разница высот с ней: точка на уступе над головой не считается
    float finalRadius = 2.0f;           // радиус "дошли" для последней точки
    float heightTolerance = 40.0f;      // допустимая разница высот с точкой
    float slowDownRadius = 40.0f;       // с какого расстояния до цели тормозим
    float minSpeedFactor = 0.3f;        // до какой доли скорости тормозим
    float passLateralLimit = 32.0f;     // "проскочили точку" засчитывается только не дальше этого
    float progressWindow = 0.4f;        // длина окна контроля продвижения, с
    float minProgressFraction = 0.2f;   // меньше этой доли от задуманного — "нет продвижения"
    int strikesBeforeStuck = 2;         // сколько окон подряд без продвижения = Stuck
    int idleWindowsBeforeStuck = 5;     // сколько окон подряд можно обходить препятствие без продвижения
    float sidestepTime = 0.6f;          // максимум одного обхода боком
    float sidestepClearTime = 0.08f;    // сколько ещё идём вбок после того, как перестали касаться препятствия
    float offPathLandingDistance = 28.0f; // приземлились дальше этого от текущей точки — путь устарел
    MoverConfig mover;
};

struct FollowStep
{
    FollowResult result = FollowResult::NoPath;
    Vec3 feet;                // новая позиция entity
    Vec3 facing;              // куда entity смотрит/идёт (горизонтальный единичный вектор; нулевой — не менять)
    float speed = 0.0f;       // фактическая горизонтальная скорость за шаг
    int touchedEntity = -1;   // entity, которой коснулись за шаг
    bool blocked = false;     // упёрлись в препятствие
    bool jumped = false;      // в этом шаге начали прыжок
    int blockEntity = -1;     // во что (0 — карта, >0 — entity)
    Vec3 blockNormal;
    bool stuckInSolid = false;
};

class PathFollower
{
public:
    explicit PathFollower(const PathFollowerConfig& config = PathFollowerConfig()) : m_config(config) {}

    const PathFollowerConfig& Config() const { return m_config; }
    PathFollowerConfig& Config() { return m_config; }

    // Один шаг следования: feet — текущая позиция, speed — желаемая скорость (юнитов/с).
    FollowStep Update(FollowState& state, IMoveTracer& tracer, MoverState& mover,
                      const Vec3& feet, float speed, float dt) const;

private:
    PathFollowerConfig m_config;

    // Продвигает currentIndex, пока текущая точка достигнута. true — путь пройден.
    bool AdvanceWaypoints(FollowState& state, const Vec3& feet, bool onGround, float speed) const;
};

} // namespace goldnav
