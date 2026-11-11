/* GoldNav — навигация без waypoint-ов для GoldSource/CS 1.6.
 *
 * Подключение в .sma:  #include <goldnav>
 *
 * Требует на сервере:
 *   - goldnav_mm.dll/.so в addons/metamod/dlls/ (прописан в metamod/plugins.ini)
 *   - goldnav_amxx.dll / goldnav_amxx_i386.so в addons/amxmodx/modules/
 *     (в modules.ini прописывать не обязательно — подгрузится сам по #pragma ниже)
 *
 * Навмеш строится автоматически при старте карты и занимает несколько секунд.
 * goldnav_move_to() можно звать сразу — запрос подождёт готовности сам.
 *
 * См. docs/public_api.md в репозитории GoldNav.
 *
 * Автор: Hozon (Telegram: @AmxM0dX)
 */
#if defined _goldnav_included
    #endinput
#endif
#define _goldnav_included

#if AMXX_VERSION_NUM >= 175
    #pragma reqlib goldnav
    #if !defined AMXMODX_NOAUTOLOAD
        #pragma loadlib goldnav
    #endif
#else
    #pragma library goldnav
#endif

/* Хуллы. Движок GoldSource сталкивает entity с картой только четырьмя размерами
 * (по ширине и высоте её bbox), навмеш строится под них:
 *   ширина <= 8             -> POINT    (точка в origin, bbox не участвует)
 *   ширина <= 36, h <= 36   -> DUCKING  (32x32x36)
 *   ширина <= 36, h >  36   -> STANDING (32x32x72)
 *   ширина >  36            -> LARGE    (64x64x64)
 * Делайте bbox своей entity ровно одного из этих размеров (например,
 * mins {-16,-16,0} maxs {16,16,36}) — иначе движок сам подставит ближайший хулл,
 * причём со смещением относительно origin.
 */
#define GOLDNAV_HULL_AUTO     -1   // тот, которым движок сталкивает эту entity
#define GOLDNAV_HULL_POINT    0
#define GOLDNAV_HULL_DUCKING  1
#define GOLDNAV_HULL_STANDING 2
#define GOLDNAV_HULL_LARGE    3

/* Состояние entity (goldnav_get_state, goldnav_move_finished). */
#define GOLDNAV_STATE_IDLE    0   // GoldNav эту entity не ведёт
#define GOLDNAV_STATE_PENDING 1   // ждёт навмеш/путь
#define GOLDNAV_STATE_MOVING  2   // идёт
#define GOLDNAV_STATE_ARRIVED 3   // дошла (или подошла так близко, как возможно)
#define GOLDNAV_STATE_FAILED  4   // пути нет / застряла окончательно

/* Флаги goldnav_move_to. */
#define GOLDNAV_MOVE_NO_TURN  (1<<0)   // не поворачивать entity (angles[1]) по ходу движения
#define GOLDNAV_MOVE_NO_DROP  (1<<1)   // не спрыгивать с уступов выше ступеньки
#define GOLDNAV_MOVE_NO_TOUCH (1<<2)   // не вызывать Touch у entity, в которые упёрлись
#define GOLDNAV_MOVE_EXACT    (1<<3)   // цель недостижима -> FAILED, а не "подойти поближе"

/**
 * Отправляет entity в точку. Это основной натив: GoldNav сам находит путь, каждый
 * кадр ведёт entity по нему (ступеньки, склоны, обход стен, обход стоящих на пути
 * игроков), перестраивает путь, если entity столкнули, и останавливает её в цели.
 * Повторный вызов просто меняет цель.
 *
 * Entity может быть любой (info_target, func_*, созданная через rg_create_entity
 * и т.п.), кроме игроков. Рекомендуется MOVETYPE_PUSHSTEP: с ним клиент плавно
 * интерполирует движение. GoldNav двигает entity сам (меняет origin) и держит её
 * velocity нулевой.
 *
 * @param entity    Индекс entity.
 * @param goal      Куда идти. Не обязана лежать на полу или быть достижимой:
 *                  entity дойдёт до ближайшего к ней места, куда можно дойти.
 * @param speed     Скорость, юнитов/с.
 * @param hull      GOLDNAV_HULL_*. AUTO — по bbox entity.
 * @param flags     GOLDNAV_MOVE_*.
 * @return          true, если запрос принят (итог придёт в goldnav_move_finished);
 *                  false — entity невалидна или это игрок.
 */
native bool:goldnav_move_to(entity, const Float:goal[3], Float:speed = 200.0, hull = GOLDNAV_HULL_AUTO, flags = 0);

/**
 * Останавливает ведение entity. Безопасно вызывать для любой entity.
 */
native goldnav_stop_following(entity);

/**
 * @return GOLDNAV_STATE_* для entity.
 */
native goldnav_get_state(entity);

/**
 * Готов ли навмеш под данный хулл. Слои POINT и LARGE строятся по первому
 * требованию — этот вызов заодно запускает их генерацию.
 */
native bool:goldnav_is_ready(hull = GOLDNAV_HULL_STANDING);

/**
 * Ищет путь между двумя точками, никого не двигая (например, чтобы нарисовать его
 * или вести бота своими средствами).
 *
 * @param from      Откуда (любая точка; привязывается к полу под ней).
 * @param to        Куда.
 * @param outPath   ПЛОСКИЙ буфер длиной maxLen*3: x0,y0,z0, x1,y1,z1, ...
 *                  Точки — положение "ног" (пол), стартовая точка не включается.
 * @param maxLen    Сколько точек вмещает outPath (не больше 256).
 * @param outLen    Сколько точек записано.
 * @param hull      GOLDNAV_HULL_* (не AUTO).
 * @return          true, если путь найден; false — пути нет или навмеш не готов.
 */
native bool:goldnav_find_path(const Float:from[3], const Float:to[3], Float:outPath[], maxLen, &outLen, hull = GOLDNAV_HULL_STANDING);

/**
 * То же, но до текущего положения entity.
 */
native bool:goldnav_find_path_to_entity(entity, const Float:from[3], Float:outPath[], maxLen, &outLen, hull = GOLDNAV_HULL_STANDING);

/**
 * Ведёт entity по готовому списку точек (обычно из goldnav_find_path). Последняя
 * точка считается целью: собьётся с пути — GoldNav перестроит его к ней.
 *
 * @param path      ПЛОСКИЙ массив длиной pathLen*3.
 */
native goldnav_move_along_path(entity, const Float:path[], pathLen, Float:speed);

/**
 * Вызывается, когда ведение entity закончилось.
 *
 * @param entity    Индекс entity.
 * @param result    GOLDNAV_STATE_ARRIVED или GOLDNAV_STATE_FAILED.
 */
forward goldnav_move_finished(entity, result);
