# Публичный API GoldNav

## AMXX (`#include <goldnav>`)

Заголовок — `pawn/include/goldnav.inc`, пример — `pawn/examples/goldnav_chicken.sma`.

```pawn
native bool:goldnav_move_to(entity, const Float:goal[3], Float:speed = 200.0, hull = GOLDNAV_HULL_AUTO, flags = 0);
native goldnav_stop_following(entity);
native goldnav_get_state(entity);                       // GOLDNAV_STATE_*
native goldnav_set_jump_height(entity, Float:height);   // как высоко запрыгивает (0 — не прыгает)
native goldnav_set_max_drop(entity, Float:height);      // с какой высоты спрыгивает (< 0 — общий предел)
native bool:goldnav_is_ready(hull = GOLDNAV_HULL_STANDING);

native bool:goldnav_find_path(const Float:from[3], const Float:to[3], Float:outPath[], maxLen, &outLen, hull = GOLDNAV_HULL_STANDING);
native bool:goldnav_find_path_to_entity(entity, const Float:from[3], Float:outPath[], maxLen, &outLen, hull = GOLDNAV_HULL_STANDING);
native goldnav_move_along_path(entity, const Float:path[], pathLen, Float:speed);

forward goldnav_move_finished(entity, result);          // ARRIVED или FAILED
```

`goldnav_move_to` — основной натив: находит путь, ведёт entity каждый кадр, перестраивает
путь при сбоях и останавливает её в цели. Если навмеш ещё строится, запрос ждёт сам
(состояние `PENDING`). Цель не обязана быть достижимой: entity подойдёт как можно ближе
(с флагом `GOLDNAV_MOVE_EXACT` — вернёт `FAILED`).

**Прыжки и спрыгивание.** По умолчанию entity берёт только ступеньки (`sv_stepsize`, 18) и
спрыгивает с уступов до `goldnav_max_drop` (150). `goldnav_set_jump_height` разрешает ей
запрыгивать на уступы и ящики до заданной высоты (прыжок игрока CS — 45, с приседом — 63;
рёбра-прыжки строятся до `goldnav_max_jump`, 64). Значения запоминаются за entity до её удаления
и действуют со следующего `goldnav_move_to`. `goldnav_find_path` прыжки не использует — он
ищет путь "пешком".

**Динамические объекты.** Двери, платформы, ломаемые стёкла и стоящие entity с `SOLID_BBOX`
учитываются автоматически: объект сменил положение и остановился (дверь — 0.3 с, коробка — 1 с) —
сетка вокруг перестраивается, идущие entity перепланируют путь. Игроки, монстры (`FL_MONSTER`)
и entity, которых GoldNav сейчас ведёт, препятствиями в сетке не считаются — их обходит на месте
сам "ходок". Entity, которую GoldNav довёл и оставил стоять, через секунду становится
препятствием для остальных.

**Состояния:** `IDLE` 0, `PENDING` 1, `MOVING` 2, `ARRIVED` 3, `FAILED` 4.

**Флаги:** `GOLDNAV_MOVE_NO_TURN` (не поворачивать entity), `NO_DROP` (не спрыгивать с
уступов), `NO_TOUCH` (не вызывать Touch у задетых entity), `EXACT`.

**Пути** (`outPath`/`path`) — плоские массивы `x0,y0,z0, x1,y1,z1, ...` длиной `maxLen*3`;
точки — положение «ног» (пол), стартовая не включается; максимум 256 точек.

### Хуллы и размер entity

Движок сталкивает entity с картой одним из четырёх размеров, выбирая по её bbox:

| ширина bbox | высота | хулл | константа |
|---|---|---|---|
| ≤ 8 | любая | точка в origin | `GOLDNAV_HULL_POINT` |
| ≤ 36 | ≤ 36 | 32x32x36 | `GOLDNAV_HULL_DUCKING` |
| ≤ 36 | > 36 | 32x32x72 | `GOLDNAV_HULL_STANDING` |
| > 36 | любая | 64x64x64 | `GOLDNAV_HULL_LARGE` |

Задавайте bbox ровно одного из этих размеров (например, `{-16,-16,0}` – `{16,16,36}`),
иначе движок подставит ближайший хулл со смещением относительно origin (GoldNav это
смещение учитывает, но визуально entity будет идти не по центру пути). `HULL_AUTO` берёт
хулл по bbox entity. Слои point и large строятся при первом обращении.

### Требования к entity

- Любая entity, кроме игроков/ботов.
- Рекомендуемый movetype — `MOVETYPE_PUSHSTEP` (клиент плавно интерполирует движение).
- Пока GoldNav ведёт entity, он сам меняет её `origin`, держит `velocity` нулевой и
  `FL_ONGROUND` взведённым; поворачивает `angles[1]` по ходу движения.
- Анимацию (sequence/framerate) выставляет ваш плагин.

## C API (для Metamod-плагинов)

`include/goldnav/goldnav_api.h` — те же функции в виде `extern "C"`:
`GoldNav_MoveEntityTo`, `GoldNav_StopFollowing`, `GoldNav_GetEntityState`,
`GoldNav_SetEntityJumpHeight`, `GoldNav_SetEntityMaxDrop`,
`GoldNav_FindPath`, `GoldNav_FindPathToEntity`, `GoldNav_MoveEntityAlongPath`,
`GoldNav_IsNavMeshReady`, `GoldNav_IsHullReady`, `GoldNav_SetMoveCallback`,
`GoldNav_GetVersion`, `GoldNav_EdictByIndex`. Только сырые указатели и длины — никаких
STL-типов через границу DLL.

## Как goldnav_amxx находит goldnav_mm

`goldnav_amxx` не линкуется на `goldnav_mm`: при первом вызове натива он находит уже
загруженный Metamod-ом модуль (`GetModuleHandle` / `dlopen(RTLD_NOLOAD)`) и берёт функции
по именам. Поэтому файлы можно класть в стандартные папки, порядок загрузки не важен.
Если `goldnav_mm` не загружен или версии не совпадают: `goldnav_is_ready()` вернёт 0,
остальные натусы запишут ошибку в лог AMXX. Обновляйте оба файла вместе.
