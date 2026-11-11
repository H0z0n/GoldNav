<p align="right"><a href="README.md">English</a> | <b>Русский</b></p>

# GoldNav

Навигация без waypoint-ов для Counter-Strike 1.6 / GoldSource. Нативный модуль Metamod +
AMXX-натусы: любая entity (кроме игроков) идёт в указанные координаты осмысленно — обходит
стены и ящики, поднимается по лестницам и пандусам, спрыгивает с уступов, огибает стоящих
на пути игроков.

Навигационная сетка строится сама при старте карты (доли секунды – пара секунд), никаких
файлов waypoint-ов готовить не нужно.

Версия: **1.0.0-beta** · Автор: **Hozon** · Telegram: [@AmxM0dX](https://t.me/AmxM0dX)

## Что внутри

| Файл | Куда класть | Что это |
|---|---|---|
| `goldnav_mm.dll` / `goldnav_mm.so` | `addons/metamod/dlls/` | сам модуль (Metamod-плагин) |
| `goldnav_amxx.dll` / `goldnav_amxx_i386.so` | `addons/amxmodx/modules/` | натусы для AMXX-плагинов |
| `goldnav.inc` | `addons/amxmodx/scripting/include/` | заголовок для `.sma` |
| `goldnav_chicken.amxx` | `addons/amxmodx/plugins/` | тестовый плагин с курицей |

Готовые файлы лежат в [`dist/`](dist/) (и приложены к релизу).

## Установка

1. Скопируйте файлы по таблице выше.
2. `addons/metamod/plugins.ini`:
   ```
   win32 addons\metamod\dlls\goldnav_mm.dll
   linux addons/metamod/dlls/goldnav_mm.so
   ```
3. `addons/amxmodx/configs/plugins.ini`: `goldnav_chicken.amxx` (нужен модуль fakemeta).
   Модуль `goldnav_amxx` в `modules.ini` прописывать не обязательно — подгрузится сам.
4. Запустите сервер. В консоли: `[GoldNav] navmesh [32x32] ready: N nodes ...`.

## Проверка: плагин с курицей

| Команда (консоль / чат) | Действие |
|---|---|
| `goldnav_chicken` / `/chicken` | заспавнить курицу (`models/chick.mdl`) там, куда смотрите |
| `goldnav_goto` / `/goto` | отправить своих кур в точку прицела (путь рисуется лучом) |
| `goldnav_goto x y z` | ...или в явные координаты |
| `goldnav_follow` / `/follow` | куры идут за вами |
| `goldnav_stop`, `goldnav_remove`, `goldnav_where` | остановить / удалить / координаты прицела |

Квары плагина: `goldnav_chicken_speed 150`, `goldnav_chicken_path 1`, `goldnav_chicken_admin 0`.

## Использование в своём плагине

```pawn
#include <goldnav>

public send(ent, Float:goal[3])
{
    goldnav_move_to(ent, goal, 200.0);   // дальше модуль всё делает сам
}

public goldnav_move_finished(ent, result)
{
    // result: GOLDNAV_STATE_ARRIVED или GOLDNAV_STATE_FAILED
}
```

Документация: [публичный API](docs/public_api.md) · [архитектура](docs/architecture.md) ·
[команды и квары сервера](docs/debug_commands.md) · [ограничения](docs/roadmap_deferred.md).

## Сборка

```
git submodule update --init --recursive
```
Windows: [docs/build_windows.md](docs/build_windows.md) · Linux: [docs/build_linux.md](docs/build_linux.md).
Ядро и юнит-тесты собираются без SDK: `-DGOLDNAV_BUILD_ENGINE_MODULE=OFF`.

## Что проверено

- 9 наборов юнит-тестов ядра (генератор, поиск пути, ходок, следование) на синтетическом мире.
- Живой HLDS под Windows (build 6153, Metamod 1.21-am, AMX Mod X 1.9, ReGameDLL): entity
  проходит маршруты «спавн ↔ спавн» на de_dust2, de_nuke, de_inferno, de_aztec, de_train,
  de_cbble, cs_office, cs_italy, cs_assault, cs_militia и останавливается в ~2 юнитах от цели.
- Linux-сборка (`.so`) собирается из тех же исходников и проходит те же юнит-тесты; на
  живом Linux-сервере эта версия ещё не запускалась.
- Не проверено глазами в клиенте: плавность движения и анимация.

## Ограничения (кратко)

Игроков и ботов модуль не двигает; нет лестниц и прыжков вверх; двери учитываются в том
состоянии, в каком были при генерации (после изменений — `goldnav_rebuild`). Полный
список: [docs/roadmap_deferred.md](docs/roadmap_deferred.md).
