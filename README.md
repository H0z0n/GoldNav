<p align="right"><b>English</b> | <a href="README.ru.md">Русский</a></p>

# GoldNav

Waypoint-free navigation for Counter-Strike 1.6 / GoldSource. A native Metamod module plus
AMXX natives: any entity (except players) walks to the given coordinates in a sensible way —
around walls and crates, up stairs and ramps, down ledges, and around players standing in
its way.

The navigation grid is built automatically when a map starts (a fraction of a second to a
couple of seconds). No waypoint files to prepare.

Version: **1.0.0-beta** · Author: **Hozon** · Telegram: [@AmxM0dX](https://t.me/AmxM0dX)

## What's inside

| File | Where to put it | What it is |
|---|---|---|
| `goldnav_mm.dll` / `goldnav_mm.so` | `addons/metamod/dlls/` | the module itself (Metamod plugin) |
| `goldnav_amxx.dll` / `goldnav_amxx_i386.so` | `addons/amxmodx/modules/` | natives for AMXX plugins |
| `goldnav.inc` | `addons/amxmodx/scripting/include/` | header for `.sma` files |
| `goldnav_chicken.amxx` | `addons/amxmodx/plugins/` | test plugin with a chicken |

Prebuilt files are in [`dist/`](dist/) (and attached to the release).

## Installation

1. Copy the files according to the table above.
2. `addons/metamod/plugins.ini`:
   ```
   win32 addons\metamod\dlls\goldnav_mm.dll
   linux addons/metamod/dlls/goldnav_mm.so
   ```
3. `addons/amxmodx/configs/plugins.ini`: `goldnav_chicken.amxx` (needs the fakemeta module).
   You don't have to list `goldnav_amxx` in `modules.ini` — plugins load it automatically.
4. Start the server. The console should print `[GoldNav] navmesh [32x32] ready: N nodes ...`.

## Try it: the chicken plugin

| Command (console / chat) | Action |
|---|---|
| `goldnav_chicken` / `/chicken` | spawn a chicken (`models/chick.mdl`) where you are aiming |
| `goldnav_goto` / `/goto` | send your chickens to the point you are aiming at (the path is drawn as a beam) |
| `goldnav_goto x y z` | ...or to explicit coordinates |
| `goldnav_follow` / `/follow` | chickens follow you |
| `goldnav_stop`, `goldnav_remove`, `goldnav_where` | stop / remove / print aim coordinates |

Plugin cvars: `goldnav_chicken_speed 150`, `goldnav_chicken_path 1`, `goldnav_chicken_admin 0`.

## Using it in your own plugin

```pawn
#include <goldnav>

public send(ent, Float:goal[3])
{
    goldnav_move_to(ent, goal, 200.0);   // the module does everything else
}

public goldnav_move_finished(ent, result)
{
    // result: GOLDNAV_STATE_ARRIVED or GOLDNAV_STATE_FAILED
}
```

Docs (in Russian): [public API](docs/public_api.md) · [architecture](docs/architecture.md) ·
[server commands and cvars](docs/debug_commands.md) · [limitations](docs/roadmap_deferred.md).

## Building

```
git submodule update --init --recursive
```
Windows: [docs/build_windows.md](docs/build_windows.md) · Linux: [docs/build_linux.md](docs/build_linux.md).
The core and its unit tests build without any SDK: `-DGOLDNAV_BUILD_ENGINE_MODULE=OFF`.

## What has been tested

- 9 unit-test suites for the core (generator, pathfinding, walker, path following) on a synthetic world.
- A live Windows HLDS (build 6153, Metamod 1.21-am, AMX Mod X 1.9, ReGameDLL): an entity walks
  spawn ↔ spawn routes on de_dust2, de_nuke, de_inferno, de_aztec, de_train, de_cbble,
  cs_office, cs_italy, cs_assault, cs_militia and stops within ~2 units of the goal.
- The Linux build (`.so`) is compiled from the same sources and passes the same unit tests;
  this version has not been run on a live Linux server yet.
- Not verified visually in the game client: movement smoothness and animation.

## Limitations (short)

Players and bots are not moved by the module; no ladders, no upward jumps; doors are taken
in the state they had at generation time (`goldnav_rebuild` after changes). Full list:
[docs/roadmap_deferred.md](docs/roadmap_deferred.md).
