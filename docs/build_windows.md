# Сборка на Windows (32-бит)

## Статус

`goldnav_mm.dll` и `goldnav_amxx.dll` реально собраны и слинкованы как валидные
32-битные (`coff-i386`) DLL с корректными экспортами (проверено `llvm-objdump`:
`GiveFnptrsToDll`/`Meta_Query`/`Meta_Attach`/`Meta_Detach`/`GetEntityAPI2(_Post)`/
`GetNewDLLFunctions(_Post)`/`GetEngineFunctions(_Post)` у `goldnav_mm`;
`AMXX_Query`/`AMXX_Attach`/... у `goldnav_amxx`) — тулчейном **llvm-mingw
(i686-w64-mingw32)**, т.к. на машине разработки не было MSVC. Официальный целевой
тулчейн проекта — MSVC (см. ниже); llvm-mingw путь верифицирован как рабочая
альтернатива и зафиксирован как отдельный CMake-пресет.

Проверено на живом HLDS (build 6153, Metamod 1.21-am, AMX Mod X 1.9, ReGameDLL): модуль
загружается, строит навмеш и ведёт entity на стандартных картах. Также проверено: корректность бинарника (архитектура
i386, полный набор Metamod/AMXX-экспортов), минимальные runtime-зависимости
(только `KERNEL32.dll`+Universal CRT после статической линковки — см. нюансы ниже),
и полный прогон engine-agnostic юнит-тестов (`ctest`, 6/6 passed) собранных тем же
тулчейном/пресетом.

## Вариант A — llvm-mingw i686 (проверено, самый быстрый путь для локальной сборки)

1. Поставить [llvm-mingw](https://github.com/mstorsjo/llvm-mingw) (через `winget install
   MartinStorsjo.LLVM-MinGW.UCRT`, либо вручную) и CMake + Ninja, добавить в PATH.
2. Подтянуть SDK (сетевая операция):
   ```powershell
   git submodule add https://github.com/theAsmodai/metamod-r.git third_party/metamod-r
   git submodule add https://github.com/alliedmodders/amxmodx.git third_party/amxmodx
   git submodule update --init --recursive
   ```
3. Собрать:
   ```powershell
   cmake --preset windows-llvm-mingw-i686
   cmake --build --preset windows-llvm-mingw-i686
   ```
4. Результат: `build/windows-llvm-mingw-i686/libgoldnav_mm.dll` и `libgoldnav_amxx.dll`
   (mingw добавляет префикс `lib` — переименовать в `goldnav_mm.dll`/`goldnav_amxx.dll`
   при установке, см. раздел "Установка" ниже).

### Два нюанса, найденные при реальной сборке (уже учтены в CMakeLists.txt)

- **`amxxmodule.h` конфликтует с современным `<cstdint>`**: старый SDK-заголовок AMXX
  сам определяет `int32_t`/`uint32_t` под компиляторы без C99 `<stdint.h>`, проверяя
  `__STDC_VERSION__` — макрос, который в C++ не определён, поэтому проверка ложно
  срабатывает и typedef'ы конфликтуют с уже включённым настоящим `<cstdint>`.
  Фикс: `target_compile_definitions(goldnav_amxx PRIVATE HAVE_STDINT_H)` — штатный
  флаг, документированный в самом amxxmodule.h, не хак.
- **`GiveFnptrsToDll` экспортируется с декорированным именем `GiveFnptrsToDll@8`**:
  функция объявлена с `WINAPI` (`__stdcall`), как того требует HLSDK, а линкер по
  умолчанию декорирует `__stdcall`-экспорты суффиксом `@N` — движок ищет функцию по
  точному недекорированному имени и не найдёт её. Фикс: `.def`-файл
  (`src/metamod/goldnav_mm.def`) с явным `GiveFnptrsToDll @1`, тот же паттерн, что и
  в официальном примере плагина Metamod-R.
- **llvm-mingw по умолчанию динамически линкует `libc++.dll`/`libunwind.dll`** —
  этих DLL нет ни на обычном Windows, ни на стоковом HLDS-сервере: без них
  `goldnav_mm.dll` просто не загрузится ("не обнаружена libc++.dll", воспроизведено
  вживую при первом прогоне тестов). Фикс: `add_link_options(-static)` для всех
  целей при `MINGW` (см. `CMakeLists.txt`) — после этого DLL зависит только от
  `KERNEL32.dll` и `api-ms-win-crt-*.dll` (Universal CRT, есть на любом Windows
  10+/Server 2016+), проверено `llvm-objdump -p`.

## Вариант B — MSVC (не протестирован)

1. Visual Studio 2022 (Community и выше) с компонентом "Desktop development with C++"
   и поддержкой сборки под x86 (Win32).
2. CMake ≥ 3.23.
3. Подтянуть SDK — см. шаг 2 варианта A.
4. Собрать (Developer PowerShell for VS):
   ```powershell
   cmake --preset windows-msvc-x86
   cmake --build --preset windows-msvc-x86 --config RelWithDebInfo
   ```
5. Результат: `build/windows-msvc-x86/**/RelWithDebInfo/goldnav_mm.dll` (точный путь
   зависит от раскладки Visual Studio generator).

Если MSVC выдаст ошибки — начните с двух нюансов из варианта A (`HAVE_STDINT_H`,
`.def`-файл): оба уже применены в CMakeLists.txt для обеих платформ, но MSVC может
проявить другие расхождения (декорирование имён у MSVC отличается от GNU ld в деталях).

## Только engine-agnostic ядро + тесты (без SDK, работает без обоих вариантов выше)

```powershell
cmake --preset tests-only
cmake --build --preset tests-only --config RelWithDebInfo
ctest --preset tests-only
```

(Пресет `tests-only` использует генератор Visual Studio — если MSVC не установлен,
собрать то же самое напрямую через Ninja + любой компилятор, как в варианте A, но с
`-DGOLDNAV_BUILD_ENGINE_MODULE=OFF`.)

## Установка в HLDS

1. Скопировать `goldnav_mm.dll` в `<HLDS>/cstrike/addons/metamod/dlls/`.
2. Добавить строку в `<HLDS>/cstrike/addons/metamod/plugins.ini`:
   ```
   win32 addons/metamod/dlls/goldnav_mm.dll
   ```
3. Если также собираете `goldnav_amxx.dll` — скопировать в
   `<HLDS>/cstrike/addons/amxmodx/modules/`, зарегистрировать в `modules.ini`
   (`goldnav_amxx`) — не обязательно, модуль подгружается плагинами сам.
4. Запустить сервер, проверить в консоли лог `[GoldNav] attached` и далее
   `navmesh generation started` / `navmesh generation complete: ...`.
5. См. `docs/debug_commands.md` (или `goldnav_status` в консоли) для диагностики.
