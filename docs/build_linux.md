# Сборка на Linux (gcc/clang, 32-бит)

## Статус

Реально собрано, установлено на живой сервер и там же диагностировано — двумя
разными способами, с важным выводом:

**Способ 1 — сборка на свежем Ubuntu 24.04 (WSL2)**: `goldnav_mm.so`/`goldnav_amxx.so`
собрались, прошли все проверки (архитектура i386, экспорты, `ldd`, все 6
юнит-тестов), но при установке на реальный сервер `goldnav_amxx` не грузился
(`failed to load ... goldnav_amxx_i386.so: No such file or directory` — неверное имя
файла, см. ниже) и `goldnav_mm` был помечен Metamod-R как `badf` (bad file). Причина
`badf`: собранный на Ubuntu 24.04 бинарник требует **glibc >= 2.38** (2023 год) даже
после статической линковки libstdc++/libgcc — сама glibc статически не линкуется, а
glibc обратно совместима, но НЕ вперёд совместима. Реальный сервер оказался (типично
для CS 1.6 хостинга) на куда более старом дистрибутиве без такой свежей glibc.

**Способ 2 — сборка в Docker-контейнере Ubuntu 18.04** (`docker/Dockerfile.linux-legacy`,
добавлен по итогам разбора): тот же исходный код, тот же CMake, тот же toolchain-файл
— единственная разница в базовом образе. Результат: `objdump -T goldnav_mm.so | grep
GLIBC` показывает максимум **GLIBC_2.17** (2012 год) вместо `GLIBC_2.38` — совместимо
практически с любым Linux-сервером последних 10+ лет. Все 6 юнит-тестов проходят и в
этой сборке. **Вывод: для реального деплоя используйте Docker-рецепт ниже, а не
голый `cmake --preset linux-gcc-x86` на произвольном (особенно свежем) хосте.**

Второй найденный и исправленный баг — неправильное имя файла `goldnav_amxx`:
AMXX сам достраивает путь модуля из `modules.ini` как `<base>_amxx_i386.so`
(см. `amxmodx/modules.cpp`, функция построения пути) независимо от того, что
написано в `modules.ini` — воспроизведено на живом сервере. Исправлено:
`CMakeLists.txt` теперь называет файл `goldnav_amxx_i386.so` на Linux (уже применено).

## Рекомендуемый способ — Docker (воспроизводимо, максимальная совместимость)

Требует Docker (в т.ч. `docker` в WSL2 на Windows — так это и было проверено).

```bash
git clone <url-этого-репозитория> GoldNav
cd GoldNav

git submodule add https://github.com/theAsmodai/metamod-r.git third_party/metamod-r
git submodule add https://github.com/alliedmodders/amxmodx.git third_party/amxmodx
git submodule update --init --recursive

docker build -f docker/Dockerfile.linux-legacy -t goldnav-build .
docker run --rm -v "$(pwd)":/work goldnav-build bash -c "
    cmake -S /work -B /work/build/linux-legacy -G Ninja \
        -DCMAKE_TOOLCHAIN_FILE=/work/cmake/Toolchain-linux-i686.cmake \
        -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ \
        -DGOLDNAV_BUILD_ENGINE_MODULE=ON -DGOLDNAV_BUILD_AMXX=ON \
    && cmake --build /work/build/linux-legacy \
    && ctest --test-dir /work/build/linux-legacy --output-on-failure
"
```

Результат: `build/linux-legacy/goldnav_mm.so` и `build/linux-legacy/goldnav_amxx_i386.so`.

Проверить требуемую версию glibc перед деплоем (на сборочной машине):
```bash
objdump -T build/linux-legacy/goldnav_mm.so | grep -oP 'GLIBC_[0-9.]+' | sort -V -u | tail -3
```
Сверить с версией на целевом сервере: `ldd --version` (первая строка).

## Альтернатива — прямая сборка без Docker (проще, но менее переносимо)

Годится, если ваш сборочный хост уже достаточно старый/совместимый с целевым
сервером, либо для локальной разработки/тестов, где переносимость не важна.

```bash
# Debian/Ubuntu
sudo apt install g++-multilib gcc-multilib libc6-dev-i386 cmake ninja-build git

cmake --preset linux-gcc-x86
cmake --build --preset linux-gcc-x86
ctest --test-dir build/linux-gcc-x86 --output-on-failure
```

Если `apt install` падает с `404 Not Found` на конкретные версии пакетов glibc
(`libc6-i386`, `libc6-dev-i386` и т.п.) — это рассинхрон зеркала apt, не проблема
проекта. Помогает: `sudo apt update` ещё раз, либо явно поставить `libc6`/`libc6-dev`
вместе с i386-пакетами одной командой (apt требует у них совпадающую версию —
рассинхрон версий между 64-битным и i386 `libc6` чаще всего и есть причина
`Unmet dependencies`).

Результат: `build/linux-gcc-x86/goldnav_mm.so` и `goldnav_amxx_i386.so`
(`PREFIX ""` в `CMakeLists.txt` убирает стандартный `lib`-префикс, `OUTPUT_NAME`
уже учитывает AMXX-конвенцию `_i386`). `GOLDNAV_BUILD_AMXX=ON` по умолчанию;
`goldnav_amxx_i386.so` не зависит от `goldnav_mm.so` при загрузке (ищет его в
рантайме — см. `docs/public_api.md`), поэтому раскладка директорий не важна.

## Только engine-agnostic ядро + тесты (без SDK, работает уже сейчас)

```bash
cmake -S . -B build/tests-only -G Ninja -DGOLDNAV_BUILD_ENGINE_MODULE=OFF
cmake --build build/tests-only
ctest --test-dir build/tests-only --output-on-failure
```

## Установка в HLDS

1. Скопировать `goldnav_mm.so` в `<HLDS>/cstrike/addons/metamod/dlls/`.
2. Добавить строку в `<HLDS>/cstrike/addons/metamod/plugins.ini`:
   ```
   linux addons/metamod/dlls/goldnav_mm.so
   ```
3. Скопировать `goldnav_amxx_i386.so` в `<HLDS>/cstrike/addons/amxmodx/modules/`
   (имя файла важно — см. "Статус" выше), прописать в `modules.ini`:
   ```
   goldnav_amxx
   ```
4. Запустить сервер, проверить `meta list` — `goldnav_mm.so` должен быть `RUN`, не
   `badf`. Если `badf` — почти наверняка проблема glibc-версии (см. "Статус" выше);
   проверьте `ldd goldnav_mm.so` прямо на сервере на предмет `not found`/version
   errors и/или пересоберите через Docker-рецепт.
5. В логе — `[GoldNav] attached`, далее прогресс генерации навмеша.
6. См. `docs/debug_commands.md`.
