cmake_minimum_required(VERSION 3.16)
project(GoldNav CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# goldnav_core — статическая библиотека, линкуемая в goldnav_mm (SHARED). Без PIC
# компилятор по умолчанию (Linux/gcc) генерирует не позиционно-независимый код для
# статических либ, из-за чего линковка в .so требует текстовых релокаций
# ("creating DT_TEXTREL in a shared object" — воспроизведено и подтверждено реальной
# сборкой gcc -m32 под WSL). PIC для статической либы, линкуемой только в SHARED,
# не имеет накладных расходов на Windows/MSVC (там PIC не применимо) — безопасно
# включить глобально.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

option(GOLDNAV_BUILD_TESTS "Build engine-agnostic unit tests (no SDK required)" ON)
option(GOLDNAV_BUILD_ENGINE_MODULE "Build the Metamod shared module (requires HLSDK + Metamod-R)" ON)
option(GOLDNAV_BUILD_AMXX "Build the AMXX natives module" ON)

# GoldSource — 32-битный движок. На Linux форсируем -m32 через toolchain-файл
# (см. cmake/Toolchain-linux-i686.cmake), на Windows — через generator platform Win32
# (см. CMakePresets.json). Здесь только предупреждаем, если платформа явно 64-битная
# и не задан toolchain-файл, чтобы не собрать несовместимый с HLDS бинарник молча.
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(WARNING
        "GoldNav targets GoldSource, a 32-bit engine. Building 64-bit here will "
        "produce a module HLDS cannot load. Use the linux-gcc-x86/linux-clang-x86 "
        "or windows-msvc-x86 CMake presets.")
endif()

if(MINGW)
    # MinGW-тулчейны (в т.ч. llvm-mingw) по умолчанию динамически линкуют рантайм
    # (libstdc++/libgcc, или libc++/libunwind у llvm-mingw) — этих DLL нет ни на
    # обычном Windows, ни на стоковом HLDS-сервере, и модуль/тест просто не
    # загрузится ("не обнаружена libc++.dll"). Статическая линковка убирает эту
    # внешнюю зависимость полностью для ВСЕХ целей ниже (goldnav_mm, goldnav_amxx,
    # тестовые .exe) — подтверждено реальной сборкой.
    add_link_options(-static)
elseif(UNIX AND (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
    # Сборочное окружение (свежий Ubuntu/др. дистрибутив) обычно новее, чем реальный
    # HLDS-сервер (часто старый/минимальный Linux) — если собрать с динамической
    # линковкой libstdc++/libgcc_s, .so требует версионные символы GLIBCXX_*/
    # GCC_*, которых на старом сервере может не быть: dlopen() падает, Metamod
    # помечает плагин как "badf" (bad file) без явной причины в логе. Статическая
    # линковка этих двух либ убирает эту версионную зависимость полностью — libc.so.6
    # (сам glibc) остаётся динамическим, т.к. его статическая линковка в модуль,
    # загружаемый dlopen()-ом в чужой процесс, небезопасна и не нужна (glibc
    # обратно совместим в отличие от libstdc++/libgcc).
    add_link_options(-static-libstdc++ -static-libgcc)
endif()

# --- goldnav_core: engine-agnostic ядро (pathfinder/navmesh-граф/pathfollower-математика).
# Не должен ни при каких опциях получить зависимость на HLSDK/Metamod-R/AMXX заголовки —
# это единственное, что позволяет тестировать его без SDK submodules.
add_library(goldnav_core STATIC
    src/platform/platform_time.cpp
    src/navmesh/nav_hull_defs.cpp
    src/navmesh/nav_grid.cpp
    src/navmesh/nav_probe.cpp
    src/navmesh/obstacle_set.cpp
    src/navmesh/nav_generator.cpp
    src/navmesh/nav_path_planner.cpp
    src/pathfinder/astar.cpp
    src/pathfinder/path_smoother.cpp
    src/pathfollower/ground_mover.cpp
    src/pathfollower/path_follower.cpp
)
target_include_directories(goldnav_core PUBLIC src)
target_compile_features(goldnav_core PUBLIC cxx_std_17)

if(GOLDNAV_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# --- goldnav_mm: полный серверный модуль (Metamod plugin). Требует HLSDK + Metamod-R
# заголовки под third_party/. Если их нет (submodules не инициализированы),
# конфигурация не падает — просто пропускает этот таргет с предупреждением, чтобы
# goldnav_core + tests оставались собираемыми уже сейчас.
if(GOLDNAV_BUILD_ENGINE_MODULE)
    include(cmake/FindHLSDK.cmake)

    if(HLSDK_FOUND)
        add_library(goldnav_mm SHARED
            src/metamod/engine_collision_world.cpp
            src/metamod/obstacle_tracker.cpp
            src/core/goldnav_module.cpp
            src/core/logger.cpp
            src/core/config.cpp
            src/metamod/h_export.cpp
            src/metamod/meta_api.cpp
            src/metamod/dllapi.cpp
            src/metamod/engine_api.cpp
            src/metamod/hooks.cpp
            src/metamod/debug_draw.cpp
            src/metamod/admin_commands.cpp
            src/metamod/entity_follow_bridge.cpp
            # .def нужен ТОЛЬКО из-за GiveFnptrsToDll: оно объявлено с WINAPI
            # (=__stdcall) как того требует HLSDK/Metamod-R (см. h_export.cpp), и
            # __stdcall-экспорты по умолчанию декорируются линкером в
            # "GiveFnptrsToDll@8" — движок ищет функцию по точному недекорированному
            # имени и не найдёт её без этого .def (воспроизведено и подтверждено
            # реальной сборкой; тот же .def-паттерн — в официальном примере плагина
            # Metamod-R, third_party/metamod-r/.../msvc/example_plugin.def). Остальные
            # экспорты — обычный cdecl (C_DLLEXPORT без WINAPI), декорации не получают.
            $<$<PLATFORM_ID:Windows>:${CMAKE_SOURCE_DIR}/src/metamod/goldnav_mm.def>
        )
        target_compile_definitions(goldnav_mm PRIVATE GOLDNAV_BUILDING_SHARED)
        target_link_libraries(goldnav_mm PRIVATE goldnav_core)
        target_include_directories(goldnav_mm PRIVATE
            src
            ${HLSDK_INCLUDE_DIRS}
            ${METAMOD_R_INCLUDE_DIRS}
        )

        if(WIN32)
            set_target_properties(goldnav_mm PROPERTIES OUTPUT_NAME "goldnav_mm")
        else()
            # Без "lib"-префикса — Metamod-R грузит модуль по точно заданному в
            # plugins.ini имени файла.
            set_target_properties(goldnav_mm PROPERTIES PREFIX "" OUTPUT_NAME "goldnav_mm")
        endif()

        if(GOLDNAV_BUILD_AMXX)
            add_library(goldnav_amxx SHARED
                src/amxx/amxx_module.cpp
                src/amxx/natives_navmesh.cpp
                # amxxmodule.cpp — SDK-предоставленная реализация (не наш код): даёт
                # реальные DLL-экспорты AMXX-модуля, читает MODULE_* макросы из
                # src/amxx/moduleconfig.h и вызывает FN_AMXX_ATTACH/_DETACH.
                ${AMXMODX_ROOT}/public/sdk/amxxmodule.cpp
            )
            # goldnav_amxx НЕ линкуется на goldnav_mm: AMXX загружает модули раньше, чем
            # Metamod подключает goldnav_mm, и путь к goldnav_mm.so на сервере заранее
            # неизвестен — DT_NEEDED-зависимость приводила к "goldnav_mm.so: cannot open
            # shared object file" при dlopen модуля (воспроизведено на живом сервере).
            # Функции GoldNav_* модуль находит в уже загруженном goldnav_mm при первом
            # вызове натива (см. src/amxx/natives_navmesh.cpp, BindApi); dlopen/dlsym на
            # Linux (glibc < 2.34 держит их в libdl) - отсюда CMAKE_DL_LIBS.
            target_link_libraries(goldnav_amxx PRIVATE ${CMAKE_DL_LIBS})
            # amxxmodule.h (SDK-код, не наш) под старые компиляторы без C99 <stdint.h>
            # определяет свои int32_t/uint32_t typedef'ы, что конфликтует с настоящим
            # <cstdint>, который уже подключён современным libc++/glibc++ до этого
            # места. HAVE_STDINT_H — штатный, документированный в самом заголовке флаг
            # ("The next section is copied from amx.h"), говорящий использовать
            # реальный <stdint.h> вместо собственных typedef'ов; не хак, а
            # предусмотренная точка конфигурации.
            target_compile_definitions(goldnav_amxx PRIVATE HAVE_STDINT_H)
            target_include_directories(goldnav_amxx PRIVATE
                src
                src/amxx
                ${HLSDK_INCLUDE_DIRS}
                ${AMXX_SDK_INCLUDE_DIRS}
            )
            if(NOT WIN32)
                # OUTPUT_NAME "goldnav_amxx_i386" — ОБЯЗАТЕЛЬНОЕ требование самого
                # AMXX-загрузчика, не наш выбор: amxmodx/modules.cpp (build_pathname)
                # для 32-бит Linux всегда достраивает путь как "<base>_amxx_i%c86.so"
                # (i386), независимо от того, что написано в modules.ini — реально
                # воспроизведено на живом сервере ("failed to load
                # .../goldnav_amxx_i386.so: No such file or directory", наш билд
                # назывался просто goldnav_amxx.so).
                set_target_properties(goldnav_amxx PROPERTIES
                    PREFIX ""
                    OUTPUT_NAME "goldnav_amxx_i386"
                )
            endif()
        endif()
    else()
        message(WARNING
            "GOLDNAV_BUILD_ENGINE_MODULE=ON but HLSDK/Metamod-R headers were not found "
            "under third_party/metamod-r/metamod/extra/example/include/. Run:\n"
            "  git submodule add https://github.com/theAsmodai/metamod-r.git third_party/metamod-r\n"
            "  git submodule add https://github.com/alliedmodders/amxmodx.git third_party/amxmodx\n"
            "  git submodule update --init --recursive\n"
            "Skipping goldnav_mm/goldnav_amxx targets for now; goldnav_core and tests "
            "still build normally.")
    endif()
endif()
