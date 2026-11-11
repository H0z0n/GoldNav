# Мягкая проверка наличия HLSDK/Metamod-R под third_party/ — не hard error, чтобы
# `cmake --preset ...` всегда конфигурировался хотя бы для goldnav_core/tests,
# даже до того как пользователь подтянул submodules (см. README.md).
#
# HLSDK/Metamod-заголовки берутся из связки, которую сам Metamod-R публикует как
# официальный пример плагина (metamod/extra/example/include/{hlsdk,metamod}) —
# это гарантированно совместимая пара версий (проверено вручную построчно при
# разработке GoldNav), в отличие от независимо версионируемого внешнего HLSDK-репозитория.

set(METAMOD_R_ROOT "${CMAKE_SOURCE_DIR}/third_party/metamod-r")
set(METAMOD_R_EXAMPLE_INCLUDE "${METAMOD_R_ROOT}/metamod/extra/example/include")
set(AMXMODX_ROOT "${CMAKE_SOURCE_DIR}/third_party/amxmodx")

set(HLSDK_FOUND FALSE)

find_path(HLSDK_ENGINE_INCLUDE_DIR
    NAMES eiface.h
    PATHS "${METAMOD_R_EXAMPLE_INCLUDE}/hlsdk/engine"
    NO_DEFAULT_PATH
)

find_path(METAMOD_R_INCLUDE_DIR
    NAMES meta_api.h
    PATHS "${METAMOD_R_EXAMPLE_INCLUDE}/metamod"
    NO_DEFAULT_PATH
)

if(HLSDK_ENGINE_INCLUDE_DIR AND METAMOD_R_INCLUDE_DIR)
    set(HLSDK_FOUND TRUE)

    set(HLSDK_INCLUDE_DIRS
        "${METAMOD_R_EXAMPLE_INCLUDE}/hlsdk/common"
        "${METAMOD_R_EXAMPLE_INCLUDE}/hlsdk/engine"
        "${METAMOD_R_EXAMPLE_INCLUDE}/hlsdk/dlls"
        "${METAMOD_R_EXAMPLE_INCLUDE}/hlsdk/pm_shared"
        "${METAMOD_R_EXAMPLE_INCLUDE}/hlsdk/public"
        "${METAMOD_R_EXAMPLE_INCLUDE}/hlsdk/game_shared"
    )
    set(METAMOD_R_INCLUDE_DIRS
        "${METAMOD_R_INCLUDE_DIR}"
    )
    set(AMXX_SDK_INCLUDE_DIRS
        "${AMXMODX_ROOT}/public"
        "${AMXMODX_ROOT}/public/sdk"
    )
endif()
