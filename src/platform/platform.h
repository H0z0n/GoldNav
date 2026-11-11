// Платформенная абстракция: экспорт-макросы, фиксированные типы, calling convention.
// GoldSource — 32-битный движок; структуры, пересекающие ABI-границу с движком
// (определённые в HLSDK), не переопределяются здесь — они приходят как есть из
// заголовков HLSDK. Здесь только то, что принадлежит самому GoldNav.
#pragma once

#include <cstdint>
#include <cstddef>

static_assert(sizeof(std::int32_t) == 4, "gn_int32 must be 4 bytes");
static_assert(sizeof(std::uint32_t) == 4, "gn_uint32 must be 4 bytes");
static_assert(sizeof(float) == 4, "gn_float32 must be 4 bytes");

using gn_int32 = std::int32_t;
using gn_uint32 = std::uint32_t;
using gn_int16 = std::int16_t;
using gn_uint16 = std::uint16_t;
using gn_int8 = std::int8_t;
using gn_uint8 = std::uint8_t;
using gn_float32 = float;

// Экспорт публичного C API из goldnav_mm (и из goldnav_core при статической линковке
// в тестовый хост экспорт не нужен, макрос тогда пуст).
#if defined(_WIN32)
    #define GOLDNAV_CALL __cdecl
    #if defined(GOLDNAV_BUILDING_SHARED)
        #define GOLDNAV_API extern "C" __declspec(dllexport)
    #elif defined(GOLDNAV_USING_SHARED)
        #define GOLDNAV_API extern "C" __declspec(dllimport)
    #else
        #define GOLDNAV_API extern "C"
    #endif
#else
    #define GOLDNAV_CALL
    #if defined(GOLDNAV_BUILDING_SHARED)
        #define GOLDNAV_API extern "C" __attribute__((visibility("default")))
    #else
        #define GOLDNAV_API extern "C"
    #endif
#endif

// Единственная точка, где GoldSource ждёт особый calling convention на движковых
// callback-ах (engine funcs / dllapi funcs) — те объявляются напрямую через типы
// HLSDK в src/metamod и src/navmesh/nav_trace_sampler.cpp, этот макрос сюда не лезет.
