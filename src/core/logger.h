// Тонкая обёртка логирования: в movмод-сборке пишет через движковый ALERT/LOG_CONSOLE
// (см. logger.cpp), при отсутствии движка (не должно случаться в goldnav_mm, но
// защищаемся) — падает на stderr.
#pragma once

namespace goldnav {

enum class LogLevel
{
    Info,
    Warning,
    Error,
};

void LogMessage(LogLevel level, const char* fmt, ...);

#define GOLDNAV_LOG_INFO(...)  ::goldnav::LogMessage(::goldnav::LogLevel::Info, __VA_ARGS__)
#define GOLDNAV_LOG_WARN(...)  ::goldnav::LogMessage(::goldnav::LogLevel::Warning, __VA_ARGS__)
#define GOLDNAV_LOG_ERROR(...) ::goldnav::LogMessage(::goldnav::LogLevel::Error, __VA_ARGS__)

} // namespace goldnav
