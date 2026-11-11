#include "logger.h"

#include <cstdio>
#include <cstdarg>
#include <extdll.h>
#include <meta_api.h>

namespace goldnav {

void LogMessage(LogLevel level, const char* fmt, ...)
{
    char buffer[1024];

    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    const char* prefix = "[GoldNav]";
    switch (level)
    {
        case LogLevel::Warning: prefix = "[GoldNav][WARN]"; break;
        case LogLevel::Error:   prefix = "[GoldNav][ERROR]"; break;
        default: break;
    }

    // LOG_CONSOLE — стандартный Metamod-макрос, пишет и в серверную консоль, и в
    // логи движка; доступен после успешного META_ATTACH (см. meta_api.cpp).
    LOG_CONSOLE(PLID, "%s %s", prefix, buffer);
}

} // namespace goldnav
