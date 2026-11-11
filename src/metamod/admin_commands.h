// Серверные консольные команды для диагностики: goldnav_rebuild, goldnav_status,
// goldnav_findpath. Только серверная консоль (REG_SVR_COMMAND) — GoldNav не
// принимает эти команды от клиентов в MVP.
#pragma once

namespace goldnav {

void RegisterAdminCommands();

} // namespace goldnav
