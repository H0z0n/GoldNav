// Кроссплатформенный высокоточный таймер для бюджетирования генерации навмеша.
#pragma once

namespace goldnav {

// Монотонные миллисекунды с произвольной точки отсчёта — годится только для
// измерения дельт (elapsed = Now() - startedAt), не для wall-clock времени.
double PlatformNowMs();

} // namespace goldnav
