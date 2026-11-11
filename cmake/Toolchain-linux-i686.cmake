# Форсирует 32-битную сборку под Linux (gcc или clang) — GoldSource является
# 32-битным движком, HLDS не загрузит 64-битный .so.
#
# Требует установленных 32-битных dev-библиотек:
#   Debian/Ubuntu: sudo apt install g++-multilib libc6-dev-i386
#   (для clang -m32 обычно работает "из коробки", если есть multilib-таргет)

set(CMAKE_C_FLAGS_INIT "-m32")
set(CMAKE_CXX_FLAGS_INIT "-m32")
set(CMAKE_EXE_LINKER_FLAGS_INIT "-m32")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "-m32")

set(CMAKE_SYSTEM_PROCESSOR "i686")
