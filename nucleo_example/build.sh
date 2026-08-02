#!/usr/bin/env bash
# 빌드 / 플래시 스크립트 (Git Bash 에서 실행)
#
#   ./build.sh          빌드만
#   ./build.sh flash    빌드 후 보드에 write + 리셋
#
# 툴체인은 STM32CubeIDE 에 번들된 것을 그대로 쓴다 (별도 설치 불필요).

set -e

CUBEIDE="/c/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins"
GCC_BIN="$CUBEIDE/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740/tools/bin"
PROG_BIN="$CUBEIDE/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304/tools/bin"

export PATH="$GCC_BIN:$PATH"

PROJ_DIR="$(cd "$(dirname "$0")" && pwd)"
ELF="$PROJ_DIR/build/Debug/nucleo_example.elf"

cd "$PROJ_DIR"

# build/Debug 가 없으면 먼저 configure
if [ ! -f build/Debug/build.ninja ]; then
    cmake --preset Debug
fi

cmake --build --preset Debug

echo ""
arm-none-eabi-size "$ELF"

if [ "$1" = "flash" ]; then
    echo ""
    echo "=== 보드에 write ==="
    "$PROG_BIN/STM32_Programmer_CLI.exe" -c port=SWD -w "$ELF" -rst
fi
