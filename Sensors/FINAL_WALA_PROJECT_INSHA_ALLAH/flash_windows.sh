#!/usr/bin/env bash
###############################################################################
# Universal STM32CubeMX CMake Build & Flash Script  v2.0  [Windows/Antigravity IDE]
#
# Usage:  ./azyan_flash.sh [options] [project_dir]
#
# Options:
#   --clean          Force clean rebuild (default: incremental)
#   --build-only     Build but do not flash
#   --flash-only     Flash last build without rebuilding
#   --debug          Build with Debug configuration (default: Release)
#   --release        Build with Release configuration (explicit)
#   --verify         Read back flash after programming to verify
#   --openocd        Use OpenOCD instead of STM32_Programmer_CLI
#   --programmer     Use STM32CubeProgrammer CLI (default on this machine)
#   --serial         Flash over UART (uses stm32flash)
#   --help           Show this help
#
# Auto-detects: project name, MCU, CPU/FPU flags, linker script, flash size,
#               flash start address, programmer tool.
# Enables: floating-point printf & scanf, hardware FPU.
# Works with any STM32CubeMX project containing a .ioc + CMakeLists.txt.
#
# Windows laptop paths (Antigravity IDE / STM32 VS Code Extension):
#   cube.exe       : ~/.antigravity-ide/extensions/stmicroelectronics.stm32cube-ide-core-1.3.0-win32-x64/resources/binaries/win32/x86_64/
#   cube-cmake.exe : ~/.antigravity-ide/extensions/stmicroelectronics.stm32cube-ide-build-cmake-1.45.0-win32-x64/resources/cube-cmake/win32/x86_64/
#   STM32_Programmer_CLI.exe : ~/AppData/Local/stm32cube/bundles/programmer/2.22.0+st.1/bin/
###############################################################################
set -euo pipefail

# ── Colors & logging ────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
step()  { echo -e "${CYAN}[STEP]${NC}  ${BOLD}$*${NC}"; }
die()   { echo -e "${RED}[ERROR]${NC} $*" >&2; exit 1; }

# ── Timing helper ───────────────────────────────────────────────────────────
timer_start() { SECONDS=0; }
timer_elapsed() { echo "${SECONDS}s"; }

# ── Defaults ────────────────────────────────────────────────────────────────
DO_CLEAN=false
BUILD_ONLY=false
FLASH_ONLY=false
DO_VERIFY=false
BUILD_TYPE="Release"
FLASH_TOOL="auto"   # auto | programmer | openocd | serial
PROJECT_DIR=""

# ── Windows/Git-Bash HOME normalisation ─────────────────────────────────────
# On Windows with Git Bash, HOME may be /c/Users/Student — normalise it
WINHOME="${USERPROFILE:-$HOME}"
# Convert Windows path to POSIX for bash if needed
if [[ "$WINHOME" =~ ^[A-Za-z]:\\ ]]; then
    WINHOME_POSIX=$(cygpath -u "$WINHOME" 2>/dev/null || echo "$WINHOME" | sed 's|\\|/|g; s|^\([A-Za-z]\):|/\L\1|')
else
    WINHOME_POSIX="$WINHOME"
fi

# ── Parse arguments ─────────────────────────────────────────────────────────
show_help() {
    sed -n '2,/^###*$/p' "$0" | grep '^#' | sed 's/^# \?//'
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --clean)          DO_CLEAN=true ;;
        --build-only)      BUILD_ONLY=true ;;
        --flash-only)      FLASH_ONLY=true ;;
        --debug)           BUILD_TYPE="Debug" ;;
        --release)         BUILD_TYPE="Release" ;;
        --relwithdebinfo)  BUILD_TYPE="RelWithDebInfo" ;;
        --minsizerel)      BUILD_TYPE="MinSizeRel" ;;
        --fast)            BUILD_TYPE="Fast" ;;
        --verify)          DO_VERIFY=true ;;
        --openocd)         FLASH_TOOL="openocd" ;;
        --programmer)      FLASH_TOOL="programmer" ;;
        --serial)          FLASH_TOOL="serial" ;;
        --help|-h)         show_help ;;
        -*)                die "Unknown option: $1  (try --help)" ;;
        *)                 PROJECT_DIR="$1" ;;
    esac
    shift
done

# ── Resolve project directory ────────────────────────────────────────────────
PROJECT_DIR="${PROJECT_DIR:-.}"
PROJECT_DIR="$(cd "$PROJECT_DIR" && pwd)"
cd "$PROJECT_DIR"

echo ""
echo -e "${BOLD}══════════════════════════════════════════════════════════════${NC}"
echo -e "${BOLD}  STM32 Build & Flash  │  $(date '+%Y-%m-%d %H:%M:%S')${NC}"
echo -e "${BOLD}══════════════════════════════════════════════════════════════${NC}"
echo ""
info "Project directory: $PROJECT_DIR"

# ── Add Windows STM32 tool paths ────────────────────────────────────────────
step "Setting up Windows toolchain paths..."

# cube.exe wrapper (arm-none-eabi compiler via Antigravity IDE / STM32 VS Code Extension)
CUBE_CORE_BIN="${WINHOME_POSIX}/.antigravity-ide/extensions/stmicroelectronics.stm32cube-ide-core-1.3.0-win32-x64/resources/binaries/win32/x86_64"
if [[ -d "$CUBE_CORE_BIN" ]]; then
    export PATH="${CUBE_CORE_BIN}:${PATH}"
    info "Added cube.exe to PATH: $CUBE_CORE_BIN"
fi

# cube-cmake.exe
CUBE_CMAKE_BIN="${WINHOME_POSIX}/.antigravity-ide/extensions/stmicroelectronics.stm32cube-ide-build-cmake-1.45.0-win32-x64/resources/cube-cmake/win32/x86_64"
if [[ -d "$CUBE_CMAKE_BIN" ]]; then
    export PATH="${CUBE_CMAKE_BIN}:${PATH}"
    info "Added cube-cmake.exe to PATH: $CUBE_CMAKE_BIN"
fi

# STM32_Programmer_CLI.exe
PROGRAMMER_BIN="${WINHOME_POSIX}/AppData/Local/stm32cube/bundles/programmer/2.22.0+st.1/bin"
if [[ -d "$PROGRAMMER_BIN" ]]; then
    export PATH="${PROGRAMMER_BIN}:${PATH}"
    info "Added STM32_Programmer_CLI.exe to PATH: $PROGRAMMER_BIN"
fi

# Also export the bundle path so cube.exe can find its internal tools
export CUBE_BUNDLE_PATH="${WINHOME_POSIX}/AppData/Local/stm32cube/bundles"

# ── Verify required tools ───────────────────────────────────────────────────
step "Checking toolchain..."
MISSING_TOOLS=()

# Check for cube (STM32CubeIDE wrapper — provides arm-none-eabi tools on Windows)
if command -v cube &>/dev/null || command -v cube.exe &>/dev/null; then
    GCC_VERSION=$(cube starm-gcc --version 2>/dev/null | head -1 || echo "STM32 cube wrapper found")
    info "Compiler: $GCC_VERSION"
else
    die "cube.exe not found. Expected at: $CUBE_CORE_BIN"
fi

# Helper: run arm-none-eabi tools via cube wrapper on Windows
arm_tool() {
    local tool="$1"; shift
    if command -v "arm-none-eabi-${tool}" &>/dev/null; then
        "arm-none-eabi-${tool}" "$@"
    else
        cube "starm-${tool}" "$@"
    fi
}

# Check for cube-cmake
if ! command -v cube-cmake &>/dev/null && ! command -v cube-cmake.exe &>/dev/null; then
    MISSING_TOOLS+=("cube-cmake")
fi

if (( ${#MISSING_TOOLS[@]} > 0 )); then
    die "Missing tools: ${MISSING_TOOLS[*]}"
fi

# ── Auto-detect flash tool ───────────────────────────────────────────────────
if [[ "$FLASH_TOOL" == "auto" ]]; then
    if command -v STM32_Programmer_CLI &>/dev/null || command -v STM32_Programmer_CLI.exe &>/dev/null; then
        FLASH_TOOL="programmer"
    elif command -v openocd &>/dev/null; then
        FLASH_TOOL="openocd"
    elif command -v stm32flash &>/dev/null; then
        FLASH_TOOL="serial"
    else
        if [[ "$BUILD_ONLY" == false && "$FLASH_ONLY" == false ]]; then
            warn "No flash tool found. Will build only."
            warn "STM32_Programmer_CLI expected at: $PROGRAMMER_BIN"
            BUILD_ONLY=true
        fi
    fi
fi
if [[ "$BUILD_ONLY" == false ]]; then
    info "Flash tool: $FLASH_TOOL"
fi

# ── Find the .ioc file ──────────────────────────────────────────────────────
step "Detecting project..."
IOC_FILES=()
while IFS= read -r -d '' f; do
    IOC_FILES+=("$f")
done < <(find "$PROJECT_DIR" -maxdepth 1 -name '*.ioc' -type f -print0)
if (( ${#IOC_FILES[@]} == 0 )); then
    die "No .ioc file found in $PROJECT_DIR"
elif (( ${#IOC_FILES[@]} > 1 )); then
    warn "Multiple .ioc files found, using first: $(basename "${IOC_FILES[0]}")"
fi
IOC_FILE="${IOC_FILES[0]}"
info "IOC file: $(basename "$IOC_FILE")"

# ── Extract project name from CMakeLists.txt or .ioc filename ────────────────
PROJECT_NAME=""
if [[ -f CMakeLists.txt ]]; then
    PROJECT_NAME=$(grep -oP 'set\s*\(\s*CMAKE_PROJECT_NAME\s+\K[^\s)]+' CMakeLists.txt 2>/dev/null || true)
fi
if [[ -z "$PROJECT_NAME" ]]; then
    PROJECT_NAME="$(basename "$IOC_FILE" .ioc)"
fi
info "Project name: $PROJECT_NAME"

# ── Extract MCU info from .ioc ──────────────────────────────────────────────
MCU_FAMILY=$(grep -oP '^Mcu\.Family=\K.*' "$IOC_FILE" | tr '[:upper:]' '[:lower:]' || true)
MCU_CPN=$(grep -oP '^Mcu\.CPN=\K.*' "$IOC_FILE" | tr '[:upper:]' '[:lower:]' || true)
MCU_PACKAGE=$(grep -oP '^Mcu\.Package=\K.*' "$IOC_FILE" || true)

[[ -n "$MCU_CPN" ]] || die "Cannot determine MCU part number from .ioc"
info "MCU: ${MCU_CPN^^}  (family: ${MCU_FAMILY^^}, package: $MCU_PACKAGE)"

# ── Determine CPU core, FPU flags, and flash size ───────────────────────────
CPU_FLAGS=""
OPENOCD_TARGET=""
PROGRAMMER_MCU=""
case "$MCU_CPN" in
    stm32f0*|stm32l0*|stm32g0*|stm32c0*)
        CPU_FLAGS="-mcpu=cortex-m0plus -mthumb"
        OPENOCD_TARGET="stm32f0x"
        PROGRAMMER_MCU="${MCU_CPN^^}" ;;
    stm32f1*)
        CPU_FLAGS="-mcpu=cortex-m3 -mthumb"
        OPENOCD_TARGET="stm32f1x"
        PROGRAMMER_MCU="${MCU_CPN^^}" ;;
    stm32f2*)
        CPU_FLAGS="-mcpu=cortex-m3 -mthumb"
        OPENOCD_TARGET="stm32f2x"
        PROGRAMMER_MCU="${MCU_CPN^^}" ;;
    stm32l1*)
        CPU_FLAGS="-mcpu=cortex-m3 -mthumb"
        OPENOCD_TARGET="stm32l1"
        PROGRAMMER_MCU="${MCU_CPN^^}" ;;
    stm32f3*)
        CPU_FLAGS="-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard"
        OPENOCD_TARGET="stm32f3x"
        PROGRAMMER_MCU="${MCU_CPN^^}" ;;
    stm32f4*)
        CPU_FLAGS="-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard"
        OPENOCD_TARGET="stm32f4x"
        PROGRAMMER_MCU="${MCU_CPN^^}" ;;
    stm32l4*)
        CPU_FLAGS="-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard"
        OPENOCD_TARGET="stm32l4x"
        PROGRAMMER_MCU="${MCU_CPN^^}" ;;
    stm32g4*)
        CPU_FLAGS="-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard"
        OPENOCD_TARGET="stm32g4x"
        PROGRAMMER_MCU="${MCU_CPN^^}" ;;
    stm32wb*)
        CPU_FLAGS="-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard"
        OPENOCD_TARGET="stm32wbx"
        PROGRAMMER_MCU="${MCU_CPN^^}" ;;
    stm32wl*)
        CPU_FLAGS="-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard"
        OPENOCD_TARGET="stm32wlx"
        PROGRAMMER_MCU="${MCU_CPN^^}" ;;
    stm32f7*)
        CPU_FLAGS="-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard"
        OPENOCD_TARGET="stm32f7x"
        PROGRAMMER_MCU="${MCU_CPN^^}" ;;
    stm32h7*)
        CPU_FLAGS="-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard"
        OPENOCD_TARGET="stm32h7x"
        PROGRAMMER_MCU="${MCU_CPN^^}" ;;
    stm32l5*)
        CPU_FLAGS="-mcpu=cortex-m33 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard"
        OPENOCD_TARGET="stm32l5x"
        PROGRAMMER_MCU="${MCU_CPN^^}" ;;
    stm32u5*)
        CPU_FLAGS="-mcpu=cortex-m33 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard"
        OPENOCD_TARGET="stm32u5x"
        PROGRAMMER_MCU="${MCU_CPN^^}" ;;
    stm32h5*)
        CPU_FLAGS="-mcpu=cortex-m33 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard"
        OPENOCD_TARGET="stm32h5x"
        PROGRAMMER_MCU="${MCU_CPN^^}" ;;
    *)
        die "Unsupported MCU: ${MCU_CPN^^} — add it to the script's case table"
        ;;
esac
info "CPU flags: $CPU_FLAGS"

# ── Get flash size from MCU part number ──────────────────────────────────────
get_flash_size_from_pn() {
    local pn="$1"
    local code
    code=$(echo "$pn" | sed -n 's/^stm32.\{4\}\(.\).*/\1/p')
    case "$code" in
        4) echo 16384 ;;   6) echo 32768 ;;   8) echo 65536 ;;
        b) echo 131072 ;;  c) echo 262144 ;;  d) echo 393216 ;;
        e) echo 524288 ;;  f) echo 786432 ;;  g) echo 1048576 ;;
        h) echo 1572864 ;; i) echo 2097152 ;; *) echo 0 ;;
    esac
}
FLASH_SIZE=$(get_flash_size_from_pn "$MCU_CPN")
if (( FLASH_SIZE > 0 )); then
    info "Flash size: $((FLASH_SIZE / 1024)) KB (from part number)"
fi

# ── Find linker script ──────────────────────────────────────────────────────
LINKER_SCRIPT=$(find "$PROJECT_DIR" -maxdepth 1 -name '*.ld' -type f | head -1)
if [[ -z "$LINKER_SCRIPT" ]]; then
    LINKER_SCRIPT=$(find "$PROJECT_DIR" -maxdepth 3 -name '*.ld' -type f | head -1)
fi
[[ -n "$LINKER_SCRIPT" ]] || die "No linker script (.ld) found"
info "Linker script: $(basename "$LINKER_SCRIPT")"

# ── Extract flash start address from linker script ──────────────────────────
FLASH_ADDR=$(grep -oP 'FLASH.*?ORIGIN\s*=\s*\K0x[0-9A-Fa-f]+' "$LINKER_SCRIPT" 2>/dev/null | head -1 || true)
FLASH_ADDR="${FLASH_ADDR:-0x8000000}"
info "Flash address: $FLASH_ADDR"

# ── Find toolchain file ─────────────────────────────────────────────────────
TOOLCHAIN_FILE=""
if [[ -f cmake/gcc-arm-none-eabi.cmake ]]; then
    TOOLCHAIN_FILE="$PROJECT_DIR/cmake/gcc-arm-none-eabi.cmake"
    info "Toolchain file: cmake/gcc-arm-none-eabi.cmake"
fi

# ── Skip to flash if --flash-only ───────────────────────────────────────────
# Build dir is per-type so all 4 configs can coexist side-by-side
BUILD_DIR="$PROJECT_DIR/build/${BUILD_TYPE}"

if [[ "$FLASH_ONLY" == true ]]; then
    step "Flash-only mode — skipping build"
    BIN_FILE="$BUILD_DIR/${PROJECT_NAME}.bin"
    ELF_FILE=""
    for candidate in "$BUILD_DIR/${PROJECT_NAME}.elf" "$BUILD_DIR/${PROJECT_NAME}"; do
        [[ -f "$candidate" ]] && ELF_FILE="$candidate" && break
    done
    [[ -f "$BIN_FILE" ]] || die "No .bin found at $BIN_FILE — run without --flash-only first"
    BIN_SIZE=$(stat -c%s "$BIN_FILE" 2>/dev/null || stat -f%z "$BIN_FILE")
    info "Using existing binary: $BIN_FILE ($BIN_SIZE bytes)"
else

# ── Clean or incremental build ──────────────────────────────────────────────
if [[ "$DO_CLEAN" == true ]]; then
    step "Cleaning build directory..."
    rm -rf "$BUILD_DIR"
else
    if [[ -d "$BUILD_DIR" ]]; then
        info "Incremental build (use --clean to force full rebuild)"
    fi
fi

# ── Install missing cube bundles (GNU ARM toolchain, Ninja, CMake etc.) ──────
step "Ensuring cube bundles are installed..."
if command -v cube &>/dev/null || command -v cube.exe &>/dev/null; then
    # Pipe "yes" to auto-confirm the installation prompt (non-interactive mode)
    echo "yes" | cube bundle install --project 2>&1 | grep -v '^$' || true
    info "Bundle install step complete"
fi

# ── Add installed bundle bin paths to PATH ───────────────────────────────────
# After cube bundle install, add the downloaded tools to PATH
BUNDLE_ROOT="${WINHOME_POSIX}/AppData/Local/stm32cube/bundles"

# Find and add Ninja (needed for -G Ninja)
NINJA_BIN=$(find "$BUNDLE_ROOT/ninja" -name "ninja.exe" -type f 2>/dev/null | head -1 | xargs -I{} dirname {} 2>/dev/null || true)
if [[ -n "$NINJA_BIN" ]]; then
    export PATH="${NINJA_BIN}:${PATH}"
    info "Ninja found: $NINJA_BIN"
else
    warn "Ninja not found in bundles — CMake may fail. Run: cube bundle install --project"
fi

# Find and add GNU ARM toolchain (arm-none-eabi-*)
GNU_TOOLS_BIN=$(find "$BUNDLE_ROOT/gnu-tools-for-stm32" -name "arm-none-eabi-gcc.exe" -type f 2>/dev/null | head -1 | xargs -I{} dirname {} 2>/dev/null || true)
if [[ -n "$GNU_TOOLS_BIN" ]]; then
    export PATH="${GNU_TOOLS_BIN}:${PATH}"
    info "ARM GCC found: $GNU_TOOLS_BIN"
fi

# Find and add CMake bundle (in case system cmake is missing)
CMAKE_BUNDLE_BIN=$(find "$BUNDLE_ROOT/cmake" -name "cmake.exe" -type f 2>/dev/null | head -1 | xargs -I{} dirname {} 2>/dev/null || true)
if [[ -n "$CMAKE_BUNDLE_BIN" ]]; then
    export PATH="${CMAKE_BUNDLE_BIN}:${PATH}"
    info "CMake bundle found: $CMAKE_BUNDLE_BIN"
fi

# ── Configure ────────────────────────────────────────────────────────────────
step "Configuring CMake ($BUILD_TYPE)..."
timer_start

CMAKE_CMD="cube-cmake"
if ! command -v cube-cmake &>/dev/null && ! command -v cube-cmake.exe &>/dev/null; then
    CMAKE_CMD="cmake"
    info "cube-cmake not found, falling back to system cmake"
else
    info "Using cube-cmake (STM32 CMake wrapper)"
fi

# Resolve ninja path for CMAKE_MAKE_PROGRAM (needed on Windows)
NINJA_EXE=$(command -v ninja.exe 2>/dev/null || command -v ninja 2>/dev/null || true)

CMAKE_ARGS=(
    -S "$PROJECT_DIR"
    -B "$BUILD_DIR"
    -G "Ninja"
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
)

if [[ "$BUILD_TYPE" == "Fast" ]]; then
    # Custom build type for extreme speed
    CMAKE_ARGS+=(
        -DCMAKE_BUILD_TYPE="Fast"
        -DCMAKE_C_FLAGS_FAST="-O3 -ffast-math -flto -DNDEBUG"
        -DCMAKE_CXX_FLAGS_FAST="-O3 -ffast-math -flto -DNDEBUG"
        -DCMAKE_EXE_LINKER_FLAGS_FAST="-flto"
    )
else
    CMAKE_ARGS+=(-DCMAKE_BUILD_TYPE="$BUILD_TYPE")
fi

# Explicitly set CMAKE_MAKE_PROGRAM if we found ninja
if [[ -n "$NINJA_EXE" ]]; then
    CMAKE_ARGS+=(-DCMAKE_MAKE_PROGRAM="$NINJA_EXE")
    info "Ninja executable: $NINJA_EXE"
fi

if [[ -n "$TOOLCHAIN_FILE" ]]; then
    CMAKE_ARGS+=(-DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN_FILE")
fi

"$CMAKE_CMD" "${CMAKE_ARGS[@]}" || die "CMake configure failed"
info "Configure time: $(timer_elapsed)"


# ── Build ────────────────────────────────────────────────────────────────────
# Use nproc if available (Git Bash), otherwise NUMBER_OF_PROCESSORS (Windows env)
JOBS="${NUMBER_OF_PROCESSORS:-$(nproc 2>/dev/null || echo 4)}"
step "Building ($JOBS parallel jobs)..."
timer_start

"$CMAKE_CMD" --build "$BUILD_DIR" -j"$JOBS" || die "Build failed"

info "Build time: $(timer_elapsed)"

# ── Locate the ELF ──────────────────────────────────────────────────────────
ELF_FILE=""
for candidate in "$BUILD_DIR/${PROJECT_NAME}.elf" "$BUILD_DIR/${PROJECT_NAME}"; do
    [[ -f "$candidate" ]] && ELF_FILE="$candidate" && break
done
[[ -n "$ELF_FILE" ]] || die "Cannot find built ELF binary in $BUILD_DIR"

# ── Generate .bin and .hex ───────────────────────────────────────────────────
BIN_FILE="$BUILD_DIR/${PROJECT_NAME}.bin"
HEX_FILE="$BUILD_DIR/${PROJECT_NAME}.hex"
arm_tool objcopy -O binary "$ELF_FILE" "$BIN_FILE" || die "objcopy .bin failed"
arm_tool objcopy -O ihex   "$ELF_FILE" "$HEX_FILE" || die "objcopy .hex failed"

# ── Symlink compile_commands.json to project root (for clangd / IDE) ────────
# On Windows use copy instead of symlink (symlink needs admin rights)
if [[ -f "$BUILD_DIR/compile_commands.json" ]]; then
    cp -f "$BUILD_DIR/compile_commands.json" "$PROJECT_DIR/compile_commands.json" 2>/dev/null || true
fi

# ── Sanity checks ───────────────────────────────────────────────────────────
BIN_SIZE=$(stat -c%s "$BIN_FILE" 2>/dev/null || stat -f%z "$BIN_FILE")
if (( BIN_SIZE == 0 )); then
    die "Generated .bin is empty!"
fi
if (( FLASH_SIZE > 0 && BIN_SIZE > FLASH_SIZE )); then
    die "Binary ($BIN_SIZE bytes) exceeds MCU flash ($((FLASH_SIZE / 1024)) KB)!"
elif (( FLASH_SIZE > 0 )); then
    USAGE_PCT=$(( (BIN_SIZE * 100) / FLASH_SIZE ))
    info "Flash usage: $BIN_SIZE / $((FLASH_SIZE / 1024))K bytes (${USAGE_PCT}%)"
else
    info "Binary size: $BIN_SIZE bytes"
fi

# ── Size summary ────────────────────────────────────────────────────────────
echo ""
arm_tool size "$ELF_FILE"
echo ""
info "Outputs:"
info "  ELF: $ELF_FILE"
info "  BIN: $BIN_FILE"
info "  HEX: $HEX_FILE"

fi   # end of build section (--flash-only jumps here)

# ── Flash ────────────────────────────────────────────────────────────────────
if [[ "$BUILD_ONLY" == true ]]; then
    echo ""
    info "Build-only mode — skipping flash"
    info "Done!"
    exit 0
fi

echo ""
MAX_RETRIES=1

# Pre-flight: check if programmer is accessible
step "Checking programmer connection..."
case "$FLASH_TOOL" in
    programmer)
        info "Using STM32CubeProgrammer CLI (STM32_Programmer_CLI.exe)"
        ;;
    openocd)
        info "Using OpenOCD for flashing"
        ;;
esac

step "Flashing via $FLASH_TOOL..."
for attempt in $(seq 1 $MAX_RETRIES); do
    info "Attempt $attempt/$MAX_RETRIES → address $FLASH_ADDR"

    FLASH_OK=false
    case "$FLASH_TOOL" in
        programmer)
            # STM32CubeProgrammer CLI — primary tool on Windows (no st-flash needed)
            if STM32_Programmer_CLI -c port=SWD mode=UR -d "$BIN_FILE" "$FLASH_ADDR" -v -rst 2>&1; then
                FLASH_OK=true
            fi
            ;;
        openocd)
            OCD_CFG=""
            for iface in "interface/stlink.cfg" "interface/stlink-v2.cfg" "interface/stlink-v2-1.cfg"; do
                if [[ -f "/usr/share/openocd/scripts/$iface" ]] || \
                   [[ -f "/usr/local/share/openocd/scripts/$iface" ]] || \
                   [[ -f "C:/openocd/scripts/$iface" ]]; then
                    OCD_CFG="$iface"
                    break
                fi
            done
            [[ -n "$OCD_CFG" ]] || OCD_CFG="interface/stlink.cfg"

            if openocd -f "$OCD_CFG" \
                       -f "target/${OPENOCD_TARGET}.cfg" \
                       -c "program $ELF_FILE verify reset exit" 2>&1; then
                FLASH_OK=true
            fi
            ;;
        serial)
            SERIAL_PORT=$(ls /dev/ttyUSB* /dev/ttyACM* /dev/ttyS* 2>/dev/null | head -1 || true)
            [[ -n "$SERIAL_PORT" ]] || die "No serial port found for UART flashing"
            info "Using serial port: $SERIAL_PORT"
            if stm32flash -w "$BIN_FILE" -v -g "$FLASH_ADDR" "$SERIAL_PORT" 2>&1; then
                FLASH_OK=true
            fi
            ;;
        st-flash)
            # Kept for compatibility if user installs st-flash on Windows
            if st-flash --reset write "$BIN_FILE" "$FLASH_ADDR" 2>&1; then
                FLASH_OK=true
            fi
            ;;
    esac

    if [[ "$FLASH_OK" == true ]]; then
        if [[ "$DO_VERIFY" == true && "$FLASH_TOOL" == "programmer" ]]; then
            step "Verification was done by STM32_Programmer_CLI (-v flag)"
        fi

        echo ""
        echo -e "${GREEN}${BOLD}══════════════════════════════════════════════════════════════${NC}"
        echo -e "${GREEN}${BOLD}  Flash successful! Device has been reset and is running.${NC}"
        echo -e "${GREEN}${BOLD}══════════════════════════════════════════════════════════════${NC}"
        echo ""
        exit 0
    fi

    warn "Flash attempt $attempt failed"
    if (( attempt < MAX_RETRIES )); then
        info "Retrying in 2 seconds..."
        sleep 2
    fi
done

echo ""
die "Flashing failed after $MAX_RETRIES attempts.
  Windows troubleshooting:
  - Check USB cable and ST-Link connection
  - Try: STM32_Programmer_CLI -c port=SWD -l
  - Make sure STM32 VS Code Extension drivers are installed
  - Try unplugging and re-plugging the board
  - Run STM32CubeProgrammer GUI once to accept any driver prompts
  - STM32_Programmer_CLI path: $PROGRAMMER_BIN"
