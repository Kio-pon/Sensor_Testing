# CMAKE Build Configuration for Line Follower

## Update CMakeLists.txt

Find your `CMakeLists.txt` in the project root and add `Core/Src/line_follower.c` to the `target_sources()` section.

### Before (example):
```cmake
target_sources(${TARGET_NAME} PRIVATE
  Core/Src/main.c
  Core/Src/robot_core.c
  Core/Src/motor_driver.c
  Core/Src/qtr_array.c
  Core/Src/pid.c
  # ... other files ...
)
```

### After (add this line):
```cmake
target_sources(${TARGET_NAME} PRIVATE
  Core/Src/main.c
  Core/Src/line_follower.c           # ADD THIS LINE
  Core/Src/robot_core.c
  Core/Src/motor_driver.c
  Core/Src/qtr_array.c
  Core/Src/pid.c
  # ... other files ...
)
```

## Include Directories

Make sure `Core/Inc` is in your include directories (should already be):

```cmake
target_include_directories(${TARGET_NAME} PRIVATE
  Core/Inc
  # ... other include paths ...
)
```

## Build Commands

After updating CMakeLists.txt:

```bash
# Generate build files
cmake -B build -DCMAKE_TOOLCHAIN_FILE=<your-toolchain>.cmake

# Build
cmake --build build

# Flash (if using your flash script)
./flash_windows.sh
# or
./flash_linux.sh
```

## Verification

After building, check for:
- ✅ No errors during compilation
- ✅ `line_follower.c` successfully compiled
- ✅ File size increased by ~5-10KB (line follower code)

If you see undefined reference errors like:
```
undefined reference to `LineFollower_Init'
```

It means `line_follower.c` wasn't compiled. Double-check:
1. File exists: `Core/Src/line_follower.c`
2. Added to CMakeLists.txt `target_sources()`
3. Rebuild from clean: `rm -rf build && cmake -B build ...`

## robot_config.h Settings

Ensure these are enabled (they probably already are):

```c
#define ENABLE_CHASSIS      1       // Motor control
#define ENABLE_QTR_ARRAY    1       // Sensor reading
#define ENABLE_TELEMETRY    1       // Debug output (optional)
```

If any are set to 0, the line follower won't work.

## Visual Studio Code Task Configuration

If using VS Code with CMake extension, add this to `.vscode/tasks.json`:

```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "Build (Line Follower)",
            "type": "shell",
            "command": "cmake",
            "args": [
                "--build",
                "build"
            ],
            "group": {
                "kind": "build",
                "isDefault": true
            },
            "problemMatcher": [
                "$gcc"
            ]
        },
        {
            "label": "Clean Build",
            "type": "shell",
            "command": "rm",
            "args": [
                "-rf",
                "build"
            ],
            "dependsOn": [
                "Build (Line Follower)"
            ]
        }
    ]
}
```

Then use Ctrl+Shift+B to build.

## Compiler Flags

No special flags needed. The line follower works with standard ARM Cortex-M4 compilation:

```cmake
# Your existing flags should include:
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 -Wall")  # Optimization level 2
```

The code is optimized for both performance and code size.

## Linker Notes

The line follower uses:
- `math.h` - Already linked in STM32 projects
- `stdio.h` - Already linked
- `stdbool.h` - Already linked
- `string.h` - NOT used (safe for embedded)

No additional linker flags required.

## Memory Usage

Estimated footprint:
- **Code:** ~4-6 KB (flash)
- **Data:** ~200 bytes (SRAM)

Very light for embedded systems.

## Debug Configuration

If using OpenOCD/GDB debugging, no special config needed. You can:
- Set breakpoints in `line_follower.c`
- Watch `line_follower.state` variable
- Monitor motor speeds in real-time

Example GDB command:
```gdb
(gdb) watch line_follower.left_motor_speed
```

---

## Quick Checklist

- [ ] Added `Core/Src/line_follower.c` to CMakeLists.txt
- [ ] Verified `Core/Inc/` in include directories
- [ ] ENABLE_QTR_ARRAY = 1 in robot_config.h
- [ ] ENABLE_CHASSIS = 1 in robot_config.h
- [ ] Built successfully with no errors
- [ ] Program flashed to board
- [ ] Can send 'S' command to start

Done! 🚀

