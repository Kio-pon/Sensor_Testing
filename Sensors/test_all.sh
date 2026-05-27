#!/usr/bin/env bash
cd /c/Users/Student/NERC/Sensor_Testing/Sensors
FLASH_SCRIPT="/c/Users/Student/NERC/Sensor_Testing/Sensors/FINAL_IOC/flash_windows.sh"

echo "| Project | Debug (-O0) | Release (-Os) | Fast (-O3 -flto) |" > results.txt
echo "|---|---|---|---|" >> results.txt

for d in "Color_Sensor" "L298N" "Line_tracker" "Robot_Integration" "Servo" "Sharp Sensor" "FINAL_IOC"; do
    if [ ! -d "$d" ]; then continue; fi
    if [ ! -f "$d/CMakeLists.txt" ]; then continue; fi
    echo "Building $d..."
    
    out_debug=$("$FLASH_SCRIPT" --build-only --debug "$d" 2>&1 | grep "FLASH:" | awk '{print $2}')
    out_release=$("$FLASH_SCRIPT" --build-only --release "$d" 2>&1 | grep "FLASH:" | awk '{print $2}')
    out_fast=$("$FLASH_SCRIPT" --build-only --fast "$d" 2>&1 | grep "FLASH:" | awk '{print $2}')
    
    out_debug=${out_debug:-FAIL}
    out_release=${out_release:-FAIL}
    out_fast=${out_fast:-FAIL}
    
    echo "| $d | ${out_debug} B | ${out_release} B | ${out_fast} B |" >> results.txt
done
cat results.txt
