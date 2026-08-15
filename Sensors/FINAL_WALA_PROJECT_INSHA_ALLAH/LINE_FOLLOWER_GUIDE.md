# Robust Line Follower Integration Guide

## Overview

This is a **competitive-grade, snappy line follower** designed for front-only movement. It reads the front QTR-8A sensor array and controls motor speeds with differential steering for smooth, fast line tracking.

**Key Features:**
- ✅ 200 Hz control frequency (5ms loop)
- ✅ Aggressive PID tuning for snappy response
- ✅ Differential motor control (left/right steering)
- ✅ Line detection with timeout handling
- ✅ Safe motor speed clipping
- ✅ Simple API - just call `LineFollower_Update()` and `LineFollower_GetMotorSpeeds()`

---

## Quick Start (2 minutes)

### 1. Add Headers to `robot_core.c`
```c
#include "line_follower.h"
```

### 2. Create Global Instance in `robot_core.c`
```c
static LineFollower_t line_follower;
```

### 3. Initialize in `Robot_Init()`
```c
void Robot_Init(void) {
    // ... existing code ...
    
    LineFollower_Init(&line_follower);
    
    // ... rest of init ...
}
```

### 4. Replace Navigation Loop in `Robot_RunLoop()`

**Remove/disable this section:**
```c
/* OLD - Comment out or remove:
Nav_RunSequence();
Nav_Update();
*/
```

**Add this instead:**
```c
/* === LINE FOLLOWING === */
if (control_due) {
    control_due = 0;
    
    // Read sensors
    QTR_Poll(&hadc1, &hadc2, &hadc3, &hadc4);
    
    // Update line follower
    LineFollower_Update(&line_follower, HAL_GetTick());
    
    // Get motor speeds
    int32_t left_speed, right_speed;
    LineFollower_GetMotorSpeeds(&line_follower, &left_speed, &right_speed);
    
    // Send to motors (for mecanum setup)
    Motor_SetSpeed(&chassis.fl, left_speed);
    Motor_SetSpeed(&chassis.rl, left_speed);
    Motor_SetSpeed(&chassis.fr, right_speed);
    Motor_SetSpeed(&chassis.rr, right_speed);
}
```

### 5. Start Following

**From UART/debug console:**
```
Send 'S' to start
Send 'X' to stop
Send '+' to increase speed
Send '-' to decrease speed
Send 'C' to show config
```

**Or from code:**
```c
LineFollower_Start(&line_follower);
```

---

## Performance Tuning

### Speed (`base_speed`)
- **Range:** 400-1800 PWM units
- **Default:** 1400
- **Effect:** Higher = faster forward speed
- **Tune:** Increase for wider tracks, decrease for tight tracks or slippery surfaces

**Increase speed if:**
- Robot is going too slow
- You want more aggressive racing

**Decrease speed if:**
- Robot oscillates left/right
- Robot loses the line frequently
- Track is narrow or slippery

### Steering Responsiveness (`kp_steering`)
- **Range:** 0.5-2.5f
- **Default:** 1.2f
- **Effect:** How aggressively robot steers to line center
- **Higher = snappier but oscillatory**

**Increase Kp if:**
- Robot follows sluggishly (slow to correct)
- You want sharper turns

**Decrease Kp if:**
- Robot oscillates (wiggles back and forth)
- Robot oversteers on turns

### Damping (`kd_steering`)
- **Range:** 0.3-1.5f
- **Default:** 0.8f
- **Effect:** Smooths out steering oscillations
- **Mutual with Kp - balance for snappy+smooth**

**Increase Kd if:**
- Too much wiggling/oscillation
- Robot hunts for centerline

**Decrease Kd if:**
- Robot feels sluggish on corrections
- Needs sharper response (but watch for oscillation)

### Integral Term (`ki_steering`)
- **Range:** 0.0-0.2f
- **Default:** 0.05f
- **Effect:** Corrects systematic drift (rare)
- **Keep low for line following**

### Line Detection Threshold (`line_threshold`)
- **Range:** 1500-3000
- **Default:** 2000
- **Effect:** ADC value above which sensor counts as "on line"
- **Depends on lighting, track color, sensor cleanliness**

**Increase if:**
- Getting false positives (white floor seen as line)
- Sensors are dirty

**Decrease if:**
- Line detection is weak/unreliable
- Track is dark

---

## Tuning Strategy

### For a Track You Don't Know:

1. **Start conservative:**
   ```c
   base_speed = 1000
   max_steering = 600
   kp_steering = 0.8f
   kd_steering = 0.8f
   ```

2. **Test at low speed:**
   - Check that line is being followed (not oscillating)
   - Robot should gently correct toward center

3. **Increase Kp gradually** (0.8 → 1.0 → 1.2 → 1.5):
   - Each increase should make steering snappier
   - Stop when you see small oscillations

4. **Increase Kd slightly** (0.8 → 0.9 → 1.0):
   - Smooth out oscillations
   - Should reduce wiggling

5. **Increase speed** (1000 → 1200 → 1400 → 1600):
   - Test at each speed level
   - Speed has huge impact - tune smaller gains first, then speed

### Final Competition Tuning:

Once basic following works:
```c
/* Maximum speed with stable following */
base_speed = 1500 - 1700   // as fast as stable
kp_steering = 1.2 - 1.5    // snappy but not oscillatory
kd_steering = 0.7 - 1.0    // smoothing
max_steering = 650         // limit steering correction range
```

---

## State Machine

The line follower operates in 4 states:

| State | Meaning | Motors |
|-------|---------|--------|
| `LF_IDLE` | Waiting for start | **OFF** |
| `LF_RUNNING` | Following line | **ACTIVE** - differentially steered |
| `LF_LINE_LOST` | Line lost for >200ms | **OFF** |
| `LF_FINISHED` | Course complete | **OFF** (optional) |

**Commands:**
- `LineFollower_Start()` → Move from IDLE to RUNNING
- `LineFollower_Stop()` → Move to IDLE
- Auto-transition RUNNING → LINE_LOST if line missing for 200ms
- `LineFollower_Reset()` → Clear error accumulation (for restarting same track)

---

## API Reference

### Initialization
```c
void LineFollower_Init(LineFollower_t *lf);
void LineFollower_SetConfig(LineFollower_t *lf, const LineFollower_Config_t *config);
```

### Control
```c
void LineFollower_Start(LineFollower_t *lf);
void LineFollower_Stop(LineFollower_t *lf);
void LineFollower_Reset(LineFollower_t *lf);
```

### Updates (call in 200 Hz loop)
```c
void LineFollower_Update(LineFollower_t *lf, uint32_t time_ms);
```

### Data Retrieval
```c
void LineFollower_GetMotorSpeeds(LineFollower_t *lf, 
                                 int32_t *left_speed, 
                                 int32_t *right_speed);
LineFollower_State_t LineFollower_GetState(LineFollower_t *lf);
bool LineFollower_IsLineDetected(LineFollower_t *lf);
int32_t LineFollower_GetSteeringCorrection(LineFollower_t *lf);
```

---

## Understanding Motor Speeds

The line follower calculates two motor speeds:

```
LEFT_MOTOR_SPEED = base_speed + steering_correction
RIGHT_MOTOR_SPEED = base_speed - steering_correction
```

**Example: Line is 500 units to the right**
- `steering_correction = 250` (positive = turn right)
- `LEFT_MOTOR = 1400 + 250 = 1650` (faster)
- `RIGHT_MOTOR = 1400 - 250 = 1150` (slower)
- **Result:** Robot turns right toward line

**Example: Line is 500 units to the left**
- `steering_correction = -250` (negative = turn left)
- `LEFT_MOTOR = 1400 - 250 = 1150` (slower)
- `RIGHT_MOTOR = 1400 + 250 = 1650` (faster)
- **Result:** Robot turns left toward line

---

## Troubleshooting

### Robot won't move
- ✓ Check `base_speed` > 400
- ✓ Check motors respond to manual commands
- ✓ Press 'S' to start or call `LineFollower_Start()`
- ✓ Verify QTR sensors are reading (check telemetry)

### Robot doesn't follow the line
- ✓ Check telemetry: Is line position changing? (0 = on line, ±3500 = off line)
- ✓ Increase `line_threshold` if "no line detected"
- ✓ Verify `qtr_front[8]` array is being populated correctly
- ✓ Check sensor wiring and ADC channels

### Robot oscillates/wiggles
- ✓ Decrease `kp_steering` (1.2 → 1.0)
- ✓ Increase `kd_steering` (0.8 → 1.0)
- ✓ Decrease `base_speed`

### Robot loses line easily
- ✓ Decrease `base_speed`
- ✓ Increase `kp_steering` for sharper corrections
- ✓ Check line detection threshold
- ✓ Ensure clean sensors

### Robot goes straight instead of steering
- ✓ Check steering correction with `LineFollower_GetSteeringCorrection()`
- ✓ Verify motor trim values are equal (no internal imbalance)
- ✓ Test left/right motors independently

---

## Files Created

1. **`line_follower.h`** - Header file, API definitions
2. **`line_follower.c`** - Core implementation, PID steering algorithm
3. **`line_follower_main.c`** - Integration example and helper functions
4. **`LINE_FOLLOWER_GUIDE.md`** - This file

---

## Implementation Details

### Control Loop (5ms, 200 Hz)

```
1. QTR_Poll()                           ← Read sensors
2. LineFollower_Update()                ← PID steering calculation
   ├─ Detect line position
   ├─ Apply steering PID
   └─ Calculate L/R motor speeds
3. Motor_SetSpeed()                     ← Send to motors
4. Telemetry (optional, 100ms)
```

### Steering Algorithm

The steering correction is calculated as:

```
error = line_position - 0           // 0 = center
steering = PID(error, dt)           // PID controller
left_speed = base_speed + steering  // Differential drive
right_speed = base_speed - steering
```

With PID parameters tuned for:
- **Fast settling** (high Kp)
- **Smooth motion** (balanced Kd)
- **Minimal overshoot** (low Ki)

---

## Competition Tips

1. **Calibrate before race:**
   - Run on actual track before competition
   - Adjust tuning for that specific lighting/color

2. **Keep PID gains balanced:**
   - Snappy but not oscillatory = faster on actual track
   - Oscillating = loses time correcting errors

3. **Test line lost handling:**
   - Verify robot stops safely if line is lost
   - 200ms timeout is good default

4. **Speed vs Stability:**
   - Faster isn't always better
   - Consistent + fast > Inconsistent + very fast
   - Find speed where 90% of time is on line

5. **Motor tuning:**
   - Ensure left/right motors have similar response
   - Adjust trim values if robot curves naturally

---

## License & Attribution

This line follower is optimized for **STM32F3 microcontroller** running at **200 Hz** with **8-sensor front array**.

Designed for competitive robot line following with:
- Millisecond-level response time
- Minimal latency (direct sensor → motor)
- Robust line detection
- Simple, reliable API

Good luck on the track! 🏁

