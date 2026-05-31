#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "main.h"
#include "robot_config.h"

/* High-level moves. Sign convention: positive vx = forward,
   positive vy = right, positive omega = clockwise. The motor
   driver owns the physical inversion, so navigation passes
   plain values. */
typedef enum {
    NAV_IDLE,
    NAV_FOLLOW_LINE,
    NAV_STRAFE,          /* param sign chooses left/right */
    NAV_TURN,            /* param sign chooses CCW/CW */
    NAV_DRIVE_TICKS,     /* blind straight move by encoder */
    NAV_APPROACH_SHELF,  /* sharp-guided, then blind handoff */
    NAV_DEPOSIT,
    NAV_COLOR_SEARCH     /* hands off to the branching routine */
} NavState_t;

/* Any non-zero field is an active stop condition. First to
   fire ends the step. */
typedef struct {
    int32_t max_ticks;          /* 0 = ignore */
    float sharp_below_cm;       /* 0 = ignore */
    uint8_t junctions_to_cross; /* 0 = ignore */
} NavCondition_t;

/* Which sensors run their PID during this step (bitmask). */
#define CORR_QTR_FRONT 0x01
#define CORR_QTR_RIGHT 0x02
#define CORR_QTR_LEFT  0x04
#define CORR_SHARP     0x08

typedef struct {
    NavState_t state;
    int32_t base_speed;
    int32_t param;              /* ticks, sign, etc. */
    NavCondition_t trigger;     /* what starts the step */
    NavCondition_t confirmation;/* second sensor must agree */
    uint8_t correction_mask;    /* PIDs active this step */
    NavCondition_t landing;     /* what ends the step */
} NavCommand_t;

/* Junction counter, tick-debounced so it is speed independent. */
void Junctions_Update(void);
uint32_t Junctions_GetCount(void);
void Junctions_Reset(void);

/* Mission sequencer */
void Nav_LoadMission(NavCommand_t *commands, uint8_t count);
void Nav_TickMission(float dt);
NavState_t Nav_GetState(void);

#endif /* NAVIGATION_H */
