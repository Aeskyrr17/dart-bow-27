#pragma once

#include "DelayHelper.hpp"
#include "magicmsgs.hpp"
#include "math.hpp"
#include <cstdint>

#define TOF_DATA_SIZE 9

#define DART_GANTRY_POS_NONE (Numeric::Pi)
#define DART_GANTRY_POS_SLOT_1 (-Numeric::Pi/2.0f)
#define DART_GANTRY_POS_SLOT_2 (0.0f)
#define DART_GANTRY_POS_SLOT_3 (Numeric::Pi/2.0f)

typedef enum
{
    IDLE = 0,
    PREPARING,
    READY,
    FIRING,
    HAND_CONTROL,
    ERROR_STOP,
    PRE_TENSION,
    LAUNCHER_FSM_STATE_INVALID = 0xFF
} LAUNCHER_FSM_STATE;

typedef enum
{
    SYN_1 = 0,
    GANTRY_1,
    SYN_2,
    GANTRY_2,
    SYN_TRIGGER_READY,
    TENSION_AND_RETRACT_AND_YAW,
    PREPARE_STATE_INVALID = 0xFF
} PREPARE_STATE;

struct Launcher_Cxt_t
{
    LAUNCHER_FSM_STATE fsm_state;
    PREPARE_STATE prep_state;

    DART_SLOT current_slot;
    bool is_first_dart;
    bool is_fire_done;
    bool last_fire_done;

    ULONG fire_done_hold_ticks = 5;
    delay_t fire_done_hold_delay;
};

inline void Update_Slot(Launcher_Cxt_t& cxt)
{
    switch (cxt.current_slot)
    {
        case DART_SLOT_NONE:
            cxt.current_slot = DART_SLOT_1;
            break;
        case DART_SLOT_1:
            cxt.current_slot = DART_SLOT_2;
            break;
        case DART_SLOT_2:
            cxt.current_slot = DART_SLOT_3;
            break;
        case DART_SLOT_3:
        default:
            cxt.current_slot = DART_SLOT_NONE;
            break;
    }
}

inline void Mark_Fire_Done(Launcher_Cxt_t& cxt, uint8_t hold_ticks = 5)
{
    cxt.fire_done_hold_delay.Reset();
    cxt.is_fire_done = true;
    cxt.fire_done_hold_ticks = hold_ticks;
}

inline void Step_Fire_Done(Launcher_Cxt_t& cxt)
{
    cxt.last_fire_done = cxt.is_fire_done;

    if (!cxt.is_fire_done)
    {
        cxt.fire_done_hold_delay.Reset();
        return;
    }

    if (cxt.fire_done_hold_delay.Reach(cxt.fire_done_hold_ticks))
    {
        cxt.is_fire_done = false;
    }
}

inline float Get_Gantry_Target_Pos(DART_SLOT slot)
{
    switch (slot)
    {
        case DART_SLOT_NONE:
            return DART_GANTRY_POS_NONE;
        case DART_SLOT_1:
            return DART_GANTRY_POS_SLOT_1;
        case DART_SLOT_2:
            return DART_GANTRY_POS_SLOT_2;
        case DART_SLOT_3:
            return DART_GANTRY_POS_SLOT_3;
        default:
            return DART_GANTRY_POS_NONE;
    }
}
