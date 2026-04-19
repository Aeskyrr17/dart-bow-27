#include "magicmsgs.hpp"
#include <cstdint>

#define TOF_DATA_SIZE 9

#define DART_GANTRY_POS_NONE (3.1415926f)
#define DART_GANTRY_POS_SLOT_1 (-1.5707963f)
#define DART_GANTRY_POS_SLOT_2 (0.0f)
#define DART_GANTRY_POS_SLOT_3 (1.5707963f)

typedef enum
{
    IDLE = 0,
    PREPARING,
    READY,
    FIRING,
    HAND_CONTROL,
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
} PREPARE_STSTE;

struct Launcher_Cxt_t
{
    LAUNCHER_FSM_STATE fsm_state;
    PREPARE_STSTE prep_state;

    DART_SLOT current_slot;
    bool is_first_dart;
    bool is_fire_done;
    bool last_fire_done;
    uint8_t fire_done_hold_ticks;
};

struct CoilResetCxt_t
{
    bool homed;
    float zero_pos;
};

class delay_t
{
public:
    void Reset();
    // Reach: first trigger starts timing; ReachStable: condition must stay true for the whole delay.
    bool Reach(ULONG delay_ticks, bool delay_init = false);
    bool Reach(bool delay_trigger, ULONG delay_ticks, bool delay_init = false);
    bool ReachStable(bool delay_enable, ULONG delay_ticks, bool delay_init = false);

private:
    bool started = false;
    bool delay_ok = false;
    ULONG start_tick = 0;
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
    cxt.is_fire_done = true;
    cxt.fire_done_hold_ticks = hold_ticks;
}

inline void Step_Fire_Done(Launcher_Cxt_t& cxt)
{
    cxt.last_fire_done = cxt.is_fire_done;

    if (cxt.fire_done_hold_ticks > 0)
    {
        cxt.fire_done_hold_ticks--;
    }

    cxt.is_fire_done = (cxt.fire_done_hold_ticks > 0);
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
