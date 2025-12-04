#include "referee.hpp"
#include "string.h"
#include "bsp_dwt.hpp"

void Referee::HandleMsg(uint8_t *_Msgptr)
{
    uint16_t cmd_id = 0;
    memcpy(&cmd_id, _Msgptr, sizeof(uint16_t));
    _Msgptr += sizeof(uint16_t);
    switch (cmd_id)
    {
    case JudgeID::GameStatus:
        memcpy(&GameStatus, _Msgptr, sizeof(GameStatus));
        break;

    case JudgeID::GameResult:
        memcpy(&GameResult, _Msgptr, sizeof(GameResult));
        break;

    case JudgeID::RobotHP:
        memcpy(&RobotHP, _Msgptr, sizeof(RobotHP));
        break;

    case JudgeID::EventData:
        memcpy(&EventData, _Msgptr, sizeof(EventData));
        break;

    case JudgeID::RefereeWarning:
        memcpy(&RefereeWarning, _Msgptr, sizeof(RefereeWarning));
        break;

    case JudgeID::DartInfo:
        memcpy(&DartInfo, _Msgptr, sizeof(DartInfo));
        break;

    case JudgeID::GameRobotStatus:
        memcpy(&GameRobotStatus, _Msgptr, sizeof(GameRobotStatus));
        GameRobotStatusTick = DWT_GetTimeline_ms();
        break;

    case JudgeID::PowerHeatData:
        memcpy(&PowerHeatData, _Msgptr, sizeof(PowerHeatData));
        PowerHeatTick = DWT_GetTimeline_ms();
        break;

    case JudgeID::GameRobotPos:
        memcpy(&GameRobotPos, _Msgptr, sizeof(GameRobotPos));
        break;

    case JudgeID::Buff:
        memcpy(&Buff, _Msgptr, sizeof(Buff));
        break;

    case JudgeID::RobotHurt:
        memcpy(&RobotHurt, _Msgptr, sizeof(RobotHurt));
        break;

    case JudgeID::ShootData:
        memcpy(&ShootData, _Msgptr, sizeof(ShootData));
        break;

    case JudgeID::RfidStatus:
        memcpy(&RfidStatus, _Msgptr, sizeof(RfidStatus));
        break;

    case JudgeID::DartClientCmd:
        memcpy(&DartClientCmd, _Msgptr, sizeof(DartClientCmd));
        break;

    case JudgeID::RoboInteractData:
        memcpy(&RoboInteractData, _Msgptr, sizeof(RoboInteractData));
        break;

    default:
        break;
    }
}