#include "DartHostService.hpp"

#include <cmath>
#include <string.h>

#include "config_launcher.hpp"
#include "tx_api.h"

namespace
{
constexpr uint8_t HOSTREQ_FIFO_LEN = 16;
constexpr uint8_t HOSTREQ_MAX_PROCESS_PER_POLL = 4;
constexpr ULONG HOST_FORCE_TELEMETRY_PERIOD_TICKS = 10;
constexpr ULONG HOST_TEST_STATE_PERIOD_TICKS = 50;

constexpr float HOST_YAW_MIN = -5.0f;
constexpr float HOST_YAW_MAX = 5.0f;
constexpr float HOST_TENSION_MIN = 0.0f;
constexpr float HOST_TENSION_MAX = 120.0f;
constexpr float HOST_PRE_TENSION_MIN = 0.0f;
constexpr float HOST_PRE_TENSION_MAX = 120.0f;

void HostPutU16(uint8_t* out, uint16_t value)
{
    out[0] = static_cast<uint8_t>(value & 0xFFU);
    out[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
}

void HostPutU32(uint8_t* out, uint32_t value)
{
    out[0] = static_cast<uint8_t>(value & 0xFFU);
    out[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
    out[2] = static_cast<uint8_t>((value >> 16U) & 0xFFU);
    out[3] = static_cast<uint8_t>((value >> 24U) & 0xFFU);
}

void HostPutFloat(uint8_t* out, float value)
{
    memcpy(out, &value, sizeof(value));
}

void HostPublish(om_topic_t* topic, const msg_hosttx_t& tx)
{
    if (topic != nullptr)
    {
        om_publish(topic, const_cast<msg_hosttx_t*>(&tx), sizeof(tx), true, false);
    }
}

void HostSendAck(om_topic_t* topic, uint8_t ref_type, uint8_t ref_seq,
                 uint8_t result, uint16_t revision)
{
    msg_hosttx_t tx{};
    tx.valid = 1;
    tx.type = HOST_TYPE_ACK;
    tx.seq = ref_seq;
    tx.len = 6;
    tx.payload[0] = ref_type;
    tx.payload[1] = ref_seq;
    tx.payload[2] = result;
    tx.payload[3] = 0;
    HostPutU16(&tx.payload[4], revision);
    HostPublish(topic, tx);
}

void HostSendError(om_topic_t* topic, uint8_t ref_type, uint8_t ref_seq,
                   uint8_t error_code, uint16_t detail)
{
    msg_hosttx_t tx{};
    tx.valid = 1;
    tx.type = HOST_TYPE_ERROR_RSP;
    tx.seq = ref_seq;
    tx.len = 6;
    tx.payload[0] = ref_type;
    tx.payload[1] = ref_seq;
    tx.payload[2] = error_code;
    tx.payload[3] = 0;
    HostPutU16(&tx.payload[4], detail);
    HostPublish(topic, tx);
}

void HostSendDartParamState(om_topic_t* topic, const DartLibrary& dart, uint8_t seq, uint8_t dart_id)
{
    if (dart_id < 1 || dart_id > 16)
    {
        return;
    }

    const Dart_Config_t& item = dart.config.dart[dart_id];
    msg_hosttx_t tx{};
    tx.valid = 1;
    tx.type = HOST_TYPE_DART_PARAM_STATE;
    tx.seq = seq;
    tx.len = 15;
    HostPutU16(&tx.payload[0], dart.config.config_revision);
    tx.payload[2] = dart_id;
    HostPutFloat(&tx.payload[3], item.yaw_offset);
    HostPutFloat(&tx.payload[7], item.tension_kg_base);
    HostPutFloat(&tx.payload[11], item.tension_kg_outpost);
    HostPublish(topic, tx);
}

void HostSendSequenceState(om_topic_t* topic, const DartLibrary& dart, uint8_t seq)
{
    msg_hosttx_t tx{};
    tx.valid = 1;
    tx.type = HOST_TYPE_SEQUENCE_STATE;
    tx.seq = seq;
    tx.len = 6;
    HostPutU16(&tx.payload[0], dart.config.config_revision);
    for (uint8_t i = 0; i < 4; i++)
    {
        tx.payload[2 + i] = static_cast<uint8_t>(dart.config.sequence[i]);
    }
    HostPublish(topic, tx);
}

void HostSendPreTensionState(om_topic_t* topic, const DartLibrary& dart, uint8_t seq)
{
    msg_hosttx_t tx{};
    tx.valid = 1;
    tx.type = HOST_TYPE_PRE_TENSION_STATE;
    tx.seq = seq;
    tx.len = 6;
    HostPutU16(&tx.payload[0], dart.config.config_revision);
    HostPutFloat(&tx.payload[2], dart.config.pre_tension_kg);
    HostPublish(topic, tx);
}

void HostSendHeartbeat(om_topic_t* topic, const DartLibrary& dart, uint8_t seq)
{
    msg_hosttx_t tx{};
    tx.valid = 1;
    tx.type = HOST_TYPE_HEARTBEAT_RSP;
    tx.seq = seq;
    tx.len = 8;
    tx.payload[0] = 0;
    tx.payload[1] = HOSTCOMM_PROTOCOL_VERSION;
    HostPutU32(&tx.payload[2], tx_time_get());
    HostPutU16(&tx.payload[6], dart.config.config_revision);
    HostPublish(topic, tx);
}

void HostSendForceTelemetry(om_topic_t* topic, uint8_t seq,
                            const msg_motor_ctrl_t& motorctrl,
                            const msg_sensor_t& sensor)
{
    msg_hosttx_t tx{};
    tx.valid = 1;
    tx.type = HOST_TYPE_FAST_TELEMETRY;
    tx.seq = seq;
    tx.len = 21;
    HostPutU32(&tx.payload[0], tx_time_get());
    HostPutFloat(&tx.payload[4], motorctrl.string_L_tension_kg);
    HostPutFloat(&tx.payload[8], motorctrl.string_R_tension_kg);
    HostPutFloat(&tx.payload[12], sensor.string_L_force_kg);
    HostPutFloat(&tx.payload[16], sensor.string_R_force_kg);
    tx.payload[20] = motorctrl.string_able ? 1U : 0U;

    // Telemetry is best effort so it never holds up the control thread.
    if (topic != nullptr)
    {
        om_publish(topic, &tx, sizeof(tx), false, false);
    }
}

void HostSendTestState(om_topic_t* topic, uint8_t seq,
                       bool test_mode_enabled, bool test_control_allowed,
                       HOST_TEST_ACTION test_action,
                       const msg_launcher2sysctrl_t& lch2sys,
                       float tension_left, float tension_right)
{
    msg_hosttx_t tx{};
    tx.valid = 1;
    tx.type = HOST_TYPE_TEST_STATE;
    tx.seq = seq;
    tx.len = 14;
    tx.payload[0] = test_mode_enabled ? 1U : 0U;
    tx.payload[1] = test_control_allowed ? 1U : 0U;
    tx.payload[2] = static_cast<uint8_t>(test_action);
    tx.payload[3] = lch2sys.current_state;
    tx.payload[4] = lch2sys.prepare_state;
    tx.payload[5] = static_cast<uint8_t>(lch2sys.fire_source);
    HostPutFloat(&tx.payload[6], tension_left);
    HostPutFloat(&tx.payload[10], tension_right);
    HostPublish(topic, tx);
}

bool HostFloatInRange(float value, float min_value, float max_value)
{
    return std::isfinite(value) && value >= min_value && value <= max_value;
}
}

void DartHostService::Init()
{
    hostreq_fifo_ = nullptr;
    hosttx_topic_ = nullptr;
    sensor_suber_ = nullptr;
    motorctrl_suber_ = nullptr;
    latest_sensor_ = {};
    latest_motorctrl_ = {};
    last_force_telemetry_tick_ = 0;
    force_telemetry_seq_ = 0;
    test_mode_enabled_ = false;
    test_control_allowed_ = false;
    test_tension_left_ = 0.0f;
    test_tension_right_ = 0.0f;
    test_action_ = HOST_TEST_ACTION_NONE;
    last_host_activity_tick_ = 0;
    last_test_state_tick_ = 0;
    test_state_seq_ = 0;
    EnsureTopicsReady();
}

void DartHostService::Poll(DartLibrary& dart, const msg_launcher2sysctrl_t& lch2sys,
                           bool test_control_allowed, ULONG test_mode_timeout_ticks)
{
    if (!EnsureTopicsReady())
    {
        return;
    }

    UpdateForceTelemetrySources();
    test_control_allowed_ = test_control_allowed;

    const ULONG now = tx_time_get();
    if (test_mode_enabled_ &&
        (!test_control_allowed_ ||
         static_cast<ULONG>(now - last_host_activity_tick_) >= test_mode_timeout_ticks))
    {
        DisableTestMode();
    }

    msg_hostreq_t req{};
    for (uint8_t i = 0; i < HOSTREQ_MAX_PROCESS_PER_POLL && om_fifo_readable(hostreq_fifo_); i++)
    {
        if (om_fifo_read(hostreq_fifo_, &req) == OM_OK)
        {
            last_host_activity_tick_ = tx_time_get();
            ProcessRequest(req, lch2sys, test_control_allowed_, dart);
        }
    }

    PublishForceTelemetry();
    PublishTestState(lch2sys);
}

bool DartHostService::IsTestModeEnabled() const
{
    return test_mode_enabled_;
}

float DartHostService::GetTestTensionLeft() const
{
    return test_tension_left_;
}

float DartHostService::GetTestTensionRight() const
{
    return test_tension_right_;
}

HOST_TEST_ACTION DartHostService::GetTestAction() const
{
    return test_action_;
}

void DartHostService::ClearTestAction()
{
    test_action_ = HOST_TEST_ACTION_NONE;
}

void DartHostService::DisableTestMode()
{
    test_mode_enabled_ = false;
    test_action_ = HOST_TEST_ACTION_NONE;
}

bool DartHostService::EnsureTopicsReady()
{
    if (hostreq_fifo_ == nullptr)
    {
        om_topic_t* hostreq_topic = om_find_topic("hostreq", 0);
        if (hostreq_topic != nullptr)
        {
            hostreq_fifo_ = om_queue_add(hostreq_topic, HOSTREQ_FIFO_LEN);
        }
    }

    if (hosttx_topic_ == nullptr)
    {
        hosttx_topic_ = om_find_topic("hosttx", 0);
    }

    return hostreq_fifo_ != nullptr && hosttx_topic_ != nullptr;
}

void DartHostService::UpdateForceTelemetrySources()
{
    if (sensor_suber_ == nullptr)
    {
        om_topic_t* sensor_topic = om_find_topic("sensor", 0);
        if (sensor_topic != nullptr)
        {
            sensor_suber_ = om_subscribe(sensor_topic);
        }
    }

    if (motorctrl_suber_ == nullptr)
    {
        om_topic_t* motorctrl_topic = om_find_topic("motorctrl", 0);
        if (motorctrl_topic != nullptr)
        {
            motorctrl_suber_ = om_subscribe(motorctrl_topic);
        }
    }

    if (sensor_suber_ != nullptr)
    {
        om_suber_export(sensor_suber_, &latest_sensor_, false);
    }
    if (motorctrl_suber_ != nullptr)
    {
        om_suber_export(motorctrl_suber_, &latest_motorctrl_, false);
    }
}

void DartHostService::PublishForceTelemetry()
{
    if (hosttx_topic_ == nullptr || sensor_suber_ == nullptr || motorctrl_suber_ == nullptr)
    {
        return;
    }

    const ULONG now = tx_time_get();
    if (static_cast<ULONG>(now - last_force_telemetry_tick_) < HOST_FORCE_TELEMETRY_PERIOD_TICKS)
    {
        return;
    }

    last_force_telemetry_tick_ = now;
    HostSendForceTelemetry(hosttx_topic_, force_telemetry_seq_++, latest_motorctrl_, latest_sensor_);
}

void DartHostService::PublishTestState(const msg_launcher2sysctrl_t& lch2sys)
{
    if (hosttx_topic_ == nullptr)
    {
        return;
    }

    const ULONG now = tx_time_get();
    if (static_cast<ULONG>(now - last_test_state_tick_) < HOST_TEST_STATE_PERIOD_TICKS)
    {
        return;
    }

    last_test_state_tick_ = now;
    HostSendTestState(hosttx_topic_, test_state_seq_++, test_mode_enabled_, test_control_allowed_,
                      test_action_, lch2sys, test_tension_left_, test_tension_right_);
}

bool DartHostService::IsResetTestRoundAllowed(const msg_launcher2sysctrl_t& lch2sys) const
{
    switch (static_cast<LAUNCHER_FSM_STATE>(lch2sys.current_state))
    {
        case IDLE:
        case ERROR_STOP:
            return true;

        case PRE_TENSION:
        case PREPARING:
        case READY:
        case FIRING:
        case HAND_CONTROL:
        case LAUNCHER_FSM_STATE_INVALID:
        default:
            return false;
    }
}

void DartHostService::ProcessRequest(const msg_hostreq_t& req, const msg_launcher2sysctrl_t& lch2sys,
                                     bool test_control_allowed, DartLibrary& dart)
{
    if (!req.valid || hosttx_topic_ == nullptr)
    {
        return;
    }

    switch (req.kind)
    {
        case HOST_REQ_HEARTBEAT:
            HostSendHeartbeat(hosttx_topic_, dart, req.seq);
            break;

        case HOST_REQ_GET_DART_TABLE:
            for (uint8_t id = 1; id <= 16; id++)
            {
                HostSendDartParamState(hosttx_topic_, dart, req.seq, id);
            }
            HostSendAck(hosttx_topic_, req.type, req.seq, HOST_ACK_APPLIED, dart.config.config_revision);
            break;

        case HOST_REQ_SET_DART_PARAMS_BATCH:
        {
            if (req.target_type > 1 ||
                req.update_mask == 0 ||
                (req.update_mask & static_cast<uint8_t>(~0x03U)) != 0 ||
                req.dart_count == 0 ||
                req.dart_count > 8)
            {
                HostSendError(hosttx_topic_, req.type, req.seq, HOST_ERR_INVALID_COMMAND, req.update_mask);
                break;
            }

            if ((req.update_mask & 0x01U) != 0 &&
                !HostFloatInRange(req.yaw_offset, HOST_YAW_MIN, HOST_YAW_MAX))
            {
                HostSendError(hosttx_topic_, req.type, req.seq, HOST_ERR_PARAM_OUT_OF_RANGE, 1);
                break;
            }

            if ((req.update_mask & 0x02U) != 0 &&
                !HostFloatInRange(req.tension, HOST_TENSION_MIN, HOST_TENSION_MAX))
            {
                HostSendError(hosttx_topic_, req.type, req.seq, HOST_ERR_PARAM_OUT_OF_RANGE, 2);
                break;
            }

            bool seen[17] = {false};
            bool valid_ids = true;
            bool duplicate = false;
            for (uint8_t i = 0; i < req.dart_count; i++)
            {
                const uint8_t id = req.dart_ids[i];
                if (id < 1 || id > 16)
                {
                    valid_ids = false;
                    break;
                }
                if (seen[id])
                {
                    duplicate = true;
                    break;
                }
                seen[id] = true;
            }

            if (!valid_ids)
            {
                HostSendError(hosttx_topic_, req.type, req.seq, HOST_ERR_PARAM_OUT_OF_RANGE, 3);
                break;
            }
            if (duplicate)
            {
                HostSendError(hosttx_topic_, req.type, req.seq, HOST_ERR_DUPLICATE_DART_ID, 0);
                break;
            }

            for (uint8_t i = 0; i < req.dart_count; i++)
            {
                Dart_Config_t& item = dart.config.dart[req.dart_ids[i]];
                if ((req.update_mask & 0x01U) != 0)
                {
                    item.yaw_offset = req.yaw_offset;
                }
                if ((req.update_mask & 0x02U) != 0)
                {
                    if (req.target_type == 0)
                    {
                        item.tension_kg_base = req.tension;
                    }
                    else
                    {
                        item.tension_kg_outpost = req.tension;
                    }
                }
            }

            dart.config.config_revision++;
            HostSendAck(hosttx_topic_, req.type, req.seq, HOST_ACK_APPLIED, dart.config.config_revision);
            for (uint8_t i = 0; i < req.dart_count; i++)
            {
                HostSendDartParamState(hosttx_topic_, dart, req.seq, req.dart_ids[i]);
            }
            break;
        }

        case HOST_REQ_GET_SEQUENCE:
            HostSendSequenceState(hosttx_topic_, dart, req.seq);
            break;

        case HOST_REQ_SET_SEQUENCE:
            for (uint8_t i = 0; i < 4; i++)
            {
                if (req.sequence[i] < 1 || req.sequence[i] > 16)
                {
                    HostSendError(hosttx_topic_, req.type, req.seq, HOST_ERR_PARAM_OUT_OF_RANGE, i);
                    return;
                }
            }
            for (uint8_t i = 0; i < 4; i++)
            {
                dart.config.sequence[i] = req.sequence[i];
            }
            dart.Update_Current_Dart_Id();
            dart.config.config_revision++;
            HostSendAck(hosttx_topic_, req.type, req.seq, HOST_ACK_APPLIED, dart.config.config_revision);
            HostSendSequenceState(hosttx_topic_, dart, req.seq);
            break;

        case HOST_REQ_GET_PRE_TENSION:
            HostSendPreTensionState(hosttx_topic_, dart, req.seq);
            break;

        case HOST_REQ_SET_PRE_TENSION:
            if (!HostFloatInRange(req.pre_tension, HOST_PRE_TENSION_MIN, HOST_PRE_TENSION_MAX))
            {
                HostSendError(hosttx_topic_, req.type, req.seq, HOST_ERR_PARAM_OUT_OF_RANGE, 0);
                break;
            }
            dart.config.pre_tension_kg = req.pre_tension;
            dart.config.config_revision++;
            HostSendAck(hosttx_topic_, req.type, req.seq, HOST_ACK_APPLIED, dart.config.config_revision);
            HostSendPreTensionState(hosttx_topic_, dart, req.seq);
            break;

        case HOST_REQ_RESET_TEST_ROUND:
            if (!IsResetTestRoundAllowed(lch2sys))
            {
                HostSendError(hosttx_topic_, req.type, req.seq, HOST_ERR_STATE_FORBIDDEN, lch2sys.current_state);
                break;
            }

            dart.runtime.current_shot_number = 1;
            dart.Update_Current_Dart_Id();
            dart.runtime.fired_count_this_open = 0;
            dart.runtime.last_fire_finished = lch2sys.is_fire_finished;
            dart.runtime.autoAim.yaw_ok = false;
            HostSendAck(hosttx_topic_, req.type, req.seq, HOST_ACK_APPLIED, dart.config.config_revision);
            break;

        case HOST_REQ_SET_TEST_MODE:
            if (req.test_mode == HOST_TEST_MODE_DISABLED)
            {
                DisableTestMode();
                HostSendAck(hosttx_topic_, req.type, req.seq, HOST_ACK_APPLIED, dart.config.config_revision);
                break;
            }
            if (req.test_mode != HOST_TEST_MODE_ENABLED)
            {
                HostSendError(hosttx_topic_, req.type, req.seq, HOST_ERR_INVALID_COMMAND, req.test_mode);
                break;
            }
            if (!test_control_allowed)
            {
                HostSendError(hosttx_topic_, req.type, req.seq, HOST_ERR_PERMISSION_DENIED, 0);
                break;
            }
            if (lch2sys.current_state != IDLE && lch2sys.current_state != PRE_TENSION)
            {
                HostSendError(hosttx_topic_, req.type, req.seq, HOST_ERR_STATE_FORBIDDEN, lch2sys.current_state);
                break;
            }
            test_mode_enabled_ = true;
            test_action_ = HOST_TEST_ACTION_NONE;
            HostSendAck(hosttx_topic_, req.type, req.seq, HOST_ACK_APPLIED, dart.config.config_revision);
            break;

        case HOST_REQ_SET_TEST_TENSION:
            if (!test_mode_enabled_)
            {
                HostSendError(hosttx_topic_, req.type, req.seq, HOST_ERR_PERMISSION_DENIED, 0);
                break;
            }
            if (!HostFloatInRange(req.tension_left, HOST_TENSION_MIN, HOST_TENSION_MAX))
            {
                HostSendError(hosttx_topic_, req.type, req.seq, HOST_ERR_PARAM_OUT_OF_RANGE, 1);
                break;
            }
            if (!HostFloatInRange(req.tension_right, HOST_TENSION_MIN, HOST_TENSION_MAX))
            {
                HostSendError(hosttx_topic_, req.type, req.seq, HOST_ERR_PARAM_OUT_OF_RANGE, 2);
                break;
            }
            test_tension_left_ = req.tension_left;
            test_tension_right_ = req.tension_right;
            HostSendAck(hosttx_topic_, req.type, req.seq, HOST_ACK_APPLIED, dart.config.config_revision);
            break;

        case HOST_REQ_TEST_ACTION:
            if (!test_mode_enabled_)
            {
                HostSendError(hosttx_topic_, req.type, req.seq, HOST_ERR_PERMISSION_DENIED, 0);
                break;
            }
            if (req.test_action == HOST_TEST_ACTION_ABORT)
            {
                DisableTestMode();
                HostSendAck(hosttx_topic_, req.type, req.seq, HOST_ACK_APPLIED, dart.config.config_revision);
                break;
            }
            if (req.test_action == HOST_TEST_ACTION_PREPARE)
            {
                if (lch2sys.current_state != IDLE &&
                    lch2sys.current_state != PRE_TENSION &&
                    lch2sys.current_state != READY)
                {
                    HostSendError(hosttx_topic_, req.type, req.seq, HOST_ERR_STATE_FORBIDDEN, lch2sys.current_state);
                    break;
                }
                test_action_ = HOST_TEST_ACTION_PREPARE;
                HostSendAck(hosttx_topic_, req.type, req.seq, HOST_ACK_ACCEPTED, dart.config.config_revision);
                break;
            }
            if (req.test_action == HOST_TEST_ACTION_FIRE)
            {
                if (lch2sys.current_state != READY)
                {
                    HostSendError(hosttx_topic_, req.type, req.seq, HOST_ERR_STATE_FORBIDDEN, lch2sys.current_state);
                    break;
                }
                test_action_ = HOST_TEST_ACTION_FIRE;
                HostSendAck(hosttx_topic_, req.type, req.seq, HOST_ACK_ACCEPTED, dart.config.config_revision);
                break;
            }
            HostSendError(hosttx_topic_, req.type, req.seq, HOST_ERR_INVALID_COMMAND, req.test_action);
            break;

        default:
            HostSendError(hosttx_topic_, req.type, req.seq, HOST_ERR_UNKNOWN_TYPE, req.kind);
            break;
    }
}
