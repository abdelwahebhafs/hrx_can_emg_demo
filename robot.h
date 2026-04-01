#pragma once

#include <cstdint>
#include <cfloat>

constexpr uint32_t CAN_ID_ANALOG2_1 = 0x181;
constexpr uint32_t CAN_ID_ANALOG2_2 = 0x182;
constexpr uint32_t CAN_ID_POS1      = 0x281;
constexpr uint32_t CAN_ID_POS2      = 0x282;
constexpr uint32_t CAN_ID_VEL1      = 0x381;
constexpr uint32_t CAN_ID_VEL2      = 0x382;
constexpr uint32_t CAN_ID_TRQ1      = 0x481;
constexpr uint32_t CAN_ID_TRQ2      = 0x482;
constexpr uint32_t CAN_ID_CMD       = 0x666;

enum class AppState : uint8_t {
    IDLE    = 0,
    RUNNING = 1,
};

inline const char* stateLabel(AppState s) {
    switch (s) {
        case AppState::IDLE:    return "IDLE";
        case AppState::RUNNING: return "RUNNING";
        default:                return "UNKNOWN";
    }
}

struct RobotData {
    float pos1_rad    = 0.0f;
    float pos2_rad    = 0.0f;
    float vel1_rads   = 0.0f;
    float vel2_rads   = 0.0f;
    float torque1_Nm  = 0.0f;
    float torque2_Nm  = 0.0f;
    int16_t analog2_1 = 0;
    int16_t analog2_2 = 0;
    bool joint1_connected = false;
    bool joint2_connected = false;
    bool analog2_1_seen   = false;
    bool analog2_2_seen   = false;
};

struct SyncTimes {
    double global_time_s = 0.0;
    double global_dt_s   = 0.0;
};

#pragma pack(push, 1)

struct SHM_State {
    float pos1_rad;
    float pos2_rad;
    float vel1_rads;
    float vel2_rads;
    float torque1_Nm;
    float torque2_Nm;
    float analog2_1;
    float analog2_2;
    float cmd_raw_mNm;
    float global_time_s;
    float global_dt_s;

    float emg[8];

    uint8_t app_state;
    uint8_t can_ok;
    uint8_t emg_ok;
    uint8_t joint1_connected;
    uint8_t joint2_connected;
    uint8_t analog2_1_seen;
    uint8_t analog2_2_seen;
    uint8_t reserved0;

    uint32_t counter;
};

struct SHM_Cmd {
    uint8_t  requested_state;
    uint8_t  participant_ready;
    uint8_t  reserved0;
    uint8_t  reserved1;
    int32_t  command_raw_mNm;
    char     session_folder[256];
};

#pragma pack(pop)
