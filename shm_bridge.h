#pragma once

#include "robot.h"

#include <cstring>
#include <iostream>
#include <windows.h>

class SHMBridge {
public:
    ~SHMBridge() {
        if (m_stateView) UnmapViewOfFile(m_stateView);
        if (m_cmdView)   UnmapViewOfFile(m_cmdView);
        if (m_stateMap)  CloseHandle(m_stateMap);
        if (m_cmdMap)    CloseHandle(m_cmdMap);
    }

    bool initialize() {
        m_stateMap = CreateFileMappingA(
            INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE,
            0, sizeof(SHM_State), "HRX_Demo_State");
        if (!m_stateMap) {
            std::cerr << "[SHM] state map creation failed\n";
            return false;
        }

        m_stateView = reinterpret_cast<SHM_State*>(
            MapViewOfFile(m_stateMap, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(SHM_State)));
        if (!m_stateView) {
            std::cerr << "[SHM] state map view failed\n";
            return false;
        }

        m_cmdMap = CreateFileMappingA(
            INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE,
            0, sizeof(SHM_Cmd), "HRX_Demo_Cmd");
        if (!m_cmdMap) {
            std::cerr << "[SHM] cmd map creation failed\n";
            return false;
        }

        m_cmdView = reinterpret_cast<SHM_Cmd*>(
            MapViewOfFile(m_cmdMap, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(SHM_Cmd)));
        if (!m_cmdView) {
            std::cerr << "[SHM] cmd map view failed\n";
            return false;
        }

        std::memset(m_stateView, 0, sizeof(SHM_State));
        std::memset(m_cmdView, 0, sizeof(SHM_Cmd));
        return true;
    }

    void writeState(const RobotData& d,
                    AppState appState,
                    int32_t cmdRaw,
                    bool canOk,
                    bool emgOk,
                    const SyncTimes& times,
                    const float emg[8]) {
        if (!m_stateView) return;

        m_stateView->pos1_rad = d.pos1_rad;
        m_stateView->pos2_rad = d.pos2_rad;
        m_stateView->vel1_rads = d.vel1_rads;
        m_stateView->vel2_rads = d.vel2_rads;
        m_stateView->torque1_Nm = d.torque1_Nm;
        m_stateView->torque2_Nm = d.torque2_Nm;
        m_stateView->analog2_1 = static_cast<float>(d.analog2_1);
        m_stateView->analog2_2 = static_cast<float>(d.analog2_2);
        m_stateView->cmd_raw_mNm = static_cast<float>(cmdRaw);
        m_stateView->global_time_s = static_cast<float>(times.global_time_s);
        m_stateView->global_dt_s = static_cast<float>(times.global_dt_s);

        for (int i = 0; i < 8; ++i) m_stateView->emg[i] = emg[i];

        m_stateView->app_state = static_cast<uint8_t>(appState);
        m_stateView->can_ok = canOk ? 1 : 0;
        m_stateView->emg_ok = emgOk ? 1 : 0;
        m_stateView->joint1_connected = d.joint1_connected ? 1 : 0;
        m_stateView->joint2_connected = d.joint2_connected ? 1 : 0;
        m_stateView->analog2_1_seen = d.analog2_1_seen ? 1 : 0;
        m_stateView->analog2_2_seen = d.analog2_2_seen ? 1 : 0;
        m_stateView->counter++;
    }

    SHM_Cmd readCmd() const {
        SHM_Cmd cmd{};
        if (m_cmdView) std::memcpy(&cmd, m_cmdView, sizeof(SHM_Cmd));
        return cmd;
    }

private:
    HANDLE     m_stateMap  = nullptr;
    HANDLE     m_cmdMap    = nullptr;
    SHM_State* m_stateView = nullptr;
    SHM_Cmd*   m_cmdView   = nullptr;
};
