#pragma once

#include "can_bus.h"
#include "robot.h"

#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>

class CANLogger {
public:
    void setFolder(const std::string& folder) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_folder = folder;
        if (!m_folder.empty()) std::filesystem::create_directories(m_folder);
    }

    bool start() {
        std::lock_guard<std::mutex> lock(m_mutex);
        closeUnlocked();

        if (m_folder.empty()) return false;

        const auto stamp = timestamp();
        m_framesPath = m_folder + "/can_frames_" + stamp + ".csv";
        m_statePath  = m_folder + "/can_state_" + stamp + ".csv";

        m_frames.open(m_framesPath);
        m_state.open(m_statePath);

        if (!m_frames.is_open() || !m_state.is_open()) {
            closeUnlocked();
            return false;
        }

        m_frames << "time_s;direction;id_hex;id_dec;len;d0;d1;d2;d3;d4;d5;d6;d7\n";
        m_state  << "step;time_s;state;cmd_raw_mNm;"
                 << "pos1_rad;pos2_rad;vel1_rads;vel2_rads;torque1_Nm;torque2_Nm;"
                 << "analog2_1_raw;analog2_2_raw;joint1_connected;joint2_connected;"
                 << "analog2_1_seen;analog2_2_seen\n";

        m_step = 0;
        return true;
    }

    void logFrames(const std::vector<CANFrameLog>& frames) {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_frames.is_open()) return;

        for (const auto& frame : frames) {
            m_frames << std::fixed << std::setprecision(6)
                     << frame.time_s << ';'
                     << frame.direction << ';'
                     << toHex(frame.id) << ';'
                     << frame.id << ';'
                     << static_cast<int>(frame.len);
            for (int i = 0; i < 8; ++i) {
                m_frames << ';';
                if (i < frame.len) {
                    m_frames << static_cast<int>(frame.data[i]);
                }
            }
            m_frames << '\n';
        }

        if (!frames.empty()) m_frames.flush();
    }

    void logState(double time_s, AppState state, int32_t cmd_raw_mNm, const RobotData& d) {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_state.is_open()) return;

        ++m_step;
        m_state << m_step << ';'
                << std::fixed << std::setprecision(6) << time_s << ';'
                << stateLabel(state) << ';'
                << cmd_raw_mNm << ';'
                << d.pos1_rad << ';'
                << d.pos2_rad << ';'
                << d.vel1_rads << ';'
                << d.vel2_rads << ';'
                << d.torque1_Nm << ';'
                << d.torque2_Nm << ';'
                << d.analog2_1 << ';'
                << d.analog2_2 << ';'
                << static_cast<int>(d.joint1_connected) << ';'
                << static_cast<int>(d.joint2_connected) << ';'
                << static_cast<int>(d.analog2_1_seen) << ';'
                << static_cast<int>(d.analog2_2_seen) << '\n';

        if ((m_step % 50) == 0) m_state.flush();
    }

    void close() {
        std::lock_guard<std::mutex> lock(m_mutex);
        closeUnlocked();
    }

    ~CANLogger() { close(); }

    const std::string& framesPath() const { return m_framesPath; }
    const std::string& statePath() const { return m_statePath; }

private:
    std::string   m_folder;
    std::string   m_framesPath;
    std::string   m_statePath;
    std::ofstream m_frames;
    std::ofstream m_state;
    uint64_t      m_step = 0;
    mutable std::mutex m_mutex;

    static std::string timestamp() {
        std::time_t t = std::time(nullptr);
        char buf[32]{};
        std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&t));
        return std::string(buf);
    }

    static std::string toHex(uint32_t id) {
        std::ostringstream oss;
        oss << "0x" << std::uppercase << std::hex << id;
        return oss.str();
    }

    void closeUnlocked() {
        if (m_frames.is_open()) {
            m_frames.flush();
            m_frames.close();
        }
        if (m_state.is_open()) {
            m_state.flush();
            m_state.close();
        }
    }
};
