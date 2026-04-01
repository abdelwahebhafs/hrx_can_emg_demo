#pragma once

#include "emg_config.h"

#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <string>
#include <vector>

class EMGLogger {
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

        m_path = m_folder + "/emg_" + timestamp() + ".csv";
        m_file.open(m_path);
        if (!m_file.is_open()) return false;

        m_file << "step;time_s";
        for (size_t i = 0; i < EMG_CHANNELS.size(); ++i) {
            m_file << ";emg_" << i;
        }
        m_file << "\n";

        m_step = 0;
        return true;
    }

    void log(double time_s, const std::vector<double>& samples) {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_file.is_open()) return;

        ++m_step;
        m_file << m_step << ';' << std::fixed << std::setprecision(6) << time_s;
        for (double value : samples) m_file << ';' << value;
        m_file << '\n';

        if ((m_step % 100) == 0) m_file.flush();
    }

    void close() {
        std::lock_guard<std::mutex> lock(m_mutex);
        closeUnlocked();
    }

    ~EMGLogger() { close(); }

    const std::string& path() const { return m_path; }

private:
    std::string   m_folder;
    std::string   m_path;
    std::ofstream m_file;
    uint64_t      m_step = 0;
    std::mutex    m_mutex;

    static std::string timestamp() {
        std::time_t t = std::time(nullptr);
        char buf[32]{};
        std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&t));
        return std::string(buf);
    }

    void closeUnlocked() {
        if (m_file.is_open()) {
            m_file.flush();
            m_file.close();
        }
    }
};
