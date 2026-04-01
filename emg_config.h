#pragma once

#include <string>
#include <vector>

inline const std::vector<std::string> EMG_CHANNELS = {
    "Dev1/ai0",
    "Dev1/ai1",
    "Dev1/ai2",
    "Dev1/ai3",
    "Dev1/ai4",
    "Dev1/ai5",
    "Dev1/ai6",
    "Dev1/ai7",
};

constexpr double  EMG_SAMPLE_RATE_HZ   = 2000.0;
constexpr double  EMG_VMIN             = -5.0;
constexpr double  EMG_VMAX             =  5.0;
constexpr int32_t EMG_SAMPLES_PER_READ = 1;
constexpr int32_t EMG_BUFFER_SIZE      = static_cast<int32_t>(EMG_SAMPLE_RATE_HZ * 10.0);
