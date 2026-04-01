#pragma once

#include "emg_config.h"
#include "C:\Program Files (x86)\National Instruments\NI-DAQ\DAQmx ANSI C Dev\include\NIDAQmx.h"

#include <iostream>
#include <string>
#include <vector>

class EMGReader {
public:
    EMGReader() = default;
    ~EMGReader() { shutdown(); }

    bool initialize() {
        std::string channelList;
        for (size_t i = 0; i < EMG_CHANNELS.size(); ++i) {
            if (i > 0) channelList += ",";
            channelList += EMG_CHANNELS[i];
        }

        int32 err = 0;

        err = DAQmxCreateTask("EMG_DEMO", &m_task);
        if (handleError(err, "CreateTask")) return false;

        err = DAQmxCreateAIVoltageChan(
            m_task,
            channelList.c_str(),
            "",
            DAQmx_Val_Diff,
            EMG_VMIN,
            EMG_VMAX,
            DAQmx_Val_Volts,
            nullptr
        );
        if (handleError(err, "CreateAIVoltageChan")) return false;

        err = DAQmxCfgSampClkTiming(
            m_task,
            "",
            EMG_SAMPLE_RATE_HZ,
            DAQmx_Val_Rising,
            DAQmx_Val_ContSamps,
            EMG_BUFFER_SIZE
        );
        if (handleError(err, "CfgSampClkTiming")) return false;

        err = DAQmxStartTask(m_task);
        if (handleError(err, "StartTask")) return false;

        m_ok = true;
        std::cout << "[EMG] started with " << EMG_CHANNELS.size()
                  << " channels @ " << EMG_SAMPLE_RATE_HZ << " Hz\n";
        return true;
    }

    void shutdown() {
        if (m_task) {
            DAQmxStopTask(m_task);
            DAQmxClearTask(m_task);
            m_task = nullptr;
        }
        m_ok = false;
    }

    bool readOneSample(std::vector<double>& out) {
        if (!m_ok) return false;

        out.assign(EMG_CHANNELS.size(), 0.0);
        int32 samplesRead = 0;

        const int32 err = DAQmxReadAnalogF64(
            m_task,
            EMG_SAMPLES_PER_READ,
            1.0,
            DAQmx_Val_GroupByScanNumber,
            out.data(),
            static_cast<uInt32>(out.size()),
            &samplesRead,
            nullptr
        );
        if (err < 0) {
            handleError(err, "ReadAnalogF64");
            return false;
        }

        return samplesRead > 0;
    }

    bool isOk() const { return m_ok; }
    size_t channelCount() const { return EMG_CHANNELS.size(); }

private:
    TaskHandle m_task = nullptr;
    bool       m_ok   = false;

    bool handleError(int32 err, const char* where) {
        if (err >= 0) return false;

        char buffer[512]{};
        DAQmxGetExtendedErrorInfo(buffer, sizeof(buffer));
        std::cerr << "[EMG] " << where << " failed: " << buffer << "\n";

        if (m_task) {
            DAQmxClearTask(m_task);
            m_task = nullptr;
        }
        m_ok = false;
        return true;
    }
};
