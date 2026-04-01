#pragma once

#define NOMINMAX
#include <windows.h>
#include "PCANBasic.h"
#include "robot.h"

#include <cstdint>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

struct CANFrameLog {
    char     direction = 'R'; // R = received, T = transmitted
    double   time_s    = 0.0;
    uint32_t id        = 0;
    uint8_t  len       = 0;
    uint8_t  data[8]{};
};

class CANBus {
public:
    CANBus(TPCANHandle channel = PCAN_USBBUS1, TPCANBaudrate baudrate = PCAN_BAUD_1M);
    ~CANBus();

    bool initialize();
    void shutdown();

    void pollMessages(double now_s);
    RobotData getData() const;

    void sendCommandRaw(int16_t raw_mNm, double now_s);
    void flushQueue(double now_s);

    std::vector<CANFrameLog> popFrameLogs();

private:
    struct PendingFrame {
        uint32_t id = 0;
        uint8_t  len = 0;
        uint8_t  data[8]{};
    };

    TPCANHandle   m_channel;
    TPCANBaudrate m_baudrate;
    bool          m_initialized = false;

    mutable std::mutex m_dataMutex;
    RobotData          m_data{};

    std::mutex              m_txMutex;
    std::queue<PendingFrame> m_txQueue;

    std::mutex                 m_logMutex;
    std::vector<CANFrameLog>   m_frameLogs;

    void decodeFrame(uint32_t id, const uint8_t* data, uint8_t len);
    void pushLog(char direction, double now_s, uint32_t id, uint8_t len, const uint8_t* data);
    void writeFrame(const PendingFrame& frame, double now_s);
};
