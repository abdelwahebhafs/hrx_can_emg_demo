#include "can_bus.h"

#include <cmath>
#include <cstring>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

CANBus::CANBus(TPCANHandle channel, TPCANBaudrate baudrate)
    : m_channel(channel), m_baudrate(baudrate) {
}

CANBus::~CANBus() {
    shutdown();
}

bool CANBus::initialize() {
    const TPCANStatus status = CAN_Initialize(m_channel, m_baudrate, 0, 0, 0);
    if (status != PCAN_ERROR_OK) {
        char errorText[256]{};
        CAN_GetErrorText(status, 0, errorText);
        std::cerr << "[CAN] init failed: " << errorText << "\n";
        return false;
    }

    m_initialized = true;
    return true;
}

void CANBus::shutdown() {
    if (m_initialized) {
        CAN_Uninitialize(m_channel);
        m_initialized = false;
    }
}

void CANBus::pollMessages(double now_s) {
    if (!m_initialized) return;

    TPCANMsg msg{};
    TPCANTimestamp ts{};

    while (true) {
        const TPCANStatus status = CAN_Read(m_channel, &msg, &ts);
        if (status == PCAN_ERROR_QRCVEMPTY) break;
        if (status != PCAN_ERROR_OK) break;

        pushLog('R', now_s, msg.ID, msg.LEN, msg.DATA);
        decodeFrame(msg.ID, msg.DATA, msg.LEN);
    }
}

RobotData CANBus::getData() const {
    std::lock_guard<std::mutex> lock(m_dataMutex);
    return m_data;
}

void CANBus::sendCommandRaw(int16_t raw_mNm, double now_s) {
    (void)now_s;

    PendingFrame frame{};
    frame.id = CAN_ID_CMD;
    frame.len = 2;
    std::memcpy(frame.data, &raw_mNm, sizeof(raw_mNm));

    std::lock_guard<std::mutex> lock(m_txMutex);
    m_txQueue.push(frame);
}

void CANBus::flushQueue(double now_s) {
    std::lock_guard<std::mutex> lock(m_txMutex);

    while (!m_txQueue.empty()) {
        writeFrame(m_txQueue.front(), now_s);
        m_txQueue.pop();
    }
}

std::vector<CANFrameLog> CANBus::popFrameLogs() {
    std::lock_guard<std::mutex> lock(m_logMutex);
    std::vector<CANFrameLog> out;
    out.swap(m_frameLogs);
    return out;
}

void CANBus::decodeFrame(uint32_t id, const uint8_t* data, uint8_t len) {
    std::lock_guard<std::mutex> lock(m_dataMutex);

    if ((id == CAN_ID_POS1 || id == CAN_ID_POS2) && len >= 4) {
        int32_t raw = 0;
        std::memcpy(&raw, data, 4);
        const float angle = static_cast<float>(raw) * 2.0f * static_cast<float>(M_PI) / 25600.0f;

        if (id == CAN_ID_POS1) {
            m_data.pos1_rad = angle + 1.994f;
            m_data.joint1_connected = true;
        } else {
            m_data.pos2_rad = angle + 1.835f;
            m_data.joint2_connected = true;
        }
    } else if ((id == CAN_ID_VEL1 || id == CAN_ID_VEL2) && len >= 4) {
        int32_t raw = 0;
        std::memcpy(&raw, data, 4);
        const float vel = static_cast<float>(raw) * 2.0f * static_cast<float>(M_PI) / 60.0f;

        if (id == CAN_ID_VEL1) {
            m_data.vel1_rads = vel;
        } else {
            m_data.vel2_rads = vel;
        }
    } else if ((id == CAN_ID_TRQ1 || id == CAN_ID_TRQ2) && len >= 2) {
        int16_t raw = 0;
        std::memcpy(&raw, data, 2);
        const float torque = static_cast<float>(raw) / 1000.0f;

        if (id == CAN_ID_TRQ1) {
            m_data.torque1_Nm = torque;
        } else {
            m_data.torque2_Nm = torque;
        }
    } else if ((id == CAN_ID_ANALOG2_1 || id == CAN_ID_ANALOG2_2) && len >= 2) {
        int16_t raw = 0;
        std::memcpy(&raw, data, 2);

        if (id == CAN_ID_ANALOG2_1) {
            m_data.analog2_1 = raw;
            m_data.analog2_1_seen = true;
        } else {
            m_data.analog2_2 = raw;
            m_data.analog2_2_seen = true;
        }
    }
}

void CANBus::pushLog(char direction, double now_s, uint32_t id, uint8_t len, const uint8_t* data) {
    CANFrameLog entry{};
    entry.direction = direction;
    entry.time_s = now_s;
    entry.id = id;
    entry.len = len;
    if (data && len > 0) std::memcpy(entry.data, data, len);

    std::lock_guard<std::mutex> lock(m_logMutex);
    m_frameLogs.push_back(entry);
}

void CANBus::writeFrame(const PendingFrame& frame, double now_s) {
    if (!m_initialized) return;

    TPCANMsg msg{};
    msg.ID = frame.id;
    msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
    msg.LEN = frame.len;
    std::memcpy(msg.DATA, frame.data, frame.len);

    const TPCANStatus status = CAN_Write(m_channel, &msg);
    if (status != PCAN_ERROR_OK) {
        char errorText[256]{};
        CAN_GetErrorText(status, 0, errorText);
        std::cerr << "[CAN] write failed: " << errorText << "\n";
        return;
    }

    pushLog('T', now_s, frame.id, frame.len, frame.data);
}
