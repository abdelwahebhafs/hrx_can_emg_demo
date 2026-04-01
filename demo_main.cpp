#include "can_bus.h"
#include "emg_logger.h"
#include "emg_reader.h"
#include "logger.h"
#include "shm_bridge.h"

#define NOMINMAX
#include <windows.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using Clock = std::chrono::steady_clock;

static std::atomic<bool>    g_running{true};
static std::atomic<bool>    g_emgOk{false};
static std::atomic<int32_t> g_commandRaw{0};
static std::atomic<uint8_t> g_requestedState{static_cast<uint8_t>(AppState::IDLE)};

static std::array<float, 8> g_latestEmg{};
static std::mutex           g_emgMutex;

static Clock::time_point g_startTime;
static std::string       g_sessionFolder;

static double nowSeconds() {
    return std::chrono::duration<double>(Clock::now() - g_startTime).count();
}

static SyncTimes makeTimes(double current_s, double& last_s) {
    SyncTimes times{};
    times.global_time_s = current_s;
    times.global_dt_s = current_s - last_s;
    last_s = current_s;
    return times;
}

static void setLatestEmg(const std::vector<double>& samples) {
    std::lock_guard<std::mutex> lock(g_emgMutex);
    for (size_t i = 0; i < g_latestEmg.size(); ++i) {
        g_latestEmg[i] = (i < samples.size()) ? static_cast<float>(samples[i]) : 0.0f;
    }
}

static std::array<float, 8> getLatestEmg() {
    std::lock_guard<std::mutex> lock(g_emgMutex);
    return g_latestEmg;
}

void canReadThread(CANBus& bus, SHMBridge& shm, CANLogger& logger) {
    constexpr double PERIOD_S = 1.0 / 1000.0;
    double lastLoopS = 0.0;
    double lastLogS = 0.0;
    double lastTimeS = 0.0;

    while (g_running.load()) {
        const double currentS = nowSeconds();
        if ((currentS - lastLoopS) < PERIOD_S) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            continue;
        }
        lastLoopS = currentS;

        const SHM_Cmd cmd = shm.readCmd();
        g_requestedState.store(cmd.requested_state);
        g_commandRaw.store(cmd.command_raw_mNm);

        bus.pollMessages(currentS);

        const RobotData data = bus.getData();
        const auto times = makeTimes(currentS, lastTimeS);
        const auto emg = getLatestEmg();

        logger.logFrames(bus.popFrameLogs());

        const AppState state = static_cast<AppState>(g_requestedState.load());
        if (state == AppState::RUNNING) {
            logger.logState(currentS, state, g_commandRaw.load(), data);
        }

        shm.writeState(
            data,
            state,
            g_commandRaw.load(),
            true,
            g_emgOk.load(),
            times,
            emg.data()
        );
    }
}

void canSendThread(CANBus& bus) {
    constexpr double PERIOD_S = 1.0 / 1000.0;
    double lastLoopS = 0.0;

    while (g_running.load()) {
        const double currentS = nowSeconds();
        if ((currentS - lastLoopS) < PERIOD_S) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            continue;
        }
        lastLoopS = currentS;

        if (static_cast<AppState>(g_requestedState.load()) == AppState::RUNNING) {
            bus.sendCommandRaw(static_cast<int16_t>(g_commandRaw.load()), currentS);
        }
        bus.flushQueue(currentS);
    }
}

void emgThread(EMGLogger& logger) {
    EMGReader reader;
    if (!reader.initialize()) {
        std::cerr << "[EMG] initialization failed, EMG logging disabled.\n";
        g_emgOk.store(false);
        return;
    }

    g_emgOk.store(true);

    std::vector<double> samples;
    while (g_running.load()) {
        if (!reader.readOneSample(samples)) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            continue;
        }

        setLatestEmg(samples);
        if (static_cast<AppState>(g_requestedState.load()) == AppState::RUNNING) {
            logger.log(nowSeconds(), samples);
        }
    }

    reader.shutdown();
    logger.close();
}

int main() {
    CANBus bus(PCAN_USBBUS1, PCAN_BAUD_1M);
    SHMBridge shm;
    CANLogger canLogger;
    EMGLogger emgLogger;

    if (!bus.initialize()) {
        std::cerr << "CAN init failed.\n";
        return 1;
    }
    if (!shm.initialize()) {
        std::cerr << "Shared memory init failed.\n";
        return 1;
    }

    std::cout << "Waiting for Python UI session folder...\n";
    while (true) {
        const SHM_Cmd cmd = shm.readCmd();
        if (cmd.participant_ready) {
            g_sessionFolder = std::string(cmd.session_folder);
            g_commandRaw.store(cmd.command_raw_mNm);
            g_requestedState.store(cmd.requested_state);
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    canLogger.setFolder(g_sessionFolder);
    emgLogger.setFolder(g_sessionFolder);

    if (!canLogger.start()) {
        std::cerr << "Could not open CAN log files in " << g_sessionFolder << "\n";
        return 1;
    }
    if (!emgLogger.start()) {
        std::cerr << "Could not open EMG log file in " << g_sessionFolder << "\n";
        return 1;
    }

    g_startTime = Clock::now();

    std::thread tCanRead(canReadThread, std::ref(bus), std::ref(shm), std::ref(canLogger));
    std::thread tCanSend(canSendThread, std::ref(bus));
    std::thread tEmg(emgThread, std::ref(emgLogger));

    std::cout << "Demo running.\n";
    std::cout << "CAN frames: " << canLogger.framesPath() << "\n";
    std::cout << "CAN state : " << canLogger.statePath() << "\n";
    std::cout << "EMG log   : " << emgLogger.path() << "\n";
    std::cout << "Press Enter to quit.\n";
    std::cin.get();

    g_running.store(false);
    tCanRead.join();
    tCanSend.join();
    tEmg.join();

    canLogger.close();
    emgLogger.close();
    bus.shutdown();
    return 0;
}
