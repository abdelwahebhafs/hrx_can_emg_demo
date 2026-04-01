# HRX CAN + EMG Demo

This project is a demo version of :

- a **C++ executable** for CAN + NI-DAQ acquisition,
- a **Python / PyQt display** connected through Windows shared memory,
- logging to CSV for offline analysis.

---

## CAN map used in this demo

For robot `i`, where `i` can be `1` or `2`:

- **Analog 2**: `0x18i` with length `2`
- **Position**: `0x28i` with length `4`
- **Velocity**: `0x38i` with length `4`
- **Torque**: `0x48i` with length `2`

So in practice the demo listens to:

- `0x181`, `0x182`
- `0x281`, `0x282`
- `0x381`, `0x382`
- `0x481`, `0x482`

The command frame is kept on:

- **Command**: `0x666`  (check the simulink file for modification)

The payload of `0x666` is a signed `int16` raw command value in mNm.

---

## EMG channels

This demo is configured for:

- `Dev1/ai0`
- `Dev1/ai1`
- `Dev1/ai2`
- `Dev1/ai3`
- `Dev1/ai4`
- `Dev1/ai5`
- `Dev1/ai6`
- `Dev1/ai7`

That gives **8 EMG channels**, indexed from **0 to 7**.

Sampling defaults:
- **2000 Hz**
- differential inputs
- voltage range **[-5 V, +5 V]**

You can change those values in `emg_config.h`.

---

## Files

- `demo_main.cpp` — main application entry point
- `can_bus.h/.cpp` — CAN initialization, decode, transmit queue, raw frame capture
- `robot.h` — CAN IDs, app enums, shared structs
- `shm_bridge.h` — shared memory bridge for the Python UI
- `logger.h` — CAN frame logger + decoded CAN state logger
- `emg_config.h` — NI-DAQ channel and rate configuration
- `emg_reader.h` — NI-DAQ reader
- `emg_logger.h` — EMG CSV logger
- `display.py` — single-screen PyQt display
- `build_cl.bat` — `cl` build script
- `requirements.txt` — Python dependency list

---

## What gets logged

### 1. Raw CAN communication log
`can_frames_YYYYMMDD_HHMMSS.csv`

This file contains one line per CAN frame:
- receive or transmit direction
- timestamp
- CAN ID
- frame length
- raw payload bytes `d0..d7`

This is the file to use when you want to show the actual bus communication in the repo.

### 2. Decoded CAN state log
`can_state_YYYYMMDD_HHMMSS.csv`

This file contains decoded variables:
- robot 1 and robot 2 position
- robot 1 and robot 2 velocity
- robot 1 and robot 2 torque
- Analog2 for robot 1 and robot 2
- current command value sent on `0x666`

### 3. EMG log
`emg_YYYYMMDD_HHMMSS.csv`

This file contains:
- timestamp
- EMG 0 to EMG 7

---

## Thread management

The demo uses **three C++ worker threads**.

### 1. CAN read / decode / publish thread
This thread runs at roughly **1000 Hz** and does four things:
1. reads all available CAN frames from PCAN
2. decodes the frames of interest
3. stores raw CAN frames for logging
4. publishes the latest decoded state to shared memory for the Python display

It also writes the decoded CAN state log while the app is in `RUNNING`.

### 2. CAN send thread
This thread also runs at roughly **1000 Hz**.

Its job is intentionally small:
- read the latest command value requested by the UI
- queue a `0x666` command when the app is `RUNNING`
- flush the CAN transmit queue

Separating transmission from reception keeps the demo structure close to your original code and makes the bus behavior easier to reason about.

### 3. EMG thread
This thread owns the NI-DAQ task:
- creates the task
- starts acquisition
- reads one sample for the 8 channels
- stores the latest EMG values for the UI
- writes them to the EMG CSV file while the app is `RUNNING`

Keeping NI-DAQ inside its own thread avoids blocking the CAN loop.

---

## Shared data between threads

The synchronization strategy is simple:

- **Atomics**
  - app running state
  - requested raw command value
  - EMG connected flag

- **Mutex-protected shared objects**
  - latest decoded CAN data inside `CANBus`
  - CAN transmit queue
  - latest EMG sample array
  - logger file writes

- **Shared memory**
  - C++ writes the latest live state into `HRX_Demo_State`
  - Python writes UI commands into `HRX_Demo_Cmd`

The Python UI never touches the CAN or NI-DAQ drivers directly.

---

## Build in VS Code with your `vcvars64.bat` setup

Before building in VS Code, run this in the terminal (replace the vcvars64.bat path with your actual vcvars64.bat path):

```powershell
cmd /C '"C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" && set' |
  ForEach-Object {
    if ($_ -match '^(.*?)=(.*)$') { Set-Item -Path Env:$($matches[1]) -Value $matches[2] }
  }
```

Then in the same terminal:

```cl /std:c++17 /EHsc /DNOMINMAX demo_main.cpp can_bus.cpp -I. -I"C:\Program Files (x86)\National Instruments\NI-DAQ\DAQmx ANSI C Dev\include" /link /LIBPATH:"C:\Program Files (x86)\National Instruments\Shared\ExternalCompilerSupport\C\lib64\msvc" PCANBasic.lib NIDAQmx.lib /OUT:demo_main.exe

```

That produces:
demo_main.exe


In another terminal, start the UI by running  python display.py

The Python UI expects `demo_main.exe` to already be running because it connects to existing shared memory.

---

## Python setup

Install the UI dependency:

```bat
pip install -r requirements.txt
```

---

## Single-screen display

This demo uses **one single window**.

It shows:
- CAN / EMG connection status
- robot 1 and robot 2 decoded variables
- Analog2 values for both robots
- latest EMG values from channels 0–7
- current raw command value
- a simple robot visualization panel

It also lets you:
- choose the output folder
- set the command raw value
- switch between `IDLE` and `RUNNING`

---

