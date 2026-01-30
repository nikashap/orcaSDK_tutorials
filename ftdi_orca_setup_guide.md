# FTDI D2XX Driver Setup for High-Speed ORCA Motor Communication on macOS

## Overview

This guide documents the process of enabling high-speed serial communication (up to 1M+ baud) between a Mac and an Iris Dynamics ORCA motor using FTDI USB-to-serial adapters. It covers the underlying issues with macOS serial drivers, the solution using FTDI's D2XX direct drivers, and modifications to the `pyorcasdk` library to support non-standard baud rates.

---

## Table of Contents

1. [Background: The Problem](#background-the-problem)
2. [Understanding the Hardware Architecture](#understanding-the-hardware-architecture)
3. [Key Concepts](#key-concepts)
4. [Solution Overview](#solution-overview)
5. [Part 1: Installing FTDI D2XX Drivers](#part-1-installing-ftdi-d2xx-drivers)
6. [Part 2: Reducing FTDI Latency (Quick Win)](#part-2-reducing-ftdi-latency-quick-win)
7. [Part 3: Modifying pyorcasdk for Non-Standard Baud Rates](#part-3-modifying-pyorcasdk-for-non-standard-baud-rates)
8. [Theoretical Communication Timing](#theoretical-communication-timing)
9. [Troubleshooting](#troubleshooting)
10. [References](#references)

---

## Background: The Problem

### Symptoms

When using an FTDI FT232R USB-to-serial adapter to communicate with an ORCA motor on macOS:

1. **High latency (~16ms)**: Round-trip communication times were approximately 16ms regardless of baud rate settings, resulting in only ~63 messages/second instead of the expected 500+ Hz.

2. **Non-standard baud rate errors**: Attempting to open a serial port at baud rates above 230,400 (e.g., 460,800 or 1,000,000) resulted in:
   ```
   RuntimeError: set_option: Invalid argument
   ```

### Root Causes

**Issue 1: FTDI Latency Timer (16ms default)**

The FTDI FT232R chip has an internal latency timer that defaults to **16 milliseconds**. This timer controls how long the chip buffers received data before sending it to the host computer. Even at high baud rates, the chip waits up to 16ms before delivering data to macOS.

**Issue 2: macOS Serial Driver Limitations**

macOS's POSIX serial API (termios) only supports a limited set of "standard" baud rates:
- 300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400

Baud rates like 460,800, 921,600, and 1,000,000 are rejected by the OS serial driver with "Invalid argument."

---

## Understanding the Hardware Architecture

### Physical Setup

```
┌─────────────┐      ┌─────────────────────────────────────┐      ┌─────────────┐
│             │      │       USB-RS422-WE Adapter          │      │             │
│   Mac Mini  │      │  ┌─────────────────────────────┐   │      │ ORCA Motor  │
│             │ USB  │  │      FTDI FT232R Chip       │   │ RS422│             │
│             │◄────►│  │                             │   │◄────►│             │
│             │      │  │  • Converts USB ↔ Serial    │   │      │             │
│             │      │  │  • Has 16ms default latency │   │      │             │
│             │      │  │  • Supports up to 3M baud   │   │      │             │
│             │      │  └─────────────────────────────┘   │      │             │
└─────────────┘      └─────────────────────────────────────┘      └─────────────┘
```

### The FTDI Chip

The FTDI FT232R is a USB-to-serial converter integrated circuit (IC) inside the USB-RS422-WE adapter. It:

1. Receives USB packets from the Mac
2. Converts them to serial data (RS-422)
3. Sends serial data to the motor at the configured baud rate
4. Buffers incoming data from the motor (where the latency timer comes in)
5. Sends buffered data back to the Mac as USB packets

The chip itself supports baud rates up to 3,000,000—the limitation is in how software accesses it.

---

## Key Concepts

### ASIO vs D2XX

**Boost.Asio (used by SerialASIO)**
- A cross-platform C++ library for network and low-level I/O
- Communicates through the **operating system's serial port driver**
- Limited to baud rates supported by the OS

```
SerialASIO → Boost.Asio → macOS Serial Driver → USB Driver → FTDI Chip → Motor
                                    ▲
                                    │
                          Limited to standard baud rates
```

**FTDI D2XX (used by SerialFTDI)**
- FTDI's proprietary library that talks **directly to the FTDI chip**
- Bypasses the operating system's serial driver entirely
- Supports any baud rate up to 3,000,000

```
SerialFTDI → FTDI D2XX Library → FTDI Chip → Motor
                    ▲
                    │
          Supports ANY baud rate!
```

### Serial Communication Timing

For ORCA motors using Modbus RTU with even parity:

**Bits per byte**: 1 (start) + 8 (data) + 1 (parity) + 1 (stop) = **11 bits**

**Motor Command Stream (Function 0x64)**:
- Request: 9 bytes
- Response: 19 bytes
- Total: 28 bytes × 11 bits = **308 bits**

| Baud Rate | Wire Time | + FTDI Latency (1ms) | + Interframe Delay | Approx. Max Rate |
|-----------|-----------|---------------------|-------------------|------------------|
| 19,200 | 16,042 µs | 17,042 µs | 19,042 µs (2ms) | ~52 Hz |
| 230,400 | 1,337 µs | 2,337 µs | 2,537 µs (200µs) | ~400 Hz |
| 1,000,000 | 308 µs | 1,308 µs | 1,308 µs (0µs) | ~765 Hz |

---

## Solution Overview

The solution involves two parts:

1. **Quick Win**: Install D2XX drivers and reduce the FTDI latency timer from 16ms to 1ms. This dramatically improves performance without code changes to pyorcasdk.

2. **Full Solution**: Modify pyorcasdk to use D2XX directly, enabling non-standard baud rates (460,800, 1,000,000, etc.) on macOS.

---

## Part 1: Installing FTDI D2XX Drivers

### Prerequisites

- macOS 12 (Monterey) or later
- Homebrew installed
- An FTDI-based USB-to-serial adapter (e.g., FT232R, FT232H)

### Step 1.1: Download D2XX Drivers

1. Visit the FTDI D2XX download page: https://ftdichip.com/drivers/d2xx-drivers/
2. Download the macOS version (DMG file)
3. Or use curl:

```bash
mkdir -p ~/ftdi
cd ~/ftdi
# Download link may change - verify on FTDI website
curl -L -o d2xx.dmg "https://ftdichip.com/wp-content/uploads/2024/02/D2XX1.4.30.dmg"
```

### Step 1.2: Install the Library

```bash
# Mount the DMG
hdiutil attach d2xx.dmg

# Check contents (path may vary)
ls /Volumes/dmg/

# Copy library files
sudo cp /Volumes/dmg/release/build/libftd2xx.dylib /usr/local/lib/
sudo cp /Volumes/dmg/release/build/libftd2xx.a /usr/local/lib/

# Copy header files
sudo mkdir -p /usr/local/include
sudo cp /Volumes/dmg/release/*.h /usr/local/include/

# Create symlinks (if files have version numbers)
sudo ln -sf /usr/local/lib/libftd2xx.1.4.30.dylib /usr/local/lib/libftd2xx.dylib
sudo ln -sf /usr/local/include/ftd2xx.1.4.30.h /usr/local/include/ftd2xx.h

# Unmount
hdiutil detach /Volumes/dmg
```

### Step 1.3: Verify Installation

```bash
# Check library
ls -la /usr/local/lib/libftd2xx*

# Check headers
ls -la /usr/local/include/ftd2xx.h /usr/local/include/WinTypes.h

# Test loading from Python
python -c "import ctypes; lib = ctypes.CDLL('/usr/local/lib/libftd2xx.dylib'); print('SUCCESS: Library loaded!')"
```

### Step 1.4: Check for Apple FTDI Driver Conflicts (Usually Not Needed)

On modern macOS (12+), Apple's FTDI kext is typically not loaded:

```bash
kextstat | grep -i ftdi
```

If it shows a loaded driver, unload it:

```bash
sudo kextunload -b com.apple.driver.AppleUSBFTDI
```

---

## Part 2: Reducing FTDI Latency (Quick Win)

This section provides immediate performance improvements without modifying pyorcasdk.

### Step 2.1: Set Up Conda Environment

```bash
# Create and activate environment
conda create -n orca_test_env python=3.11
conda activate orca_test_env

# Set up environment variables for FTDI
mkdir -p $CONDA_PREFIX/etc/conda/activate.d
mkdir -p $CONDA_PREFIX/etc/conda/deactivate.d

cat > $CONDA_PREFIX/etc/conda/activate.d/ftdi_env.sh << 'EOF'
#!/bin/bash
export DYLD_LIBRARY_PATH="/usr/local/lib:$DYLD_LIBRARY_PATH"
export FTDI_LIBRARY_PATH="/usr/local/lib/libftd2xx.dylib"
EOF

cat > $CONDA_PREFIX/etc/conda/deactivate.d/ftdi_env.sh << 'EOF'
#!/bin/bash
unset FTDI_LIBRARY_PATH
EOF

chmod +x $CONDA_PREFIX/etc/conda/activate.d/ftdi_env.sh
chmod +x $CONDA_PREFIX/etc/conda/deactivate.d/ftdi_env.sh

# Reactivate to load variables
conda deactivate
conda activate orca_test_env
```

### Step 2.2: Create Latency Configuration Script

Save as `configure_ftdi_latency.py`:

```python
#!/usr/bin/env python
"""
Configure FTDI latency timer for improved serial performance.
Run this BEFORE opening your motor serial connection.
"""

import ctypes
import sys
import time


def configure_ftdi_latency(serial_number: str = None, latency_ms: int = 1) -> bool:
    """
    Set FTDI latency timer to reduce communication delays.
    
    Args:
        serial_number: Device serial (e.g., "ABA76SF6"), or None for first device
        latency_ms: Latency timer value in milliseconds (1-255, default 1)
        
    Returns:
        True if successful
    """
    print(f"Configuring FTDI latency timer to {latency_ms}ms...")
    
    try:
        ftd2xx = ctypes.CDLL("/usr/local/lib/libftd2xx.dylib")
    except OSError as e:
        print(f"ERROR: Could not load FTDI library: {e}")
        return False
    
    # Get number of devices
    num_devices = ctypes.c_ulong()
    status = ftd2xx.FT_CreateDeviceInfoList(ctypes.byref(num_devices))
    
    if status != 0 or num_devices.value == 0:
        print("ERROR: No FTDI devices found")
        return False
    
    print(f"Found {num_devices.value} FTDI device(s)")
    
    # Open device
    handle = ctypes.c_void_p()
    
    if serial_number:
        status = ftd2xx.FT_OpenEx(
            serial_number.encode('utf-8'),
            1,  # FT_OPEN_BY_SERIAL_NUMBER
            ctypes.byref(handle)
        )
    else:
        status = ftd2xx.FT_Open(0, ctypes.byref(handle))
    
    if status != 0:
        print(f"ERROR: Could not open device (status {status})")
        return False
    
    # Get current latency
    current_latency = ctypes.c_ubyte()
    ftd2xx.FT_GetLatencyTimer(handle, ctypes.byref(current_latency))
    print(f"Current latency timer: {current_latency.value}ms")
    
    # Set new latency
    status = ftd2xx.FT_SetLatencyTimer(handle, latency_ms)
    
    if status == 0:
        # Verify
        ftd2xx.FT_GetLatencyTimer(handle, ctypes.byref(current_latency))
        print(f"New latency timer: {current_latency.value}ms")
        print("SUCCESS!")
    else:
        print(f"ERROR: Failed to set latency (status {status})")
    
    ftd2xx.FT_Close(handle)
    return status == 0


if __name__ == "__main__":
    serial = sys.argv[1] if len(sys.argv) > 1 else None
    latency = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    
    configure_ftdi_latency(serial, latency)
```

### Step 2.3: Usage

```bash
# Run before your motor script
python configure_ftdi_latency.py ABA76SF6 1

# Then run your motor script normally
python your_motor_script.py
```

### Expected Results

| Metric | Before (16ms latency) | After (1ms latency) |
|--------|----------------------|---------------------|
| Round-trip latency | ~16,000 µs | ~2,000 µs |
| Max messages/sec | ~63 Hz | ~500 Hz |
| Wire efficiency | ~8% | ~65% |

---

## Part 3: Modifying pyorcasdk for Non-Standard Baud Rates

This section enables baud rates above 230,400 on macOS by creating a new `SerialFTDI` class that uses D2XX directly.

### Step 3.1: Clone pyorcasdk

```bash
cd ~
git clone --recursive https://github.com/IrisDynamics/pyorcasdk.git
cd pyorcasdk
```

### Step 3.2: Create SerialFTDI.h

Create file: `~/pyorcasdk/extern/orcaSDK/src/SerialFTDI.h`

```cpp
#pragma once

#include "serial_interface.h"
#include "error_types.h"
#include <vector>
#include <string>
#include <cstring>
#include <chrono>
#include <thread>

// FTDI D2XX headers
#include <ftd2xx.h>

namespace orcaSDK {

/**
 * @brief SerialInterface implementation using FTDI D2XX library.
 * 
 * This bypasses the OS serial driver to support non-standard baud rates
 * on macOS (and other platforms). Supports any baud rate up to 3,000,000.
 */
class SerialFTDI : public SerialInterface {
public:
    /**
     * @brief Construct SerialFTDI with specified latency timer.
     * @param latency_ms FTDI latency timer in milliseconds (1-255, default 1)
     */
    SerialFTDI(uint8_t latency_ms = 1) : latency_timer_ms(latency_ms) {
        read_buffer.reserve(256);
        send_buffer.reserve(256);
    }

    ~SerialFTDI() {
        close_serial_port();
    }

    OrcaError open_serial_port(int serial_port_number, unsigned int baud) override {
        FT_STATUS status = FT_Open(serial_port_number, &ft_handle);
        if (status != FT_OK) {
            return { static_cast<int>(status), 
                     "Failed to open FTDI device by index: " + std::to_string(status) };
        }
        return configure(baud);
    }

    OrcaError open_serial_port(std::string serial_port_path, unsigned int baud) override {
        std::string serial_number = serial_port_path;
        
        // Extract serial number from macOS path format
        size_t pos = serial_port_path.find("usbserial-");
        if (pos != std::string::npos) {
            serial_number = serial_port_path.substr(pos + 10);
        }
        
        FT_STATUS status = FT_OpenEx(
            const_cast<char*>(serial_number.c_str()),
            FT_OPEN_BY_SERIAL_NUMBER,
            &ft_handle
        );
        
        if (status != FT_OK) {
            status = FT_Open(0, &ft_handle);
            if (status != FT_OK) {
                return { static_cast<int>(status), 
                         "Failed to open FTDI device. Status: " + std::to_string(status) };
            }
        }
        
        return configure(baud);
    }

    void close_serial_port() override {
        if (port_is_open) {
            FT_Close(ft_handle);
            port_is_open = false;
            ft_handle = nullptr;
        }
    }

    void adjust_baud_rate(uint32_t baud_rate_bps) override {
        if (port_is_open) {
            FT_STATUS status = FT_SetBaudRate(ft_handle, baud_rate_bps);
            if (status == FT_OK) {
                current_baud_rate = baud_rate_bps;
            }
        }
    }

    bool ready_to_send() override {
        return port_is_open;
    }

    void send_byte(uint8_t data) override {
        send_buffer.push_back(data);
    }

    void tx_enable(size_t _bytes_to_read) override {
        if (!port_is_open || send_buffer.empty()) return;

        expected_response_length = _bytes_to_read;
        
        FT_Purge(ft_handle, FT_PURGE_RX);
        read_buffer.clear();

        DWORD bytes_written = 0;
        FT_Write(ft_handle, send_buffer.data(), 
                 static_cast<DWORD>(send_buffer.size()), &bytes_written);
        
        send_buffer.clear();
    }

    bool ready_to_receive() override {
        if (!port_is_open) return false;
        if (!read_buffer.empty()) return true;

        DWORD rx_bytes = 0, tx_bytes = 0, event_status = 0;
        FT_GetStatus(ft_handle, &rx_bytes, &tx_bytes, &event_status);
        
        if (rx_bytes > 0) {
            std::vector<uint8_t> temp_buffer(rx_bytes);
            DWORD bytes_read = 0;
            FT_Read(ft_handle, temp_buffer.data(), rx_bytes, &bytes_read);
            
            for (DWORD i = 0; i < bytes_read; i++) {
                read_buffer.push_back(temp_buffer[i]);
            }
        }

        return !read_buffer.empty();
    }

    uint8_t receive_byte() override {
        if (read_buffer.empty()) return 0;
        
        uint8_t byte = read_buffer.front();
        read_buffer.erase(read_buffer.begin());
        return byte;
    }

    OrcaResult<std::vector<uint8_t>> receive_bytes_blocking() override {
        if (!port_is_open) {
            return { {}, { 1, "No serial port open." } };
        }

        std::vector<uint8_t> response;
        
        while (!read_buffer.empty() && response.size() < expected_response_length) {
            response.push_back(read_buffer.front());
            read_buffer.erase(read_buffer.begin());
        }
        
        int timeout_ms = 50;
        if (expected_response_length > 0 && current_baud_rate > 0) {
            int byte_time_ms = static_cast<int>(
                (expected_response_length * 11 * 1000) / current_baud_rate);
            timeout_ms = std::max(timeout_ms, byte_time_ms + 25);
        }

        auto start_time = std::chrono::steady_clock::now();
        auto timeout_duration = std::chrono::milliseconds(timeout_ms);

        while (response.size() < expected_response_length) {
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed > timeout_duration) {
                return { response, { 1, "Read blocking timed out." } };
            }

            DWORD rx_bytes = 0, tx_bytes = 0, event_status = 0;
            FT_GetStatus(ft_handle, &rx_bytes, &tx_bytes, &event_status);
            
            if (rx_bytes > 0) {
                DWORD to_read = std::min(rx_bytes, 
                    static_cast<DWORD>(expected_response_length - response.size()));
                std::vector<uint8_t> temp_buffer(to_read);
                DWORD bytes_read = 0;
                
                FT_Read(ft_handle, temp_buffer.data(), to_read, &bytes_read);
                
                for (DWORD i = 0; i < bytes_read; i++) {
                    response.push_back(temp_buffer[i]);
                }
            } else {
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        }

        return { response, { 0, "" } };
    }

    void flush_and_discard_receive_buffer() override {
        if (port_is_open) {
            FT_Purge(ft_handle, FT_PURGE_RX);
            read_buffer.clear();
        }
    }

    bool is_open() override {
        return port_is_open;
    }

    void set_latency_timer(uint8_t latency_ms) {
        latency_timer_ms = latency_ms;
        if (port_is_open) {
            FT_SetLatencyTimer(ft_handle, latency_timer_ms);
        }
    }

    uint8_t get_latency_timer() const {
        return latency_timer_ms;
    }

private:
    FT_HANDLE ft_handle = nullptr;
    bool port_is_open = false;
    uint8_t latency_timer_ms = 1;
    uint32_t current_baud_rate = 19200;
    size_t expected_response_length = 0;

    std::vector<uint8_t> send_buffer;
    std::vector<uint8_t> read_buffer;

    OrcaError configure(unsigned int baud) {
        FT_STATUS status;

        status = FT_ResetDevice(ft_handle);
        if (status != FT_OK) {
            FT_Close(ft_handle);
            return { static_cast<int>(status), "Failed to reset device" };
        }

        status = FT_SetBaudRate(ft_handle, baud);
        if (status != FT_OK) {
            FT_Close(ft_handle);
            return { static_cast<int>(status), 
                     "Failed to set baud rate: " + std::to_string(baud) };
        }
        current_baud_rate = baud;

        status = FT_SetDataCharacteristics(ft_handle, FT_BITS_8, 
                                           FT_STOP_BITS_1, FT_PARITY_EVEN);
        if (status != FT_OK) {
            FT_Close(ft_handle);
            return { static_cast<int>(status), "Failed to set data characteristics" };
        }

        status = FT_SetFlowControl(ft_handle, FT_FLOW_NONE, 0, 0);
        if (status != FT_OK) {
            FT_Close(ft_handle);
            return { static_cast<int>(status), "Failed to set flow control" };
        }

        status = FT_SetLatencyTimer(ft_handle, latency_timer_ms);
        if (status != FT_OK) {
            FT_Close(ft_handle);
            return { static_cast<int>(status), "Failed to set latency timer" };
        }

        status = FT_SetTimeouts(ft_handle, 100, 100);
        if (status != FT_OK) {
            FT_Close(ft_handle);
            return { static_cast<int>(status), "Failed to set timeouts" };
        }

        FT_SetUSBParameters(ft_handle, 4096, 4096);
        FT_Purge(ft_handle, FT_PURGE_RX | FT_PURGE_TX);

        port_is_open = true;
        return { 0 };
    }
};

} // namespace orcaSDK
```

### Step 3.3: Modify orcaSDK CMakeLists.txt

Replace `~/pyorcasdk/extern/orcaSDK/CMakeLists.txt` with:

```cmake
cmake_minimum_required(VERSION 3.23)

project(orcaSDK VERSION 1.1.0)

option(orcaSDK_USE_FTDI "Enable FTDI D2XX support for non-standard baud rates" ON)

add_library(orcaSDK_core 
    src/actuator.cpp
    src/orca_stream.cpp
    src/standard_modbus_functions.cpp
    src/command_and_confirm.cpp
)
add_library(orcaSDK::core ALIAS orcaSDK_core)

target_sources(orcaSDK_core
    PUBLIC
    FILE_SET publicAPI
    TYPE HEADERS
    FILES
    actuator.h
    src/orca_modes.h
    src/orca_stream_config.h
    src/serial_interface.h
    src/message_priority.h
    src/command_and_confirm.h
    src/clock.h
    src/error_types.h
    src/command_stream_structs.h
    tools/log.h
    tools/log_interface.h
    tools/timer.h
    src/chrono_clock.h
    src/diagnostics_tracker.h
    src/function_code_parameters.h
    src/constants.h
    src/mb_crc.h
    src/message_queue.h
    src/modbus_client.h
    src/standard_modbus_functions.h
    src/orca_stream.h
    src/transaction.h
    src/SerialASIO.h
    src/SerialFTDI.h
)

target_compile_features(orcaSDK_core PUBLIC cxx_std_11)

include(GNUInstallDirs)
option(orcaSDK_VENDOR_ASIO "Should the SDK automatically download asio?" ON)
include(ASIO.cmake)
option(orcaSDK_VENDOR_ORCAAPI "Should the SDK automatically download orcaAPI?" ON)
include(orcaAPI.cmake)

if(orcaSDK_USE_FTDI)
    find_library(FTDI_LIBRARY 
        NAMES ftd2xx libftd2xx
        PATHS /usr/local/lib
        REQUIRED
    )
    
    find_path(FTDI_INCLUDE_DIR 
        NAMES ftd2xx.h
        PATHS /usr/local/include
        REQUIRED
    )
    
    message(STATUS "FTDI D2XX library found: ${FTDI_LIBRARY}")
    message(STATUS "FTDI D2XX headers found: ${FTDI_INCLUDE_DIR}")
    
    target_include_directories(orcaSDK_core PUBLIC ${FTDI_INCLUDE_DIR})
    target_link_libraries(orcaSDK_core PUBLIC ${FTDI_LIBRARY})
    target_compile_definitions(orcaSDK_core PUBLIC ORCASDK_HAS_FTDI=1)
endif()

set_target_properties(orcaSDK_core PROPERTIES
    VERSION ${orcaSDK_VERSION}
    SOVERSION ${orcaSDK_VERSION_MAJOR}
    EXPORT_NAME core
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    orcaSDKConfigVersion.cmake
    VERSION ${orcaSDK_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(TARGETS orcaSDK_core
    EXPORT orcaSDKTargets
    FILE_SET publicAPI
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT orcaSDKTargets 
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/orcaSDK
    NAMESPACE orcaSDK::
)

install(
    FILES "orcaSDKConfig.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/orcaSDK"
)

set(orcaSDK_TEST "false" CACHE BOOL "Should the orcaSDK test targets be included?")
if (orcaSDK_TEST)
    add_subdirectory(tests)
endif()
```

### Step 3.4: Modify Python Bindings

Replace `~/pyorcasdk/bindings/pyorcasdk.cpp` to expose `SerialFTDI`, `Clock`, and `ChronoClock`. Add these includes and class bindings:

```cpp
// Add these includes at the top
#include "src/SerialFTDI.h"
#include "src/chrono_clock.h"

// Add these bindings in PYBIND11_MODULE:

// SerialInterface base class
py::class_<orcaSDK::SerialInterface, std::shared_ptr<orcaSDK::SerialInterface>>(m, "SerialInterface")
    .def("close_serial_port", &orcaSDK::SerialInterface::close_serial_port)
    .def("is_open", &orcaSDK::SerialInterface::is_open);

// SerialFTDI
py::class_<orcaSDK::SerialFTDI, orcaSDK::SerialInterface, 
           std::shared_ptr<orcaSDK::SerialFTDI>>(m, "SerialFTDI")
    .def(py::init<uint8_t>(), py::arg("latency_ms") = 1)
    .def("set_latency_timer", &orcaSDK::SerialFTDI::set_latency_timer)
    .def("get_latency_timer", &orcaSDK::SerialFTDI::get_latency_timer);

// Clock base class
py::class_<orcaSDK::Clock, std::shared_ptr<orcaSDK::Clock>>(m, "Clock")
    .def("get_time_microseconds", &orcaSDK::Clock::get_time_microseconds);

// ChronoClock
py::class_<orcaSDK::ChronoClock, orcaSDK::Clock, 
           std::shared_ptr<orcaSDK::ChronoClock>>(m, "ChronoClock")
    .def(py::init<>());
```

### Step 3.5: Build

```bash
cd ~/pyorcasdk

# Clean previous builds
rm -rf build/ _skbuild/ dist/ *.egg-info/ _pyorcasdk/*.so

# Activate environment
conda activate orca_test_env

# Build
pip install -e . --verbose --no-build-isolation
```

### Step 3.6: Test High-Speed Communication

```python
from pyorcasdk import Actuator, SerialFTDI, ChronoClock, MessagePriority
import time

# Create custom serial interface with 1ms latency
serial = SerialFTDI(latency_ms=1)
clock = ChronoClock()

# Create Actuator with custom serial
motor = Actuator(serial, clock, "HighSpeedMotor", 1)

# Connect at 1M baud!
port = "/dev/cu.usbserial-ABA76SF6"
motor.open_serial_port(port, 1_000_000, 0)

# Test latency
latencies = []
for _ in range(100):
    start = time.perf_counter()
    motor.read_register_blocking(342, MessagePriority.important)
    latencies.append((time.perf_counter() - start) * 1_000_000)

print(f"Average latency: {sum(latencies)/len(latencies):.0f} µs")
print(f"Message rate: {1_000_000 / (sum(latencies)/len(latencies)):.0f} Hz")

motor.close_serial_port()
```

---

## Theoretical Communication Timing

### ORCA Modbus Message Sizes

| Message Type | Request (bytes) | Response (bytes) | Total Bytes | Total Bits (×11) |
|--------------|-----------------|------------------|-------------|------------------|
| Manage High-speed Stream (0x41) | 12 | 12 | 24 | 264 |
| Motor Command Stream (0x64) | 9 | 19 | 28 | 308 |
| Motor Read Stream (0x68) | 7 | 23 | 30 | 330 |
| Motor Write Stream (0x69) | 10 | 18 | 28 | 308 |
| Read Single Register (0x03) | 8 | 7 | 15 | 165 |
| Write Single Register (0x06) | 8 | 8 | 16 | 176 |

### Timing Formula

```
Roundtrip Time = Wire Time + FTDI Latency + Interframe Delay + Processing

Where:
  Wire Time = (Total Bits) / Baud Rate
  FTDI Latency = 1ms (minimum for FT232R) or 0.1ms (FT232H)
  Interframe Delay = Configurable (0-2000+ µs)
  Processing = ~100-200 µs (motor + SDK overhead)
```

### Performance Limits by Hardware

| FTDI Chip | USB Speed | Min Latency | Max Practical Message Rate |
|-----------|-----------|-------------|---------------------------|
| FT232R | 12 Mbps (Full-Speed) | 1 ms | ~500-700 Hz |
| FT232H | 480 Mbps (High-Speed) | 0.1 ms | ~2000+ Hz |

---

## Troubleshooting

### "Library not found" error

```bash
# Verify library exists
ls -la /usr/local/lib/libftd2xx*

# Check library architecture
file /usr/local/lib/libftd2xx.dylib

# Ensure symlinks exist
sudo ln -sf /usr/local/lib/libftd2xx.1.4.30.dylib /usr/local/lib/libftd2xx.dylib
```

### "No FTDI devices found"

```bash
# Check USB connection
system_profiler SPUSBDataType | grep -A 15 -i "FT232"

# Verify device serial number
system_profiler SPUSBDataType | grep -i "Serial Number"
```

### "Device in use" error

```bash
# Check what's using the serial port
lsof | grep usbserial

# Close any applications using the port
```

### Build errors finding ftd2xx.h

```bash
# Create symlink if header has version number
sudo ln -sf /usr/local/include/ftd2xx.1.4.30.h /usr/local/include/ftd2xx.h

# Verify
ls -la /usr/local/include/ftd2xx.h
```

### "set_option: Invalid argument" (without SerialFTDI)

This error occurs when using standard pyorcasdk with non-standard baud rates. You must use the modified pyorcasdk with SerialFTDI as described in Part 3.

---

## References

- [FTDI D2XX Drivers Download](https://ftdichip.com/drivers/d2xx-drivers/)
- [FTDI D2XX Programmer's Guide](https://ftdichip.com/document/programming-guides/)
- [Iris Dynamics pyorcasdk GitHub](https://github.com/IrisDynamics/pyorcasdk)
- [Iris Dynamics orcaSDK GitHub](https://github.com/IrisDynamics/orcaSDK)
- [ORCA Motors Modbus RTU User Guide (UG210912)](https://irisdynamics.com/downloads)
- [Modbus Application Protocol Specification](https://modbus.org/specs.php)

---

## Summary

| Problem | Solution | Result |
|---------|----------|--------|
| 16ms latency | Set FTDI latency timer to 1ms | ~2ms latency |
| Limited to 230,400 baud | Use SerialFTDI with D2XX | Any baud up to 3M |
| ~63 Hz message rate | Both fixes combined | 500-700+ Hz |

---

*Document created: January 2026*
*Tested on: macOS Monterey 12.7.4, Mac Mini (Late 2014), FTDI FT232R USB-RS422-WE adapter*
