#!/usr/bin/env python
"""
Test FTDI D2XX installation and set latency timer.
"""

import ctypes
import sys


def main():
    print("=" * 60)
    print("FTDI D2XX Library Test")
    print("=" * 60)
    
    # Load library
    print("\n1. Loading FTDI D2XX library...")
    try:
        ftd2xx = ctypes.CDLL("/usr/local/lib/libftd2xx.1.4.30.dylib")
        print("   SUCCESS: Library loaded!")
    except OSError as e:
        print(f"   FAILED: {e}")
        sys.exit(1)
    
    # Get number of devices
    print("\n2. Scanning for FTDI devices...")
    num_devices = ctypes.c_ulong()
    status = ftd2xx.FT_CreateDeviceInfoList(ctypes.byref(num_devices))
    
    if status != 0:
        print(f"   FAILED: FT_CreateDeviceInfoList returned {status}")
        sys.exit(1)
    
    print(f"   Found {num_devices.value} FTDI device(s)")
    
    if num_devices.value == 0:
        print("\n   ERROR: No devices found!")
        print("   Your device IS visible to macOS (we saw it in system_profiler)")
        print("   but D2XX can't see it. This might mean a driver conflict.")
        print("\n   Try disconnecting and reconnecting the USB adapter.")
        sys.exit(1)
    
    # Open device and check/set latency
    print("\n3. Opening device and configuring latency...")
    handle = ctypes.c_void_p()
    
    # Try opening by serial number
    serial = b"ABA76SF6"
    status = ftd2xx.FT_OpenEx(serial, 1, ctypes.byref(handle))  # 1 = FT_OPEN_BY_SERIAL_NUMBER
    
    if status != 0:
        print(f"   Could not open by serial number (status {status}), trying by index...")
        status = ftd2xx.FT_Open(0, ctypes.byref(handle))
        
        if status != 0:
            print(f"   FAILED: Could not open device (status {status})")
            sys.exit(1)
    
    print("   Device opened successfully!")
    
    # Get current latency
    latency = ctypes.c_ubyte()
    status = ftd2xx.FT_GetLatencyTimer(handle, ctypes.byref(latency))
    
    if status == 0:
        print(f"   Current latency timer: {latency.value} ms")
    else:
        print(f"   Could not read latency timer (status {status})")
    
    # Set new latency
    new_latency = 1  # 1 millisecond
    print(f"\n4. Setting latency timer to {new_latency} ms...")
    status = ftd2xx.FT_SetLatencyTimer(handle, new_latency)
    
    if status == 0:
        print("   SUCCESS: Latency timer updated!")
        
        # Verify
        status = ftd2xx.FT_GetLatencyTimer(handle, ctypes.byref(latency))
        if status == 0:
            print(f"   Verified new latency: {latency.value} ms")
    else:
        print(f"   FAILED: Could not set latency (status {status})")
    
    # Close device
    ftd2xx.FT_Close(handle)
    print("\n5. Device closed.")
    
    print("\n" + "=" * 60)
    print("TEST COMPLETE")
    print("=" * 60)
    print("\nIf latency was set to 1ms, you can now run your motor script.")
    print("The improvement should be dramatic: ~16ms -> ~2ms latency!")


if __name__ == "__main__":
    main()