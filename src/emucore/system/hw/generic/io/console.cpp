#include <stdio.h>

#include "console.h"
#include "emucore/system/common/system.h"

#define OUTPUT_DEVICE_IRQ_ID 0x1

namespace Core {
    GenericConsoleDevice::GenericConsoleDevice(System* system)
    : IOComponent(system) {
    }
    bool GenericConsoleDevice::Reset() {
        return true;
    }
    void GenericConsoleDevice::Update() {
        
    }
    int GenericConsoleDevice::AcknowledgeInterrupt() {
        return 0xffffffff;
    }
    u8 GenericConsoleDevice::Read8(u64 addr) {
        if (addr == 0) {
            return (char)getchar();
        }
        if (addr == 0x10) {
            Reset();
        }
        return 0;
    }
    u16 GenericConsoleDevice::Read16(u64 addr) {
        return Read8(addr) | (Read8(addr + 1) << 8);
    }
    u32 GenericConsoleDevice::Read32(u64 addr) {
        return Read16(addr) | (Read16(addr + 2) << 16);
    }
    void GenericConsoleDevice::Write8(u64 addr, u8 value) {
        if (addr == 0) {
            putchar(value);
        }
        if (addr == 0x10) {
            Reset();
        }
    }
    void GenericConsoleDevice::Write16(u64 addr, u16 value) {
        Write8(addr, value & 0xff);
        Write8(addr + 1, (value >> 8) & 0xff);
    }
    void GenericConsoleDevice::Write32(u64 addr, u32 value) {
        Write16(addr, value & 0xffff);
        Write16(addr + 2, (value >> 16) & 0xffff);
    }
    float GenericConsoleDevice::ReadFloat(u64 addr) {
        return 0.0f;
    }
    void GenericConsoleDevice::WriteFloat(u64 addr, float value) {
    }
    double GenericConsoleDevice::ReadDouble(u64 addr) {
        return 0.0;
    }
    void GenericConsoleDevice::WriteDouble(u64 addr, double value) {
    }
    const char* GenericConsoleDevice::ReadCString(u64 addr) {
        return "";
    }
    void GenericConsoleDevice::WriteCString(u64 addr, const char* value) {

    }
} // namespace Core
