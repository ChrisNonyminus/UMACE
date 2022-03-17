#include <stdio.h>

#include "ide.h"
#include "emucore/system/common/system.h"

#define OUTPUT_DEVICE_IRQ_ID 0x1

namespace Core {
    GenericIDEDevice::GenericIDEDevice(System* system)
    : IOComponent(system) {
    }
    bool GenericIDEDevice::Reset() {
        return true;
    }
    void GenericIDEDevice::Update() {
        
    }
    int GenericIDEDevice::AcknowledgeInterrupt() {
        return 0xffffffff;
    }
    u8 GenericIDEDevice::Read8(u64 addr) {
        switch (addr) {
            case 0x30:
                return data.status;
        }
        if (addr >= 0x4000 && addr < 0x4200) {
            return data.buffer[addr - 0x4000];
        }
        return 0;
    }
    u16 GenericIDEDevice::Read16(u64 addr) {
        return Read8(addr) | (Read8(addr + 1) << 8);
    }
    u32 GenericIDEDevice::Read32(u64 addr) {
        return Read16(addr) | (Read16(addr + 2) << 16);
    }
    void GenericIDEDevice::Write8(u64 addr, u8 value) {
        switch (addr) {
            default:
                break;
        }
        if (addr >= 0x4000 && addr < 0x4200) {
            data.buffer[addr - 0x4000] = value;
        }
    }
    void GenericIDEDevice::Write16(u64 addr, u16 value) {
        Write8(addr, value & 0xff);
        Write8(addr + 1, (value >> 8) & 0xff);
    }
    void GenericIDEDevice::Write32(u64 addr, u32 value) {
        switch (addr) {
            case 0x0000: {
                data.offset = value;
                return;
            }
            case 0x0010: {
                data.ide_id = value;
                return;
            }
            case 0x0020: {
                data.command = value;
                return;
            }
            default:
                break;
        }
        Write16(addr, value & 0xffff);
        Write16(addr + 2, (value >> 16) & 0xffff);
    }
    float GenericIDEDevice::ReadFloat(u64 addr) {
        return 0.0f;
    }
    void GenericIDEDevice::WriteFloat(u64 addr, float value) {
    }
    double GenericIDEDevice::ReadDouble(u64 addr) {
        return 0.0;
    }
    void GenericIDEDevice::WriteDouble(u64 addr, double value) {
    }
    const char* GenericIDEDevice::ReadCString(u64 addr) {
        return "";
    }
    void GenericIDEDevice::WriteCString(u64 addr, const char* value) {

    }
} // namespace Core
