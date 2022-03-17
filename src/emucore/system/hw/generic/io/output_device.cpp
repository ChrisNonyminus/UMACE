#include "output_device.h"
#include "emucore/system/common/system.h"

#define OUTPUT_DEVICE_IRQ_ID 0x1

namespace Core {
    GenericOutputDevice::GenericOutputDevice(System* system)
    : IOComponent(system) {
    }
    bool GenericOutputDevice::Reset() {
        last_output = time(NULL);
        system->ClearIRQ(OUTPUT_DEVICE_IRQ_ID);
        return true;
    }
    void GenericOutputDevice::Update() {
        if (!ready) {
            if (time(NULL) - last_output >= 1) {
                system->SetIRQ(OUTPUT_DEVICE_IRQ_ID, shared_from_this());
                ready = true;
            }
        }
    }
    int GenericOutputDevice::AcknowledgeInterrupt() {
        return 0xffffffff;
    }
    u8 GenericOutputDevice::Read8(u64 addr) {
        return 0;
    }
    u16 GenericOutputDevice::Read16(u64 addr) {
        return 0;
    }
    u32 GenericOutputDevice::Read32(u64 addr) {
        return 0;
    }
    void GenericOutputDevice::Write8(u64 addr, u8 value) {
        if (ready) {
            printf("%c", value);
            last_output = time(NULL);
            ready = false;
            system->SetIRQ(OUTPUT_DEVICE_IRQ_ID, nullptr);
        }
    }
    void GenericOutputDevice::Write16(u64 addr, u16 value) {
        Write8(addr, value & 0xff);
        Write8(addr + 1, (value >> 8) & 0xff);
    }
    void GenericOutputDevice::Write32(u64 addr, u32 value) {
        Write16(addr, value & 0xffff);
        Write16(addr + 2, (value >> 16) & 0xffff);
    }
    float GenericOutputDevice::ReadFloat(u64 addr) {
        return 0.0f;
    }
    void GenericOutputDevice::WriteFloat(u64 addr, float value) {
    }
    double GenericOutputDevice::ReadDouble(u64 addr) {
        return 0.0;
    }
    void GenericOutputDevice::WriteDouble(u64 addr, double value) {
    }
    const char* GenericOutputDevice::ReadCString(u64 addr) {
        return "";
    }
    void GenericOutputDevice::WriteCString(u64 addr, const char* value) {
        if (ready) {
            printf("%s", value);
            last_output = time(NULL);
            ready = false;
            system->SetIRQ(OUTPUT_DEVICE_IRQ_ID, nullptr);
        }
    }
} // namespace Core
