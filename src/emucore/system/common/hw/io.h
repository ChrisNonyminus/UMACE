#pragma once

#include "component.h"

namespace Core {
    class IOComponent : public HardwareComponent {
    public:
        IOComponent(System* system);
        virtual u8 Read8(u64 addr) = 0;
        virtual u16 Read16(u64 addr) = 0;
        virtual u32 Read32(u64 addr) = 0;
        virtual void Write8(u64 addr, u8 value) = 0;
        virtual void Write16(u64 addr, u16 value) = 0;
        virtual void Write32(u64 addr, u32 value) = 0;
        virtual float ReadFloat(u64 addr) = 0;
        virtual void WriteFloat(u64 addr, float value) = 0;
        virtual double ReadDouble(u64 addr) = 0;
        virtual void WriteDouble(u64 addr, double value) = 0;
        virtual const char* ReadCString(u64 addr) = 0;
        virtual void WriteCString(u64 addr, const char* value) = 0;

        u64 GetStartAddress() { return start_addr; }
        void SetStartAddress(u64 addr) { start_addr = addr; }
        u64 GetSize() { return mapped_size; }
        void SetSize(u64 size) { mapped_size = size; }
    private:
        u64 start_addr;
        u64 mapped_size;
    };
} // namespace Core
