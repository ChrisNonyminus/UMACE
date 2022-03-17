#pragma once

#include "emucore/system/common/hw/io.h"

namespace Core {
    class GenericOutputDevice : public IOComponent {
    public:
        GenericOutputDevice(System* system);

        virtual bool Reset() override;
        virtual void Update() override;
        virtual int AcknowledgeInterrupt() override;

        virtual u8 Read8(u64 addr) override;
        virtual u16 Read16(u64 addr) override;
        virtual u32 Read32(u64 addr) override;
        virtual void Write8(u64 addr, u8 value) override;
        virtual void Write16(u64 addr, u16 value) override;
        virtual void Write32(u64 addr, u32 value) override;
        virtual float ReadFloat(u64 addr) override;
        virtual void WriteFloat(u64 addr, float value) override;
        virtual double ReadDouble(u64 addr) override;
        virtual void WriteDouble(u64 addr, double value) override;
        virtual const char* ReadCString(u64 addr) override;
        virtual void WriteCString(u64 addr, const char* value) override;
    private:
        time_t last_output = 0;
        bool ready = false;
    };
}