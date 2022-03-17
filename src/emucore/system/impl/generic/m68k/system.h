#pragma once

#include <memory>
#include <thread>

#include "emucore/system/common/system.h"
#include "emucore/system/cpu/m68k/m68k_defs.h"

#include "emucore/system/hw/generic/io/output_device.h"

#define M68K_24_BIT_ADDRESS_SPACE MB(16)
#define M68K_32_BIT_ADDRESS_SPACE GB(4)

namespace Core {
    class GenericM68KSystem : public System {
    private:
        int cpu_type;
        std::thread cpu_thread;
    public:
        GenericM68KSystem(int cpu_type, SystemEmulationMode mode, u64 ram_size, u64 rom_size, u64 entry, bool multithread) : System(mode, multithread), cpu_type(cpu_type) {
            u64 address_space_size;
            if (cpu_type < M68K_CPU_TYPE_68020) {
                address_space_size = M68K_24_BIT_ADDRESS_SPACE;
            } else {
                address_space_size = M68K_32_BIT_ADDRESS_SPACE;
            }
            min_irq_level = 1;
            max_irq_level = 8;
            address_space = std::shared_ptr<AddressSpace>(new AddressSpace(address_space_size));
            ram = std::shared_ptr<Memory>(new Memory(ram_size));
            rom = std::shared_ptr<Memory>(new Memory(rom_size));
            address_space->AddMemoryRegion("mainram", ram, 0);
            address_space->AddMemoryRegion("rom", rom, address_space_size - rom_size);
            InitMusashi(address_space, this, cpu_type);
            InitComponents();
            m68k_set_reg(M68K_REG_PC, entry);
        }
        void InitComponents() {
            auto output_device = std::shared_ptr<GenericOutputDevice>(new GenericOutputDevice(this));
            u64 start_addr = address_space->GetMemoryRegion("mainram")->GetSize();
            address_space->AddIORegion("generic_uart", output_device, start_addr, 0x1000);
            hardware_components["generic_uart"] = output_device;
        }
        bool LoadRom(const char* path) override {
            FILE* fp = fopen(path, "rb");
            if (fp == nullptr) {
                return false;
            }
            u8* buf;
            fread(&buf, 1, rom->GetSize(), fp);
            fclose(fp);
            rom->WriteBuffer(0, buf, rom->GetSize());
            delete[] buf;
            return true;
        }
        bool Run(u32 steps) override {
            if (multi_threaded) {
                cpu_thread = std::thread([this, steps]() {
                    MusashiRun(steps);
                });
            } else {
                MusashiRun(steps);
            }
            return true;
        }
        bool Stop() override {
            m68k_pulse_halt();
            return true;
        }
        int IRQ(int irq) override {
            if (irq == M68K_IRQ_NONE) {
                return M68K_INT_ACK_SPURIOUS;
            }
            if (irq_components.contains(irq)) {
                return irq_components[irq]->AcknowledgeInterrupt();
            } else {
                return M68K_INT_ACK_SPURIOUS;
            }
        }
    };
} // namespace Core
