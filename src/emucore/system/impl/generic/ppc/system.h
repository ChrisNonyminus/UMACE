#pragma once

#include <memory>
#include <thread>

#include "emucore/system/common/system.h"
#include "emucore/system/cpu/ppc/powerpc.h"

#include "emucore/system/hw/generic/io/console.h"

namespace Core {
    class GenericPPCSystem : public System {
    private:
        std::thread cpu_thread;
        std::shared_ptr<PowerPC> cpu;
    public:
        GenericPPCSystem(SystemEmulationMode mode, u64 clock_speed, u64 ram_size, u64 rom_size, u64 entry, bool multithread) : System(mode, multithread) {
            u64 address_space_size = 0x1'0000'0000;
            min_irq_level = 0;
            max_irq_level = 32;
            address_space = std::shared_ptr<AddressSpace>(new AddressSpace(address_space_size));
            ram = std::shared_ptr<Memory>(new Memory(ram_size, 0, true));
            rom = std::shared_ptr<Memory>(new Memory(rom_size, 0, true));
            address_space->AddMemoryRegion("mainram", ram, 0);
            address_space->AddMemoryRegion("rom", rom, address_space_size - rom_size);
            cpu = std::shared_ptr<PowerPC>(new PowerPC(this, clock_speed, ram_size));
            InitComponents();
            cpu->GetInterpreter().SetPC(entry);
        }
        void InitComponents() {
            auto serial_device = std::shared_ptr<GenericConsoleDevice>(new GenericConsoleDevice(this));
            u64 start_addr = address_space->GetMemoryRegion("mainram")->GetSize();
            address_space->AddIORegion("generic_uart", serial_device, start_addr, 0x20);
            hardware_components["generic_uart"] = serial_device;
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
            bool ret = true;
            running = true;
            if (multi_threaded) {
                cpu_thread = std::thread([this, steps]() {
                    cpu->Run(steps);
                });
            } else {
                ret = cpu->Run(steps);
            }
            return ret;
        }
        bool Stop() override {
            running = false;
            cpu_thread.join();
            return true;
        }
        int IRQ(int irq) override {
            if (irq_components.contains(irq)) {
                return irq_components[irq]->AcknowledgeInterrupt();
            } else {
                return -1;
            }
        }
    };
}