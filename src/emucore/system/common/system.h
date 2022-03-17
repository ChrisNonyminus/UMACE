#pragma once

#include <map>
#include <atomic>
#include <string>
#include <functional>
#include <memory>

#include "emucore/common/types.h"
#include "emucore/system/common/hw/component.h"
#include "emucore/system/common/userspace/component.h"
#include "emucore/system/common/address_space.h"

namespace Core {
    class System : public std::enable_shared_from_this<System> {
    protected:
        std::atomic<bool> running;
        bool multi_threaded = false;
        int min_irq_level = 0;
        int max_irq_level = 0;
        std::shared_ptr<AddressSpace> address_space;
        std::shared_ptr<Memory> ram;
        std::shared_ptr<Memory> rom;
    public:
        enum class SystemEmulationMode : u32 {
            Userspace,   // Emulate an OS application in userspace
            HardwareLLE, // LLE the system hardware
            HardwareHLE, // HLE the system hardware
        };
        System(SystemEmulationMode mode, bool multithread) : running(false), emulation_mode(mode), multi_threaded(multithread) {}
        virtual bool LoadRom(const char* path) = 0;
        virtual bool Run(u32 steps) = 0;
        virtual bool Stop() = 0;
        virtual int IRQ(int irq) = 0;
        bool IsRunning() { return running; }
        std::shared_ptr<AddressSpace>& GetAddressSpace() { return address_space; }
        std::shared_ptr<Memory>& GetMemory() { return ram; }
        std::shared_ptr<Memory>& GetROM() { return rom; }
        SystemEmulationMode emulation_mode;
        std::map<std::string, std::shared_ptr<HardwareComponent>> hardware_components;
        std::map<std::string, std::shared_ptr<HLEComponent>> user_space_components;
        std::map<int, std::shared_ptr<HardwareComponent>> irq_components;
        void SetIRQ(int irq, std::shared_ptr<HardwareComponent> component) {
            if (irq_components[irq] != nullptr ) {
                throw std::runtime_error("IRQ already set");
                return;
            }
            irq_components[irq] = component;
        }
        void ClearIRQ(int irq) {
            irq_components.erase(irq);
        }
        int GetFreeIRQ() {
            for (int i = min_irq_level; i < max_irq_level; i++) {
                if (irq_components.find(i) == irq_components.end()) {
                    return i;
                }
            }
            return -1;
        }
    };
} // namespace Core
