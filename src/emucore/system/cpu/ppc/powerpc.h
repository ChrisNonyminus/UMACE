#pragma once

#include "emucore/common/types.h"
#include "emucore/system/cpu/ppc/interpreter/ppc_interpreter.h"
#include "emucore/system/common/system.h"


namespace Core {
    #define POWERPC32_ADDRSPACE GB(4)

    #define PPC604_CLOCK_SPEED_MIN MEGA(100)
    #define PPC604_CLOCK_SPEED_MAX MEGA(180)

    class PowerPC {
    public:
        PowerPC(System* _system, u32 clock_speed, u64 mem_size);
        bool Run(u32 steps);
        PowerPCInterpreter& GetInterpreter() { return *interpreter.get(); }
    private:
        std::shared_ptr<PowerPCInterpreter> interpreter;
        std::shared_ptr<AddressSpace> memory_map;
        std::shared_ptr<Memory> ram;
        System* system;
        u32 clock_speed;
    };
}