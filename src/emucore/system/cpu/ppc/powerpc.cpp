#include "powerpc.h"
#include "emucore/system/common/system.h"

namespace Core {
    PowerPC::PowerPC(System* _system, u32 clock_speed, u64 mem_size)
    : system(_system)  {
        this->clock_speed = clock_speed;
        memory_map = system->GetAddressSpace();
        ram = system->GetMemory();
        interpreter = std::shared_ptr<PowerPCInterpreter>(new PowerPCInterpreter(memory_map));
    }
    bool PowerPC::Run(u32 steps) {
        while (steps > 0) {
            if (system->IsRunning() == false) {
                return true;
            }
            if (interpreter->Execute(clock_speed) == false) {
                return false;
            }
            steps--;
        }
        return true;
    }
}