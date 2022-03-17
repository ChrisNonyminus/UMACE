// header file that includes all impl header files
#include "impl/generic/m68k/system.h"
#include "impl/generic/ppc/system.h"
#include "system_config.h"

namespace Core {
    std::shared_ptr<Core::System> CreateSystem_PPC(SystemConfig config, u64 entry) {
        if (config.system_name == "ppc-generic") {
            return std::shared_ptr<Core::GenericPPCSystem>(new GenericPPCSystem(config.emulation_mode, config.clock_speed, config.ram_size, config.rom_size, entry, false));
        } else {
            return nullptr;
        }
    }
    std::shared_ptr<Core::System> CreateSystem_M68K(SystemConfig config, u64 entry) {
        if (config.system_name == "m68k-generic") {
            return std::shared_ptr<Core::GenericM68KSystem>(new GenericM68KSystem(M68K_CPU_TYPE_68030, config.emulation_mode, config.ram_size, config.rom_size, entry, false));
        } else {
            return nullptr;
        }
    }
    std::shared_ptr<Core::System> CreateSystem(SystemConfig config, u64 entry) {
        if (config.system_family == "m68k") {
            return CreateSystem_M68K(config, entry);
        }
        else if (config.system_family == "ppc") {
            return CreateSystem_PPC(config, entry);
        }
        else {
            return nullptr;
        }
    }
}