#include <iostream>
#include "emucore/system/all_systems.h"
#include "emucore/common/types.h"
#include "emucore/system/common/system.h"

int main(int argc, char* argv[]) {
    std::string system_name = argv[1];
    u64 entry_point = std::stoull(argv[2], nullptr, 16);
    log_file = fopen("log.txt", "w");
    Core::InstallConfigs();
    if (!Core::g_system_configs.contains(system_name)) {
        printf("System %s not found\n", system_name.c_str());
        return 1;
    }

    Core::SystemConfig config = Core::g_system_configs[system_name];

    std::shared_ptr<Core::System> system = Core::CreateSystem(config, entry_point);
    if (!system) {
        printf("System %s not found\n", system_name.c_str());
        return 1;
    }
    while (true) {
        u64 steps;
        std::cin >> steps;
        if (steps > 0) {
            system->Run(steps);
        } else {
            break;
        }
    }
    return 0;
}