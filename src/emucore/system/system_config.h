#pragma once

#include <map>

#include "emucore/system/common/system.h"

namespace Core {
    struct SystemConfig { // struct defining the setup for what kind of system is being emulated
        std::string system_family;
        std::string system_name;
        System::SystemEmulationMode emulation_mode;
        u64 ram_size;
        u64 rom_size;
        u64 clock_speed;
        u64 cpu_count;
    };

    std::map<std::string, SystemConfig> g_system_configs;

    // pre-defined system configurations
    static SystemConfig g_system_config_ppc_generic = {
        "ppc",
        "ppc-generic",
        System::SystemEmulationMode::HardwareLLE,
        MB(256),
        512,
        MEGA(133),
        1
    };

    static SystemConfig g_system_config_m68k_generic = {
        "m68k",
        "m68k-generic",
        System::SystemEmulationMode::HardwareLLE,
        MB(32),
        512,
        MEGA(40),
        1
    };



    void InstallConfigs() {
        g_system_configs["ppc-generic"] = g_system_config_ppc_generic;
        g_system_configs["m68k-generic"] = g_system_config_m68k_generic;
    }
} // namespace Core
