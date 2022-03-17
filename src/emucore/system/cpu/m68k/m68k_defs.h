#pragma once

#include <memory>
#include <bitset>

#include "musashi/m68k.h"

#include "emucore/common/types.h"
#include "emucore/common/bitfield.h"
#include "emucore/system/common/address_space.h"
#include "emucore/system/common/system.h"

namespace Core {
// for musashi
    extern std::shared_ptr<AddressSpace> musashi_address_space;
    extern u32 musashi_clock_speed;
    void InitMusashi(std::shared_ptr<AddressSpace> address_space, System* m68k_system_, int cpu_type) ;
    inline void MusashiRun(u32 steps) {
        for (size_t i = 0; i < steps; i++) {
            m68k_execute(musashi_clock_speed);
        }
    }
} // namespace Core
