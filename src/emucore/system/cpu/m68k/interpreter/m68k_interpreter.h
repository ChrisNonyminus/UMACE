#pragma once

#include <memory>
#include <bitset>

#include "emucore/common/types.h"
#include "emucore/common/bitfield.h"
#include "emucore/system/common/address_space.h"
#include "../m68k_defs.h"

namespace Core {
    class M68KInterpreter {
    private:
        struct IntegerRegs {
                u32 d[8];
                u32 a[7];
                u32 usp;
                u32 ssp;
                u32 pc;
                u32 vbr;
                u8 sfc : 3;
                u8 dfc : 3;
                u16 sr;
            } integer_regs;
        struct FPURegs { 
                float fp[8]; 
                u16 fpcr; 
                u32 fpsr; 
                u32 fpiar; 
            } fpu_regs;
        std::shared_ptr<AddressSpace> address_space;
        bool is_24bit_address_mode = true; // 24 bit if true 32 bit if false
        bool is_using_fpu = false; // true if fpu is enabled
    public:
        struct InstructionFunctionMap {
            void (Core::M68KInterpreter::*func)(u32&);
            void Execute(M68KInterpreter* interpreter, u32& cycles) const {
                (interpreter->*func)(cycles);
            }
        };
        M68KInterpreter(std::shared_ptr<AddressSpace> address_space, bool is_24bit_address_mode_, bool is_using_fpu_);
        void Reset();
        void Execute(u32 cycles);

        u32 GetPC() { return integer_regs.pc; }
    private:
        std::map<u32, InstructionFunctionMap> instruction_map;
    };
} // namespace Core
