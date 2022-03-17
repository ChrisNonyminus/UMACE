#pragma once

#include <memory>
#include <bitset>
#include <map>

#include "emucore/system/common/cpu/clk/isa/ppc/decoder.h"
#include "emucore/common/types.h"
#include "emucore/common/bitfield.h"
#include "emucore/system/common/address_space.h"

// thanks https://github.com/dingusdev/dingusppc/blob/master/cpu/ppc/ppcopcodes.cpp
using namespace InstructionSet::PowerPC;
namespace Core {
    typedef union {
        struct {
            u8 bit0 : 1;
            u8 bit1 : 1;
            u8 bit2 : 1;
            u8 bit3 : 1;
            u8 bit4 : 1;
            u8 bit5 : 1;
            u8 bit6 : 1;
            u8 bit7 : 1;
            u8 bit8 : 1;
            u8 bit9 : 1;
            u8 bit10 : 1;
            u8 bit11 : 1;
            u8 bit12 : 1;
            u8 bit13 : 1;
            u8 bit14 : 1;
            u8 bit15 : 1;
            u8 bit16 : 1;
            u8 bit17 : 1;
            u8 bit18 : 1;
            u8 bit19 : 1;
            u8 bit20 : 1;
            u8 bit21 : 1;
            u8 bit22 : 1;
            u8 bit23 : 1;
            u8 bit24 : 1;
            u8 bit25 : 1;
            u8 bit26 : 1;
            u8 bit27 : 1;
            u8 bit28 : 1;
            u8 bit29 : 1;
            u8 bit30 : 1;
            u8 bit31 : 1;
        };
        u32 whole;
    } word_by_bits;
    static_assert(sizeof(word_by_bits) == 4, "word_by_bits is not 32 bits");
    class PowerPCInterpreter {
    private:
        u32 pc;
        struct Regs {
            u32 gpr[32];
            double fpr[32];
            u32 ctr;
            u32 lr;
            word_by_bits xer;
            struct {
                //u32 value;
                struct {
                    std::bitset<32> bits;
                };
            } fpscr;
            struct {
                //u32 value;
                struct {
                    std::bitset<32> bits;
                };
            } cr;
        } regs;

        std::shared_ptr<AddressSpace> memory;
        bool exception_occured = false;
    public:
        struct InstructionFunctionMap {
            void (Core::PowerPCInterpreter::*func)(u32&, Instruction);
            void Execute(PowerPCInterpreter* interpreter, u32& cycles, Instruction instruction) const {
                (interpreter->*func)(cycles, instruction);
            }
        };
        void SetCarry(bool on) {
            regs.xer.bit2 = on;
        }
        bool GetCarry() {
            return regs.xer.bit2;
        }
        void SetOverflow(bool on) {
            regs.xer.bit1 = on;
        }
        bool GetOverflow() {
            return regs.xer.bit1;
        }
        void SetSummaryOverflow(bool on) {
            regs.xer.bit0 = on;
        }
        bool GetSummaryOverflow() {
            return regs.xer.bit0;
        }
        #define CR_NEGATIVE(CR_FIELD) (regs.cr.bits[CR_FIELD * 4 + 0])
        #define CR_POSITIVE(CR_FIELD) (regs.cr.bits[CR_FIELD * 4 + 1])
        #define CR_ZERO(CR_FIELD) (regs.cr.bits[CR_FIELD * 4 + 2])
        #define CR_SOV(CR_FIELD) (regs.cr.bits[CR_FIELD * 4 + 3])
        void UpdateCR0(u32 value) {
            // negative
            if (value & 0x80000000) {
                regs.cr.bits[0 * 4 + 0] = 1;
            } else {
                regs.cr.bits[0 * 4 + 0] = 0;
            }
            // positive
            if (value & 0x7FFFFFFF) {
                regs.cr.bits[0 * 4 + 1] = 1;
            } else {
                regs.cr.bits[0 * 4 + 1] = 0;
            }
            // zero
            if (value == 0) {
                regs.cr.bits[0 * 4 + 2] = 1;
            } else {
                regs.cr.bits[0 * 4 + 2] = 0;
            }
            // summary overflow
            regs.cr.bits[0 * 4 + 3] = regs.xer.bit0;
        }
        enum FPSCR : u32 {
            RN = 0x3,
            NI = 0x4,
            XE = 0x8,
            ZE = 0x10,
            UE = 0x20,
            OE = 0x40,
            VE = 0x80,
            VXCVI = 0x100,
            VXSQRT = 0x200,
            VXSOFT = 0x400,
            FPRF   = 0x1F000,
            FPCC_FUNAN = 0x10000,
            FPCC_NEG   = 0x8000,
            FPCC_POS   = 0x4000,
            FPCC_ZERO  = 0x2000,
            FPCC_FPRCD = 0x1000,
            FI         = 0x20000,
            FR         = 0x40000,
            VXVC       = 0x80000,
            VXIMZ      = 0x100000,
            VXZDZ      = 0x200000,
            VXIDI      = 0x400000,
            VXISI      = 0x800000,
            VXSNAN     = 0x1000000,
            XX         = 0x2000000,
            ZX         = 0x4000000,
            UX         = 0x8000000,
            OX         = 0x10000000,
            VX         = 0x20000000,
            FEX        = 0x40000000,
            FX         = 0x80000000
        };
        void ConfirmFPResult(bool updatecr1, double value) {
            // negative
            if (value > 0) {
                if (updatecr1) regs.cr.bits[1 * 4 + 0] = 1;
                regs.fpscr.bits |= FPSCR::FPCC_POS;
            } else {
                if (updatecr1) regs.cr.bits[1 * 4 + 0] = 0;
                regs.fpscr.bits &= ~FPSCR::FPCC_POS;
            }
            // positive
            if (value < 0) {
                if (updatecr1) regs.cr.bits[1 * 4 + 1] = 1;
                regs.fpscr.bits |= FPSCR::FPCC_NEG;
            } else {
                if (updatecr1) regs.cr.bits[1 * 4 + 1] = 0;
                regs.fpscr.bits &= ~FPSCR::FPCC_NEG;
            }
            // zero
            if (value == 0) {
                if (updatecr1) regs.cr.bits[1 * 4 + 2] = 1;
                regs.fpscr.bits |= FPSCR::FPCC_ZERO;
            } else {
                if (updatecr1) regs.cr.bits[1 * 4 + 2] = 0;
                regs.fpscr.bits &= ~FPSCR::FPCC_ZERO;
            }
            // summary overflow
            if (isnan(value)) {
                if (updatecr1) regs.cr.bits[1 * 4 + 3] = 1;
                regs.fpscr.bits |= FPSCR::FPCC_FUNAN;
            } else {
                if (updatecr1) regs.cr.bits[1 * 4 + 3] = 0;
                regs.fpscr.bits &= ~FPSCR::FPCC_FUNAN;
            }
        }
        void UpdateCR1() {
            regs.fpscr.bits |= 0xF0000000;
        }
        PowerPCInterpreter(std::shared_ptr<AddressSpace>& memory) : memory(memory) {
            Reset();
            RegisterInstructionFunctions();
        }
        void Reset() {
            for (size_t i = 0; i < 32; i++) {
                regs.gpr[i] = 0;
                regs.fpr[i] = 0;
            }
            regs.ctr = 0;
            regs.lr = 0;
            regs.xer.whole = 0;
            regs.fpscr.bits.reset();
            regs.cr.bits.reset();
            pc = 0xFFF00100;
        }
        void RegisterInstructionFunctions() {
#pragma region branchproc
            // branch processor
            instruction_map[Operation::bx] = { &PowerPCInterpreter::Branch };
            instruction_map[Operation::bcx] = { &PowerPCInterpreter::BranchConditional };
            instruction_map[Operation::bcctrx] = { &PowerPCInterpreter::BranchConditionalToCountRegister };
            instruction_map[Operation::bclrx] = { &PowerPCInterpreter::BranchConditionalToLinkRegister };
            instruction_map[Operation::sc] = { &PowerPCInterpreter::SystemCall };
            instruction_map[Operation::crand] = { &PowerPCInterpreter::ConditionalRegisterAND };
            instruction_map[Operation::cror] = { &PowerPCInterpreter::ConditionalRegisterOR };
            instruction_map[Operation::crxor] = { &PowerPCInterpreter::ConditionalRegisterXOR };
            instruction_map[Operation::crnand] = { &PowerPCInterpreter::ConditionalRegisterNAND };
            instruction_map[Operation::crnor] = { &PowerPCInterpreter::ConditionalRegisterNOR };
            instruction_map[Operation::creqv] = { &PowerPCInterpreter::ConditionalRegisterEQV };
            instruction_map[Operation::crandc] = { &PowerPCInterpreter::ConditionalRegisterANDC };
            instruction_map[Operation::crorc] = { &PowerPCInterpreter::ConditionalRegisterORC };
            instruction_map[Operation::mcrf] = { &PowerPCInterpreter::MoveConditionalRegisterField };
#pragma endregion
#pragma region fxproc
            // fixed point processor
            instruction_map[Operation::lbz] = { &PowerPCInterpreter::LoadByteAndZero };
            instruction_map[Operation::lbzx] = { &PowerPCInterpreter::LoadByteAndZeroIndexed };
            instruction_map[Operation::lbzu] = { &PowerPCInterpreter::LoadByteAndZeroWithUpdate };
            instruction_map[Operation::lbzux] = { &PowerPCInterpreter::LoadByteAndZeroWithUpdateIndexed };
            instruction_map[Operation::lhz] = { &PowerPCInterpreter::LoadHalfWordAndZero };
            instruction_map[Operation::lhzx] = { &PowerPCInterpreter::LoadHalfWordAndZeroIndexed };
            instruction_map[Operation::lhzu] = { &PowerPCInterpreter::LoadHalfWordAndZeroWithUpdate };
            instruction_map[Operation::lhzux] = { &PowerPCInterpreter::LoadHalfWordAndZeroWithUpdateIndexed };
            instruction_map[Operation::lha] = { &PowerPCInterpreter::LoadHalfWordAlgebraic };
            instruction_map[Operation::lhax] = { &PowerPCInterpreter::LoadHalfWordAlgebraicIndexed };
            instruction_map[Operation::lhau] = { &PowerPCInterpreter::LoadHalfWordAlgebraicWithUpdate };
            instruction_map[Operation::lhaux] = { &PowerPCInterpreter::LoadHalfWordAlgebraicWithUpdateIndexed };
            instruction_map[Operation::lwz] = { &PowerPCInterpreter::LoadWordAndZero };
            instruction_map[Operation::lwzx] = { &PowerPCInterpreter::LoadWordAndZeroIndexed };
            instruction_map[Operation::lwzu] = { &PowerPCInterpreter::LoadWordAndZeroWithUpdate };
            instruction_map[Operation::lwzux] = { &PowerPCInterpreter::LoadWordAndZeroWithUpdateIndexed };
            instruction_map[Operation::stb] = { &PowerPCInterpreter::StoreByte };
            instruction_map[Operation::stbx] = { &PowerPCInterpreter::StoreByteIndexed };
            instruction_map[Operation::stbu] = { &PowerPCInterpreter::StoreByteWithUpdate };
            instruction_map[Operation::stbux] = { &PowerPCInterpreter::StoreByteWithUpdateIndexed };
            instruction_map[Operation::sth] = { &PowerPCInterpreter::StoreHalfWord };
            instruction_map[Operation::sthx] = { &PowerPCInterpreter::StoreHalfWordIndexed };
            instruction_map[Operation::sthu] = { &PowerPCInterpreter::StoreHalfWordWithUpdate };
            instruction_map[Operation::sthux] = { &PowerPCInterpreter::StoreHalfWordWithUpdateIndexed };
            instruction_map[Operation::stw] = { &PowerPCInterpreter::StoreWord };
            instruction_map[Operation::stwx] = { &PowerPCInterpreter::StoreWordIndexed };
            instruction_map[Operation::stwu] = { &PowerPCInterpreter::StoreWordWithUpdate };
            instruction_map[Operation::stwux] = { &PowerPCInterpreter::StoreWordWithUpdateIndexed };
            instruction_map[Operation::lhbrx] = { &PowerPCInterpreter::LoadHalfWordByteReverseIndexed };
            instruction_map[Operation::lwbrx] = { &PowerPCInterpreter::LoadWordByteReverseIndexed };
            instruction_map[Operation::sthbrx] = { &PowerPCInterpreter::StoreHalfWordByteReverseIndexed };
            instruction_map[Operation::stwbrx] = { &PowerPCInterpreter::StoreWordByteReverseIndexed };
            instruction_map[Operation::lmw] = { &PowerPCInterpreter::LoadMultipleWords };
            instruction_map[Operation::stmw] = { &PowerPCInterpreter::StoreMultipleWords };
            instruction_map[Operation::lswi] = { &PowerPCInterpreter::LoadStringWordImmediate };
            instruction_map[Operation::lswx] = { &PowerPCInterpreter::LoadStringWordIndexed };
            instruction_map[Operation::stswi] = { &PowerPCInterpreter::StoreStringWordImmediate };
            instruction_map[Operation::stswx] = { &PowerPCInterpreter::StoreStringWordIndexed };
            instruction_map[Operation::addi] = { &PowerPCInterpreter::AddImmediate };
            instruction_map[Operation::addis] = { &PowerPCInterpreter::AddImmediateShifted };
            instruction_map[Operation::addic] = { &PowerPCInterpreter::AddImmediateCarrying };
            instruction_map[Operation::addic_] = { &PowerPCInterpreter::AddImmediateCarryingAndRecord };
            instruction_map[Operation::subfic] = { &PowerPCInterpreter::SubtractFromImmediateCarrying };
            instruction_map[Operation::mulli] = { &PowerPCInterpreter::MultiplyLowImmediate };
            instruction_map[Operation::addx] = { &PowerPCInterpreter::AddRegister };
            instruction_map[Operation::subfx] = { &PowerPCInterpreter::SubtractFromRegister };
            instruction_map[Operation::addcx] = { &PowerPCInterpreter::AddRegisterCarrying };
            instruction_map[Operation::subfcx] = { &PowerPCInterpreter::SubtractFromRegisterCarrying };
            instruction_map[Operation::addex] = { &PowerPCInterpreter::AddExtended };
            instruction_map[Operation::subfex] = { &PowerPCInterpreter::SubtractFromExtended };
            instruction_map[Operation::addmex] = { &PowerPCInterpreter::AddToMinusOneExtended };
            instruction_map[Operation::subfmex] = { &PowerPCInterpreter::SubtractFromMinusOneExtended };
            instruction_map[Operation::addzex] = { &PowerPCInterpreter::AddToZeroExtended };
            instruction_map[Operation::subfzex] = { &PowerPCInterpreter::SubtractFromZeroExtended };
            instruction_map[Operation::mullwx] = { &PowerPCInterpreter::MultiplyLowWord };
            instruction_map[Operation::negx] = { &PowerPCInterpreter::Negate };
            instruction_map[Operation::mulhwx] = { &PowerPCInterpreter::MultiplyHighWord };
            instruction_map[Operation::mulhwux] = { &PowerPCInterpreter::MultiplyHighWordUnsigned };
            instruction_map[Operation::divwx] = { &PowerPCInterpreter::DivideWord };
            instruction_map[Operation::divwux] = { &PowerPCInterpreter::DivideWordUnsigned };
            instruction_map[Operation::cmpi] = { &PowerPCInterpreter::CompareImmediate };
            instruction_map[Operation::cmp] = { &PowerPCInterpreter::Compare };
            instruction_map[Operation::cmpli] = { &PowerPCInterpreter::CompareLogicalImmediate };
            instruction_map[Operation::cmpl] = { &PowerPCInterpreter::CompareLogical };
            instruction_map[Operation::twi] = { &PowerPCInterpreter::TrapWordImmediate };
            instruction_map[Operation::tw] = { &PowerPCInterpreter::TrapWord };
            instruction_map[Operation::andi_] = { &PowerPCInterpreter::AndImmediate };
            instruction_map[Operation::andis_] = { &PowerPCInterpreter::AndImmediateShifted };
            instruction_map[Operation::ori] = { &PowerPCInterpreter::OrImmediate };
            instruction_map[Operation::oris] = { &PowerPCInterpreter::OrImmediateShifted };
            instruction_map[Operation::xori] = { &PowerPCInterpreter::XorImmediate };
            instruction_map[Operation::xoris] = { &PowerPCInterpreter::XorImmediateShifted };
            instruction_map[Operation::andx] = { &PowerPCInterpreter::AndRegister };
            instruction_map[Operation::orx] = { &PowerPCInterpreter::OrRegister };
            instruction_map[Operation::xorx] = { &PowerPCInterpreter::XorRegister };
            instruction_map[Operation::nandx] = { &PowerPCInterpreter::NandRegister };
            instruction_map[Operation::norx] = { &PowerPCInterpreter::NorRegister };
            instruction_map[Operation::eqvx] = { &PowerPCInterpreter::EqvRegister };
            instruction_map[Operation::andcx] = { &PowerPCInterpreter::AndRegisterWithComplement };
            instruction_map[Operation::orcx] = { &PowerPCInterpreter::OrRegisterWithComplement };
            instruction_map[Operation::extsbx] = { &PowerPCInterpreter::ExtendSignByte };
            instruction_map[Operation::extshx] = { &PowerPCInterpreter::ExtendSignHalfWord };
            instruction_map[Operation::cntlzwx] = { &PowerPCInterpreter::CountLeadingZerosWord };
            instruction_map[Operation::rlwinmx] = { &PowerPCInterpreter::RotateLeftWordImmediateThenANDWithMask };
            instruction_map[Operation::rlwnmx] = { &PowerPCInterpreter::RotateLeftWordThenANDWithMask };
            instruction_map[Operation::rlwimix] = { &PowerPCInterpreter::RotateLeftWordImmediateThenMaskInsert };
            instruction_map[Operation::slwx] = { &PowerPCInterpreter::ShiftLeftWord };
            instruction_map[Operation::srwx] = { &PowerPCInterpreter::ShiftRightWord };
            instruction_map[Operation::srawix] = { &PowerPCInterpreter::ShiftRightAlgebraicWordImmediate };
            instruction_map[Operation::srawx] = { &PowerPCInterpreter::ShiftRightAlgebraicWord };
            instruction_map[Operation::mtspr] = { &PowerPCInterpreter::MoveToSpecialPurposeRegister };
            instruction_map[Operation::mfspr] = { &PowerPCInterpreter::MoveFromSpecialPurposeRegister };
            instruction_map[Operation::mtcrf] = { &PowerPCInterpreter::MoveToConditionRegisterFields };
            instruction_map[Operation::mfcr] = { &PowerPCInterpreter::MoveFromConditionRegister };
#pragma endregion
#pragma region fpproc
            // Floating Point Instructions
            instruction_map[Operation::lfs] = { &PowerPCInterpreter::LoadFloatingPointSingle };
            instruction_map[Operation::lfsx] = { &PowerPCInterpreter::LoadFloatingPointSingleIndexed };
            instruction_map[Operation::lfsu] = { &PowerPCInterpreter::LoadFloatingPointSingleWithUpdate };
            instruction_map[Operation::lfsux] = { &PowerPCInterpreter::LoadFloatingPointSingleWithUpdateIndexed };
            instruction_map[Operation::lfd] = { &PowerPCInterpreter::LoadFloatingPointDouble };
            instruction_map[Operation::lfdx] = { &PowerPCInterpreter::LoadFloatingPointDoubleIndexed };
            instruction_map[Operation::lfdu] = { &PowerPCInterpreter::LoadFloatingPointDoubleWithUpdate };
            instruction_map[Operation::lfdux] = { &PowerPCInterpreter::LoadFloatingPointDoubleWithUpdateIndexed };
            instruction_map[Operation::stfs] = { &PowerPCInterpreter::StoreFloatingPointSingle };
            instruction_map[Operation::stfsx] = { &PowerPCInterpreter::StoreFloatingPointSingleIndexed };
            instruction_map[Operation::stfsu] = { &PowerPCInterpreter::StoreFloatingPointSingleWithUpdate };
            instruction_map[Operation::stfsux] = { &PowerPCInterpreter::StoreFloatingPointSingleWithUpdateIndexed };
            instruction_map[Operation::stfd] = { &PowerPCInterpreter::StoreFloatingPointDouble };
            instruction_map[Operation::stfdx] = { &PowerPCInterpreter::StoreFloatingPointDoubleIndexed };
            instruction_map[Operation::stfdu] = { &PowerPCInterpreter::StoreFloatingPointDoubleWithUpdate };
            instruction_map[Operation::stfdux] = { &PowerPCInterpreter::StoreFloatingPointDoubleWithUpdateIndexed };
            instruction_map[Operation::stfiwx] = { &PowerPCInterpreter::StoreFloatingPointAsIntegerWordIndexed };
            instruction_map[Operation::fmrx] = { &PowerPCInterpreter::FloatingPointMoveRegister };
            instruction_map[Operation::fnegx] = { &PowerPCInterpreter::FloatingNegate };
            instruction_map[Operation::fabsx] = { &PowerPCInterpreter::FloatingAbs };
            instruction_map[Operation::fnabsx] = { &PowerPCInterpreter::FloatingNegativeAbs };
            instruction_map[Operation::faddsx] = { &PowerPCInterpreter::FloatingAddSingle };
            instruction_map[Operation::faddx] = { &PowerPCInterpreter::FloatingAddDouble };
            instruction_map[Operation::fsubsx] = { &PowerPCInterpreter::FloatingSubtractSingle };
            instruction_map[Operation::fsubx] = { &PowerPCInterpreter::FloatingSubtractDouble };
            instruction_map[Operation::fmulsx] = { &PowerPCInterpreter::FloatingMultiplySingle };
            instruction_map[Operation::fmulx] = { &PowerPCInterpreter::FloatingMultiplyDouble };
            instruction_map[Operation::fdivsx] = { &PowerPCInterpreter::FloatingDivideSingle };
            instruction_map[Operation::fdivx] = { &PowerPCInterpreter::FloatingDivideDouble };
            instruction_map[Operation::fmaddsx] = { &PowerPCInterpreter::FloatingMultiplyAddSingle };
            instruction_map[Operation::fmaddx] = { &PowerPCInterpreter::FloatingMultiplyAddDouble };
            instruction_map[Operation::fmsubsx] = { &PowerPCInterpreter::FloatingMultiplySubtractSingle };
            instruction_map[Operation::fmsubx] = { &PowerPCInterpreter::FloatingMultiplySubtractDouble };
            instruction_map[Operation::fnmaddsx] = { &PowerPCInterpreter::FloatingNegativeMultiplyAddSingle };
            instruction_map[Operation::fnmaddx] = { &PowerPCInterpreter::FloatingNegativeMultiplyAddDouble };
            instruction_map[Operation::fnmsubsx] = { &PowerPCInterpreter::FloatingNegativeMultiplySubtractSingle };
            instruction_map[Operation::fnmsubx] = { &PowerPCInterpreter::FloatingNegativeMultiplySubtractDouble };
            instruction_map[Operation::frspx] = { &PowerPCInterpreter::FloatingRoundToSinglePrecision };
            instruction_map[Operation::fctiwx] = { &PowerPCInterpreter::FloatingConvertToIntegerWord };
            instruction_map[Operation::fctiwzx] = { &PowerPCInterpreter::FloatingConvertToIntegerWordWithRoundTowardZero };
            instruction_map[Operation::fcmpu] = { &PowerPCInterpreter::FloatingCompareUnordered };
            instruction_map[Operation::fcmpo] = { &PowerPCInterpreter::FloatingCompareOrdered };
            instruction_map[Operation::mffsx] = { &PowerPCInterpreter::MoveFromFPSCR };
            instruction_map[Operation::mcrfs] = { &PowerPCInterpreter::MoveToConditionRegisterFromFPSCR };
            instruction_map[Operation::mtfsfix] = { &PowerPCInterpreter::MoveToFPSCRFieldImmediate };
            instruction_map[Operation::mtfsfx] = { &PowerPCInterpreter::MoveToFPSCRFields };
            instruction_map[Operation::mtfsb0x] = { &PowerPCInterpreter::MoveToFPSCRBit0 };
            instruction_map[Operation::mtfsb1x] = { &PowerPCInterpreter::MoveToFPSCRBit1 };
#pragma endregion
        }
        bool Execute(u32 cycles) {
            while (cycles > 0) {
                u32 value = memory->Read32(pc);
                auto insn = DecipherInstruction(value);
                if (insn.operation == Operation::Undefined) {
                    printf("Unknown opcode...\n... from instruction 0x%08X at pc 0x%08X\n", value, pc);
                    return false;
                }
                auto& map = instruction_map[insn.operation];
                map.Execute(this, cycles, value);
                pc += 4;
                cycles--;
                if (cycles == 0) {
                    return !exception_occured;
                }
            }
            return true;
        }
        Instruction DecipherInstruction(u32 instruction) {
            Decoder decoder(Model::GenericPPC);
            return decoder.decode(instruction);
        }
        
        // Instruction functions

#pragma region BranchProcessorInstructions
        void Branch(u32& cycles, Instruction instruction) {
            u32 target_addr = instruction.aa() ? instruction.li() << 2 : pc + (instruction.li() << 2);
            if (instruction.lk()) {
                regs.lr = pc + 4;
            }
            pc = target_addr;
        }

        void BranchConditional(u32& cycles, Instruction instruction) {
            u32 target_addr = instruction.aa() ? instruction.li() << 2 : pc + (instruction.li() << 2);
            u32 bo = instruction.bo();
            Common::Bitfield<2, 1> bo_b2;
            bo_b2 = bo;
            if (!bo_b2)
                regs.ctr--;
            Common::Bitfield<3, 1> bo_b3;
            bo_b3 = bo;
            Common::Bitfield<0, 1> bo_b0;
            bo_b0 = bo;
            Common::Bitfield<1, 1> bo_b1;
            bo_b1 = bo;
            int ctr_ok =  (bo_b2) || ((regs.ctr != 0) ^ bo_b3);
            int cond_ok = (bo_b0) || ~(regs.cr.bits[instruction.bi()] ^ bo_b1); // todo: & or ==?
            if (ctr_ok && cond_ok) {
                if (instruction.lk()) {
                    regs.lr = pc + 4;
                }
                pc = target_addr;
            }
        }

        void BranchConditionalToLinkRegister(u32&, Instruction instruction) {
            u32 target_addr = regs.lr;
            u32 bo = instruction.bo();
            Common::Bitfield<2, 1> bo_b2;
            bo_b2 = bo;
            if (!bo_b2)
                regs.ctr--;
            Common::Bitfield<3, 1> bo_b3;
            bo_b3 = bo;
            Common::Bitfield<0, 1> bo_b0;
            bo_b0 = bo;
            Common::Bitfield<1, 1> bo_b1;
            bo_b1 = bo;
            int ctr_ok =  (bo_b2) || ((regs.ctr != 0) ^ bo_b3);
            int cond_ok = (bo_b0) || ~(regs.cr.bits[instruction.bi()] ^ bo_b1); // todo: & or ==?
            if (ctr_ok && cond_ok) {
                if (instruction.lk()) {
                    regs.lr = pc + 4;
                }
                pc = target_addr;
            }
        }

        void BranchConditionalToCountRegister(u32&, Instruction instruction) {
            u32 bo = instruction.bo();
            Common::Bitfield<2, 1> bo_b2;
            bo_b2 = bo;
            if (!bo_b2)
                regs.ctr--;
            Common::Bitfield<3, 1> bo_b3;
            bo_b3 = bo;
            Common::Bitfield<0, 1> bo_b0;
            bo_b0 = bo;
            Common::Bitfield<1, 1> bo_b1;
            bo_b1 = bo;
            int cond_ok = (bo_b0) || ~(regs.cr.bits[instruction.bi()] ^ bo_b1); // todo: & or ==?
            if (cond_ok) {
                if (instruction.lk()) {
                    regs.lr = pc + 4;
                }
                pc = regs.ctr;
            }
        }

        void SystemCall(u32& cycles, Instruction instruction) {
            printf("System call!\n");
            cycles = 0; exception_occured = true;
            exception_occured = true;
        }

        void ConditionalRegisterAND(u32& cycles, Instruction instruction) {
            u32 cr_destination = instruction.crbD();
            u32 cr_a = instruction.crbA();
            u32 cr_b = instruction.crbB();
            regs.cr.bits[cr_destination] = regs.cr.bits[cr_a] & regs.cr.bits[cr_b];
        }

        void ConditionalRegisterOR(u32& cycles, Instruction instruction) {
            u32 cr_destination = instruction.crbD();
            u32 cr_a = instruction.crbA();
            u32 cr_b = instruction.crbB();
            regs.cr.bits[cr_destination] = regs.cr.bits[cr_a] | regs.cr.bits[cr_b];
        }

        void ConditionalRegisterXOR(u32& cycles, Instruction instruction) {
            u32 cr_destination = instruction.crbD();
            u32 cr_a = instruction.crbA();
            u32 cr_b = instruction.crbB();
            regs.cr.bits[cr_destination] = regs.cr.bits[cr_a] ^ regs.cr.bits[cr_b];
        }

        void ConditionalRegisterNAND(u32& cycles, Instruction instruction) {
            u32 cr_destination = instruction.crbD();
            u32 cr_a = instruction.crbA();
            u32 cr_b = instruction.crbB();
            regs.cr.bits[cr_destination] = ~(regs.cr.bits[cr_a] & regs.cr.bits[cr_b]);
        }

        void ConditionalRegisterNOR(u32& cycles, Instruction instruction) {
            u32 cr_destination = instruction.crbD();
            u32 cr_a = instruction.crbA();
            u32 cr_b = instruction.crbB();
            regs.cr.bits[cr_destination] = ~(regs.cr.bits[cr_a] | regs.cr.bits[cr_b]);
        }

        void ConditionalRegisterEQV(u32& cycles, Instruction instruction) {
            u32 cr_destination = instruction.crbD();
            u32 cr_a = instruction.crbA();
            u32 cr_b = instruction.crbB();
            regs.cr.bits[cr_destination] = ~(regs.cr.bits[cr_a] ^ regs.cr.bits[cr_b]);
        }

        void ConditionalRegisterANDC(u32& cycles, Instruction instruction) {
            u32 cr_destination = instruction.crbD();
            u32 cr_a = instruction.crbA();
            u32 cr_b = instruction.crbB();
            regs.cr.bits[cr_destination] = (regs.cr.bits[cr_a] & ~regs.cr.bits[cr_b]);
        }

        void ConditionalRegisterORC(u32& cycles, Instruction instruction) {
            u32 cr_destination = instruction.crbD();
            u32 cr_a = instruction.crbA();
            u32 cr_b = instruction.crbB();
            regs.cr.bits[cr_destination] = (regs.cr.bits[cr_a] | ~regs.cr.bits[cr_b]);
        }
        
        void MoveConditionalRegisterField(u32& cycles, Instruction instruction) {
            u32 cr_dest = instruction.crfD();
            u32 cr_src = instruction.crfS();

            for (size_t i = 0; i < 4; i++) {
                regs.cr.bits[cr_dest*4 + i] = regs.cr.bits[cr_src*4 + i];
            }
        }
#pragma endregion
        
#pragma region FixedPointProcessorInstructions
        void LoadByteAndZero(u32& cycles, Instruction instruction) {
            u32 rt = instruction.rD();
            u32 ra = instruction.rA();
            s16 offset = instruction.d();
            u32 address = regs.gpr[ra] + offset;
            regs.gpr[rt] = memory->Read8(address);
        }
        void LoadByteAndZeroIndexed(u32&, Instruction instruction) {
            u32 rt = instruction.rD();
            u32 ra = instruction.rA();
            u32 rb = instruction.rB();
            u32 address = regs.gpr[ra] + regs.gpr[rb];
            regs.gpr[rt] = memory->Read8(address);
        }
        void LoadByteAndZeroWithUpdate(u32& cycles, Instruction instruction) {
            u32 rt = instruction.rD();
            u32 ra = instruction.rA();
            s16 offset = instruction.d();
            u32 address = regs.gpr[ra] + offset;
            if (ra == rt || ra == 0) {
                printf("ERROR: Invalid lbzu instruction\n");
                cycles = 0; exception_occured = true;
                exception_occured = true;
                return;
            }
            regs.gpr[ra] = address;
            regs.gpr[rt] = memory->Read8(address);
        }
        void LoadByteAndZeroWithUpdateIndexed(u32& cycles, Instruction instruction) {
            u32 rt = instruction.rD();
            u32 ra = instruction.rA();
            u32 rb = instruction.rB();
            u32 address = regs.gpr[ra] + regs.gpr[rb];
            if (ra == rt || ra == 0) {
                printf("ERROR: Invalid lbzux instruction\n");
                cycles = 0; exception_occured = true;
                exception_occured = true;
                return;
            }
            regs.gpr[ra] = address;
            regs.gpr[rt] = memory->Read8(address);
        }
        void LoadHalfWordAndZero(u32& cycles, Instruction instruction) {
            u32 rt = instruction.rD();
            u32 ra = instruction.rA();
            s16 offset = instruction.d();
            u32 address = regs.gpr[ra] + offset;
            regs.gpr[rt] = memory->Read16(address);
        }
        void LoadHalfWordAndZeroIndexed(u32&, Instruction instruction) {
            u32 rt = instruction.rD();
            u32 ra = instruction.rA();
            u32 rb = instruction.rB();
            u32 address = regs.gpr[ra] + regs.gpr[rb];
            regs.gpr[rt] = memory->Read16(address);
        }
        void LoadHalfWordAndZeroWithUpdate(u32& cycles, Instruction instruction) {
            u32 rt = instruction.rD();
            u32 ra = instruction.rA();
            s16 offset = instruction.d();
            u32 address = regs.gpr[ra] + offset;
            if (ra == rt || ra == 0) {
                printf("ERROR: Invalid lhzu instruction\n");
                cycles = 0; exception_occured = true;
                exception_occured = true;
                return;
            }
            regs.gpr[ra] = address;
            regs.gpr[rt] = memory->Read16(address);
        }
        void LoadHalfWordAndZeroWithUpdateIndexed(u32& cycles, Instruction instruction) {
            u32 rt = instruction.rD();
            u32 ra = instruction.rA();
            u32 rb = instruction.rB();
            u32 address = regs.gpr[ra] + regs.gpr[rb];
            if (ra == rt || ra == 0) {
                printf("ERROR: Invalid lhzux instruction\n");
                cycles = 0; exception_occured = true;
                exception_occured = true;
                return;
            }
            regs.gpr[ra] = address;
            regs.gpr[rt] = memory->Read16(address);
        }
        void LoadHalfWordAlgebraic(u32& cycles, Instruction instruction) {
            u32 rt = instruction.rD();
            u32 ra = instruction.rA();
            s16 offset = instruction.d();
            u32 address = regs.gpr[ra] + offset;
            regs.gpr[rt] = memory->Read16(address) << 16;
        }
        void LoadHalfWordAlgebraicIndexed(u32& cycles, Instruction instruction) {
            u32 rt = instruction.rD();
            u32 ra = instruction.rA();
            u32 rb = instruction.rB();
            u32 address = regs.gpr[ra] + regs.gpr[rb];
            regs.gpr[rt] = memory->Read16(address) << 16;
        }
        void LoadHalfWordAlgebraicWithUpdate(u32& cycles, Instruction instruction) {
            u32 rt = instruction.rD();
            u32 ra = instruction.rA();
            s16 offset = instruction.d();
            u32 address = regs.gpr[ra] + offset;
            if (ra == rt || ra == 0) {
                printf("ERROR: Invalid lhau instruction\n");
                cycles = 0; exception_occured = true;
                exception_occured = true;
                return;
            }
            regs.gpr[ra] = address;
            regs.gpr[rt] = memory->Read16(address) << 16;
        }
        void LoadHalfWordAlgebraicWithUpdateIndexed(u32& cycles, Instruction instruction) {
            u32 rt = instruction.rD();
            u32 ra = instruction.rA();
            u32 rb = instruction.rB();
            u32 address = regs.gpr[ra] + regs.gpr[rb];
            if (ra == rt || ra == 0) {
                printf("ERROR: Invalid lhaux instruction\n");
                cycles = 0; exception_occured = true;
                exception_occured = true;
                return;
            }
            regs.gpr[ra] = address;
            regs.gpr[rt] = memory->Read16(address) << 16;
        }
        void LoadWordAndZero(u32& cycles, Instruction instruction) {
            u32 rt = instruction.rD();
            u32 ra = instruction.rA();
            s16 offset = instruction.d();
            u32 address = regs.gpr[ra] + offset;
            regs.gpr[rt] = memory->Read32(address);
        }
        void LoadWordAndZeroIndexed(u32&, Instruction instruction) {
            u32 rt = instruction.rD();
            u32 ra = instruction.rA();
            u32 rb = instruction.rB();
            u32 address = regs.gpr[ra] + regs.gpr[rb];
            regs.gpr[rt] = memory->Read32(address);
        }
        void LoadWordAndZeroWithUpdate(u32& cycles, Instruction instruction) {
            u32 rt = instruction.rD();
            u32 ra = instruction.rA();
            s16 offset = instruction.d();
            u32 address = regs.gpr[ra] + offset;
            if (ra == rt || ra == 0) {
                printf("ERROR: Invalid lwzu instruction\n");
                cycles = 0; exception_occured = true;
                exception_occured = true;
                return;
            }
            regs.gpr[ra] = address;
            regs.gpr[rt] = memory->Read32(address);
        }
        void LoadWordAndZeroWithUpdateIndexed(u32& cycles, Instruction instruction) {
            u32 rt = instruction.rD();
            u32 ra = instruction.rA();
            u32 rb = instruction.rB();
            u32 address = regs.gpr[ra] + regs.gpr[rb];
            if (ra == rt || ra == 0) {
                printf("ERROR: Invalid lwzux instruction\n");
                cycles = 0; exception_occured = true;
                exception_occured = true;
                return;
            }
            regs.gpr[ra] = address;
            regs.gpr[rt] = memory->Read32(address);
        }
        void StoreByte(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            s16 offset = instruction.d();
            u32 address = regs.gpr[ra] + offset;
            memory->Write8(address, regs.gpr[rs] & 0xFF);
        }
        void StoreByteIndexed(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            u32 rb = instruction.rB();
            u32 address = regs.gpr[ra] + regs.gpr[rb];
            memory->Write8(address, regs.gpr[rs] & 0xFF);
        }
        void StoreByteWithUpdate(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            s16 offset = instruction.d();
            u32 address = regs.gpr[ra] + offset;
            if (ra == rs || ra == 0) {
                printf("ERROR: Invalid stb instruction\n");
                cycles = 0; exception_occured = true;
                exception_occured = true;
                return;
            }
            regs.gpr[ra] = address;
            memory->Write8(address, regs.gpr[rs] & 0xFF);
        }
        void StoreByteWithUpdateIndexed(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            u32 rb = instruction.rB();
            u32 address = regs.gpr[ra] + regs.gpr[rb];
            if (ra == rs || ra == 0) {
                printf("ERROR: Invalid stbx instruction\n");
                cycles = 0; exception_occured = true;
                exception_occured = true;
                return;
            }
            regs.gpr[ra] = address;
            memory->Write8(address, regs.gpr[rs] & 0xFF);
        }
        void StoreHalfWord(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            s16 offset = instruction.d();
            u32 address = regs.gpr[ra] + offset;
            memory->Write16(address, regs.gpr[rs] & 0xFFFF);
        }
        void StoreHalfWordIndexed(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            u32 rb = instruction.rB();
            u32 address = regs.gpr[ra] + regs.gpr[rb];
            memory->Write16(address, regs.gpr[rs] & 0xFFFF);
        }
        void StoreHalfWordWithUpdate(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            s16 offset = instruction.d();
            u32 address = regs.gpr[ra] + offset;
            if (ra == rs || ra == 0) {
                printf("ERROR: Invalid sth instruction\n");
                cycles = 0; exception_occured = true;
                exception_occured = true;
                return;
            }
            regs.gpr[ra] = address;
            memory->Write16(address, regs.gpr[rs] & 0xFFFF);
        }
        void StoreHalfWordWithUpdateIndexed(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            u32 rb = instruction.rB();
            u32 address = regs.gpr[ra] + regs.gpr[rb];
            if (ra == rs || ra == 0) {
                printf("ERROR: Invalid sthx instruction\n");
                cycles = 0; exception_occured = true;
                exception_occured = true;
                return;
            }
            regs.gpr[ra] = address;
            memory->Write16(address, regs.gpr[rs] & 0xFFFF);
        }
        void StoreWord(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            s16 offset = instruction.d();
            u32 address = regs.gpr[ra] + offset;
            memory->Write32(address, regs.gpr[rs]);
        }
        void StoreWordIndexed(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            u32 rb = instruction.rB();
            u32 address = regs.gpr[ra] + regs.gpr[rb];
            memory->Write32(address, regs.gpr[rs]);
        }
        void StoreWordWithUpdate(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            s16 offset = instruction.d();
            u32 address = regs.gpr[ra] + offset;
            if (ra == rs || ra == 0) {
                printf("ERROR: Invalid stw instruction\n");
                cycles = 0; exception_occured = true;
                exception_occured = true;
                return;
            }
            regs.gpr[ra] = address;
            memory->Write32(address, regs.gpr[rs]);
        }
        void StoreWordWithUpdateIndexed(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            u32 rb = instruction.rB();
            u32 address = regs.gpr[ra] + regs.gpr[rb];
            if (ra == rs || ra == 0) {
                printf("ERROR: Invalid stwx instruction\n");
                cycles = 0; exception_occured = true;
                exception_occured = true;
                return;
            }
            regs.gpr[ra] = address;
            memory->Write32(address, regs.gpr[rs]);
        }
        void LoadHalfWordByteReverseIndexed(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            u32 rb = instruction.rB();
            u32 address = regs.gpr[ra] + regs.gpr[rb];
            regs.gpr[ra] = address;
            regs.gpr[rs] = bswap_16(memory->Read16(address));
        }
        void LoadWordByteReverseIndexed(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            u32 rb = instruction.rB();
            u32 address = regs.gpr[ra] + regs.gpr[rb];
            regs.gpr[ra] = address;
            regs.gpr[rs] = bswap_32(memory->Read32(address));
        }
        void StoreHalfWordByteReverseIndexed(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            u32 rb = instruction.rB();
            u32 address = regs.gpr[ra] + regs.gpr[rb];
            memory->Write16(address, bswap_16(regs.gpr[rs] & 0xFFFF));
        }
        void StoreWordByteReverseIndexed(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            u32 rb = instruction.rB();
            u32 address = regs.gpr[ra] + regs.gpr[rb];
            memory->Write32(address, bswap_32(regs.gpr[rs]));
        }
        void LoadMultipleWords(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            s16 offset = instruction.d();
            u32 address = regs.gpr[ra] + offset;
            for (int i = 0; i < 32 - rs; i++) {
                if (ra == 0 || ra == rs + i) {
                    printf("ERROR: Invalid lmw instruction\n");
                    cycles = 0; exception_occured = true;
                    exception_occured = true;
                    return;
                }
                regs.gpr[rs + i] = memory->Read32(address);
                address += 4;
            }
        }
        void StoreMultipleWords(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            s16 offset = instruction.d();
            u32 address = regs.gpr[ra] + offset;
            for (int i = 0; i < 32 - rs; i++) {
                if (ra == 0 || ra == rs + i) {
                    printf("ERROR: Invalid smw instruction\n");
                    cycles = 0; exception_occured = true;
                    exception_occured = true;
                    return;
                }
                memory->Write32(address, regs.gpr[rs + i]);
                address += 4;
            }
        }
        void LoadStringWordImmediate(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            u32 address = regs.gpr[ra];
            u32 n = instruction.nb();
            if (n == 0)
                n = 32;
            auto nr = ceil(n / 4.0);
            while (n > 0 && nr > 0) {
                if (ra == 0 || ra == rs || rs > 31) {
                    printf("ERROR: Invalid lswi instruction\n");
                    cycles = 0; exception_occured = true;
                    exception_occured = true;
                    return;
                }
                u32 val = 0;
                for (int j = 0; j < 4; j++) {
                    if (n > 0) {
                        val |= memory->Read8(address) << (j * 8);
                        address++;
                        n--;
                    }
                }
                regs.gpr[rs++] = val;
                nr--;
            }
        }
        void LoadStringWordIndexed(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            u32 rb = instruction.rB();
            u32 address = regs.gpr[ra] + regs.gpr[rb];
            u8 n = regs.xer.whole & 0xFF;
            if (n == 0) {
                    printf("ERROR: Invalid lswx instruction\n");
                    cycles = 0; exception_occured = true;
                exception_occured = true;
                    return;
            }
            auto nr = ceil(n / 4.0);
            while (n > 0 && nr > 0) {
                if (ra == 0 || ra == rs || rs > 31 || rb == rs) {
                    printf("ERROR: Invalid lswx instruction\n");
                    cycles = 0; exception_occured = true;
                exception_occured = true;
                    return;
                }
                u32 val = 0;
                for (int j = 0; j < 4; j++) {
                    if (n > 0) {
                        val |= memory->Read8(address) << (j * 8);
                        address++;
                        n--;
                    }
                }
                regs.gpr[rs++] = val;
                nr--;
            }
        }
        void StoreStringWordImmediate(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            u32 address = regs.gpr[ra];
            u32 n = instruction.nb();
            if (n == 0)
                n = 32;
            auto nr = ceil(n / 4.0);
            while (n > 0 && nr > 0) {
                if (ra == 0 || ra == rs || rs > 31) {
                    printf("ERROR: Invalid stswi instruction\n");
                    cycles = 0; exception_occured = true;
                    return;
                }
                u32 val = regs.gpr[rs++];
                memory->Write8(address++, val >> 24);
                val <<= 8;
                n--;
                nr--;
            }
        }
        void StoreStringWordIndexed(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            u32 rb = instruction.rB();
            u32 address = regs.gpr[ra] + regs.gpr[rb];
            u8 n = regs.xer.whole & 0xFF;
            if (n == 0) {
                    printf("ERROR: Invalid stswx instruction\n");
                    cycles = 0; exception_occured = true;
                    return;
            }
            auto nr = ceil(n / 4.0);
            while (n > 0 && nr > 0) {
                if (ra == 0 || ra == rs || rs > 31 || rb == rs) {
                    printf("ERROR: Invalid stswx instruction\n");
                    cycles = 0; exception_occured = true;
                    return;
                }
                u32 val = regs.gpr[rs++];
                memory->Write8(address++, val >> 24);
                val <<= 8;
                n--;
                nr--;
            }
        }
        void AddImmediate(u32& cycles, Instruction instruction) {
            s16 si = instruction.simm();
            u32 ra = instruction.rA();
            u32 rd = instruction.rD();
            regs.gpr[rd] = regs.gpr[ra] + si;
        }
        void AddImmediateShifted(u32& cycles, Instruction instruction) {
            s16 si = instruction.simm();
            u32 ra = instruction.rA();
            u32 rd = instruction.rD();
            regs.gpr[rd] = regs.gpr[ra] + (si << 16);
        }
        void AddRegister(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rb = instruction.rB();
            u32 rd = instruction.rD();
            regs.gpr[rd] = regs.gpr[ra] + regs.gpr[rb];
            
            if (instruction.oe()) {
                SetSummaryOverflow(((regs.gpr[ra] ^ regs.gpr[rd]) & (~regs.gpr[rb] ^ regs.gpr[rd])) & 0x80000000);
            }
            if (instruction.rc() == 1) {
                UpdateCR0(regs.gpr[rd]);
            }
        }
        void SubtractFromRegister(u32&, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rb = instruction.rB();
            u32 rd = instruction.rD();
            regs.gpr[rd] = regs.gpr[rb] - regs.gpr[ra];
            
            if (instruction.oe()) {
                SetSummaryOverflow(((regs.gpr[rb] ^ regs.gpr[rd]) & (regs.gpr[ra] ^ regs.gpr[rd])) & 0x80000000);
            }
            if (instruction.rc() == 1) {
                UpdateCR0(regs.gpr[rd]);
            }
        }
        void AddImmediateCarrying(u32& cycles, Instruction instruction) {
            s16 si = instruction.simm();
            u32 ra = instruction.rA();
            u32 rd = instruction.rD();
            regs.gpr[rd] = regs.gpr[ra] + si;
            SetCarry(regs.gpr[rd] > regs.gpr[ra]); // todo: is this right?
        }
        void AddImmediateCarryingAndRecord(u32&, Instruction instruction) {
            s16 si = instruction.simm();
            u32 ra = instruction.rA();
            u32 rd = instruction.rD();
            regs.gpr[rd] = regs.gpr[ra] + si;
            SetCarry(regs.gpr[rd] > regs.gpr[ra]); // todo: is this right?
            UpdateCR0(regs.gpr[rd]);
        }
        void SubtractFromImmediateCarrying(u32&, Instruction instruction) {
            s16 si = instruction.simm();
            u32 ra = instruction.rA();
            u32 rd = instruction.rD();
            regs.gpr[rd] = (~regs.gpr[ra]) + si - 1;
            SetCarry(regs.gpr[rd] > ~regs.gpr[ra]); // todo: is this right?
        }
        void AddRegisterCarrying(u32&, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rb = instruction.rB();
            u32 rd = instruction.rD();
            regs.gpr[rd] = regs.gpr[ra] + regs.gpr[rb];
            SetCarry(regs.gpr[rd] > regs.gpr[ra]); // todo: is this right?
            
            if (instruction.oe()) {
                SetSummaryOverflow(((regs.gpr[ra] ^ regs.gpr[rd]) & (~regs.gpr[rb] ^ regs.gpr[rd])) & 0x80000000);
            }
            if (instruction.rc() == 1) {
                UpdateCR0(regs.gpr[rd]);
            }
        }
        void SubtractFromRegisterCarrying(u32&, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rb = instruction.rB();
            u32 rd = instruction.rD();
            regs.gpr[rd] = regs.gpr[rb] - regs.gpr[ra];
            SetCarry(regs.gpr[rb] < regs.gpr[ra]); // todo: is this right?
            
            if (instruction.oe()) {
                SetSummaryOverflow(((regs.gpr[rb] ^ regs.gpr[rd]) & (regs.gpr[ra] ^ regs.gpr[rd])) & 0x80000000);
            }
            if (instruction.rc() == 1) {
                UpdateCR0(regs.gpr[rd]);
            }
        }
        void AddExtended(u32&, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rb = instruction.rB();
            u32 rd = instruction.rD();
            regs.gpr[rd] = regs.gpr[ra] + regs.gpr[rb] + (int)GetCarry();
            SetCarry(regs.gpr[rd] > regs.gpr[ra]); // todo: is this right?
            
            if (instruction.oe()) {
                SetSummaryOverflow(((regs.gpr[ra] ^ regs.gpr[rd]) & (~regs.gpr[rb] ^ regs.gpr[rd])) & 0x80000000);
            }
            if (instruction.rc() == 1) {
                UpdateCR0(regs.gpr[rd]);
            }
        }
        void SubtractFromExtended(u32&, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rb = instruction.rB();
            u32 rd = instruction.rD();
            regs.gpr[rd] = ~regs.gpr[ra] + regs.gpr[rb] + (int)GetCarry();
            SetCarry(~regs.gpr[ra] > regs.gpr[rd]); // todo: is this right?
            
            if (instruction.oe()) {
                SetSummaryOverflow(((regs.gpr[rb] ^ regs.gpr[rd]) & (regs.gpr[ra] ^ regs.gpr[rd])) & 0x80000000);
            }
            if (instruction.rc()) {
                UpdateCR0(regs.gpr[rd]);
            }
        }
        void AddToMinusOneExtended(u32&, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rd = instruction.rD();
            regs.gpr[rd] = regs.gpr[ra] + (int)GetCarry() - 1;
            SetCarry((GetCarry() - 1) < -1 || (regs.gpr[rd] < regs.gpr[ra])); // todo: is this right?)

            if (instruction.oe()) {
                SetSummaryOverflow(((regs.gpr[ra] ^ regs.gpr[rd]) & (0 ^ regs.gpr[rd])) & 0x80000000);
            }
            if (instruction.rc()) {
                UpdateCR0(regs.gpr[rd]);
            }
        }
        void SubtractFromMinusOneExtended(u32&, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rd = instruction.rD();
            regs.gpr[rd] = ~regs.gpr[ra] + (int)GetCarry() - 1;
            SetCarry(regs.gpr[rd] > ~regs.gpr[ra]); // todo: is this right?

            if (instruction.oe()) {
                SetSummaryOverflow(((-1 ^ regs.gpr[rd]) & (regs.gpr[ra] ^ regs.gpr[rd])) & 0x80000000);
            }
            if (instruction.rc()) {
                UpdateCR0(regs.gpr[rd]);
            }
        }
        void AddToZeroExtended(u32&, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rd = instruction.rD();
            regs.gpr[rd] = regs.gpr[ra] + (int)GetCarry();
            SetCarry(regs.gpr[rd] > regs.gpr[ra]); // todo: is this right?

            if (instruction.oe()) {
                SetSummaryOverflow(((regs.gpr[ra] ^ regs.gpr[rd]) & (-1 ^ regs.gpr[rd])) & 0x80000000);
            }
            if (instruction.rc()) {
                UpdateCR0(regs.gpr[rd]);
            }
        }
        void SubtractFromZeroExtended(u32&, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rd = instruction.rD();
            regs.gpr[rd] = ~regs.gpr[ra] + (int)GetCarry();
            SetCarry(regs.gpr[rd] > ~regs.gpr[ra]); // todo: is this right?

            if (instruction.oe()) {
                SetSummaryOverflow(((-1 ^ regs.gpr[rd]) & (regs.gpr[ra] ^ regs.gpr[rd])) & 0x80000000);
            }
            if (instruction.rc()) {
                UpdateCR0(regs.gpr[rd]);
            }
        }
        void Negate(u32&, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rd = instruction.rD();
            regs.gpr[rd] = ~regs.gpr[ra] + 1;
            SetCarry(regs.gpr[rd] > ~regs.gpr[ra]); // todo: is this right?

            if (instruction.oe()) {
                SetSummaryOverflow(regs.gpr[ra] == 0x8000'0000);
            }
            if (instruction.rc()) {
                UpdateCR0(regs.gpr[rd]);
            }
        }
        void MultiplyLowImmediate(u32&, Instruction instruction) {
            s16 si = instruction.simm();
            u32 ra = instruction.rA();
            u32 rd = instruction.rD();
            s64 prod = (s64)regs.gpr[ra] * (s64)si;
            regs.gpr[rd] = prod & 0xFFFFFFFF;
        }
        void MultiplyLowWord(u32&, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rb = instruction.rB();
            u32 rd = instruction.rD();
            s64 prod = (s64)regs.gpr[ra] * (s64)regs.gpr[rb];
            regs.gpr[rd] = prod & 0xFFFFFFFF;
            
            if (instruction.oe()) {
                SetSummaryOverflow(prod != (s64)(s32)prod);
            }
            if (instruction.rc()) {
                UpdateCR0(regs.gpr[rd]);
            }
        }
        void MultiplyHighWord(u32&, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rb = instruction.rB();
            u32 rd = instruction.rD();
            s64 prod = (s64)regs.gpr[ra] * (s64)regs.gpr[rb];
            regs.gpr[rd] = (prod >> 32) & 0xFFFFFFFF;
            
            if (instruction.oe()) {
                SetSummaryOverflow(prod != (s64)(s32)prod);
            }
            if (instruction.rc()) {
                UpdateCR0(regs.gpr[rd]);
            }
        }
        void MultiplyHighWordUnsigned(u32&, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rb = instruction.rB();
            u32 rd = instruction.rD();
            u64 prod = (u64)regs.gpr[ra] * (u64)regs.gpr[rb];
            regs.gpr[rd] = (prod >> 32) & 0xFFFFFFFF;
            
            if (instruction.oe()) {
                SetSummaryOverflow(prod != (u64)(u32)prod);
            }
            if (instruction.rc()) {
                UpdateCR0(regs.gpr[rd]);
            }
        }
        void DivideWord(u32&, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rb = instruction.rB();
            u32 rd = instruction.rD();
            if (!regs.gpr[rb]) {
                regs.gpr[rd] = regs.gpr[ra] & 0x80000000 ? 0xFFFFFFFF : 0;
                if (instruction.oe()) {
                    SetSummaryOverflow(true);
                }
            } else if (regs.gpr[ra] == 0x80000000 && regs.gpr[rb] == 0xFFFFFFFF) {
                regs.gpr[rd] = 0xFFFFFFFF;
                if (instruction.oe()) {
                    SetSummaryOverflow(true);
                }
            } else {
                regs.gpr[rd] = (s32)regs.gpr[ra] / (s32)regs.gpr[rb];
                if (instruction.oe()) {
                    SetSummaryOverflow(false);
                }
            }

            if (instruction.rc()) {
                UpdateCR0(regs.gpr[rd]);
            }
        }
        void DivideWordUnsigned(u32&, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rb = instruction.rB();
            u32 rd = instruction.rD();
            if (!regs.gpr[rb]) {
                regs.gpr[rd] = regs.gpr[ra] & 0x80000000 ? 0xFFFFFFFF : 0;
                if (instruction.oe()) {
                    SetSummaryOverflow(true);
                }
            } else {
                regs.gpr[rd] = (u32)regs.gpr[ra] / (u32)regs.gpr[rb];
                if (instruction.oe()) {
                    SetSummaryOverflow(false);
                }
            }

            if (instruction.rc()) {
                UpdateCR0(regs.gpr[rd]);
            }
        }
        void CompareImmediate(u32& cycles, Instruction instruction) {
            if (instruction.opcode & 0x200000) {
                printf("Invalid instruction\n");
                cycles = 0; exception_occured = true;
                return;
            }

            u32 crf_d = instruction.crfD();
            u32 ra = instruction.rA();
            s16 simm = instruction.simm();
            if ((s32)regs.gpr[ra] < (simm)) {
                CR_NEGATIVE(crf_d) = 1;
            } else if ((s32)regs.gpr[ra] > (simm)) {
                CR_POSITIVE(crf_d) = 1;
            } else {
                CR_ZERO(crf_d) = 1;
            }
            CR_SOV(crf_d) = GetSummaryOverflow();
        }
        void Compare(u32& cycles, Instruction instruction) {
            if (instruction.opcode & 0x200000) {
                printf("Invalid instruction\n");
                cycles = 0; exception_occured = true;
                return;
            }
            u32 crf_d = instruction.crfD();
            u32 ra = instruction.rA();
            u32 rb = instruction.rB();
            if ((s32)regs.gpr[ra] < (s32)regs.gpr[rb]) {
                CR_NEGATIVE(crf_d) = 1;
            } else if ((s32)regs.gpr[ra] > (s32)regs.gpr[rb]) {
                CR_POSITIVE(crf_d) = 1;
            } else {
                CR_ZERO(crf_d) = 1;
            }
            CR_SOV(crf_d) = GetSummaryOverflow();
        }
        void CompareLogicalImmediate(u32& cycles, Instruction instruction) {
            if (instruction.opcode & 0x200000) {
                printf("Invalid instruction\n");
                cycles = 0; exception_occured = true;
                return;
            }

            u32 crf_d = instruction.crfD();
            u32 ra = instruction.rA();
            u16 imm = instruction.uimm();
            if (regs.gpr[ra] < imm) {
                CR_NEGATIVE(crf_d) = 1;
            } else if (regs.gpr[ra] > imm) {
                CR_POSITIVE(crf_d) = 1;
            } else {
                CR_ZERO(crf_d) = 1;
            }
            CR_SOV(crf_d) = GetSummaryOverflow();
        }
        void CompareLogical(u32& cycles, Instruction instruction) {
            if (instruction.opcode & 0x200000) {
                printf("Invalid instruction\n");
                cycles = 0; exception_occured = true;
                return;
            }
            u32 crf_d = instruction.crfD();
            u32 ra = instruction.rA();
            u32 rb = instruction.rB();
            if (regs.gpr[ra] < regs.gpr[rb]) {
                CR_NEGATIVE(crf_d) = 1;
            } else if (regs.gpr[ra] > regs.gpr[rb]) {
                CR_POSITIVE(crf_d) = 1;
            } else {
                CR_ZERO(crf_d) = 1;
            }
            CR_SOV(crf_d) = GetSummaryOverflow();
        }
        void TrapWordImmediate(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            s16 simm = instruction.simm();
            s32 to = instruction.to();

            Common::Bitfield<0, 1> to_b0;
            Common::Bitfield<1, 1> to_b1;
            Common::Bitfield<2, 1> to_b2;
            Common::Bitfield<3, 1> to_b3;
            Common::Bitfield<4, 1> to_b4;

            to_b0 = to;
            to_b1 = to;
            to_b2 = to;
            to_b3 = to;
            to_b4 = to;

            if (((s32)regs.gpr[ra] < simm) && to_b0) {
                cycles = 0; exception_occured = true;
                TrapHandler(instruction.opcode);
            }
            if (((s32)regs.gpr[ra] > simm) && to_b1) {
                cycles = 0; exception_occured = true;
                TrapHandler(instruction.opcode);
            }
            if (((s32)regs.gpr[ra] == simm) && to_b2) {
                cycles = 0; exception_occured = true;
                TrapHandler(instruction.opcode);
            }
            if ((regs.gpr[ra] < (u16)simm) && to_b3) {
                cycles = 0; exception_occured = true;
                TrapHandler(instruction.opcode);
            }
            if ((regs.gpr[ra] > (u16)simm) && to_b4) {
                cycles = 0; exception_occured = true;
                TrapHandler(instruction.opcode);
            }
        }
        void TrapWord(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rb = instruction.rB();
            s32 to = instruction.to();

            Common::Bitfield<0, 1> to_b0;
            Common::Bitfield<1, 1> to_b1;
            Common::Bitfield<2, 1> to_b2;
            Common::Bitfield<3, 1> to_b3;
            Common::Bitfield<4, 1> to_b4;

            to_b0 = to;
            to_b1 = to;
            to_b2 = to;
            to_b3 = to;
            to_b4 = to;

            if (((s32)regs.gpr[ra] < (s32)regs.gpr[rb]) && to_b0) {
                cycles = 0; exception_occured = true;
                TrapHandler(instruction.opcode);
            }
            if (((s32)regs.gpr[ra] > (s32)regs.gpr[rb]) && to_b1) {
                cycles = 0; exception_occured = true;
                TrapHandler(instruction.opcode);
            }
            if (((s32)regs.gpr[ra] == (s32)regs.gpr[rb]) && to_b2) {
                cycles = 0; exception_occured = true;
                TrapHandler(instruction.opcode);
            }
            if ((regs.gpr[ra] < regs.gpr[rb]) && to_b3) {
                cycles = 0; exception_occured = true;
                TrapHandler(instruction.opcode);
            }
            if ((regs.gpr[ra] > regs.gpr[rb]) && to_b4) {
                cycles = 0; exception_occured = true;
                TrapHandler(instruction.opcode);
            }
        }

        void AndImmediate(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            u16 uimm = instruction.uimm();

            regs.gpr[ra] = regs.gpr[rs] & uimm;
            UpdateCR0(regs.gpr[ra]);
        }

        void AndImmediateShifted(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            u16 uimm = instruction.uimm();

            regs.gpr[ra] = regs.gpr[rs] & (uimm << 16);
            UpdateCR0(regs.gpr[ra]);
        }
        void OrImmediate(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            u16 uimm = instruction.uimm();

            regs.gpr[ra] = regs.gpr[rs] | uimm;
        }
        void OrImmediateShifted(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            u16 uimm = instruction.uimm();

            regs.gpr[ra] = regs.gpr[rs] | (uimm << 16);
        }
        void XorImmediate(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            u16 uimm = instruction.uimm();

            regs.gpr[ra] = regs.gpr[rs] ^ uimm;
        }
        void XorImmediateShifted(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            u16 uimm = instruction.uimm();

            regs.gpr[ra] = regs.gpr[rs] ^ (uimm << 16);
        }
        void AndRegister(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            u32 rb = instruction.rB();

            regs.gpr[ra] = regs.gpr[rs] & regs.gpr[rb];
            if (instruction.rc()) {
                UpdateCR0(regs.gpr[ra]);
            }
        }
        void OrRegister(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            u32 rb = instruction.rB();

            regs.gpr[ra] = regs.gpr[rs] | regs.gpr[rb];
            if (instruction.rc()) {
                UpdateCR0(regs.gpr[ra]);
            }
        }
        void XorRegister(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            u32 rb = instruction.rB();

            regs.gpr[ra] = regs.gpr[rs] ^ regs.gpr[rb];
            if (instruction.rc()) {
                UpdateCR0(regs.gpr[ra]);
            }
        }
        void NandRegister(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            u32 rb = instruction.rB();

            regs.gpr[ra] = ~(regs.gpr[rs] & regs.gpr[rb]);
            if (instruction.rc()) {
                UpdateCR0(regs.gpr[ra]);
            }
        }
        void NorRegister(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            u32 rb = instruction.rB();

            regs.gpr[ra] = ~(regs.gpr[rs] | regs.gpr[rb]);
            if (instruction.rc()) {
                UpdateCR0(regs.gpr[ra]);
            }
        }
        void EqvRegister(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            u32 rb = instruction.rB();

            regs.gpr[ra] = ~(regs.gpr[rs] ^ regs.gpr[rb]);
            if (instruction.rc()) {
                UpdateCR0(regs.gpr[ra]);
            }
        }
        void AndRegisterWithComplement(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            u32 rb = instruction.rB();

            regs.gpr[ra] = (regs.gpr[rs] & ~regs.gpr[rb]);
            if (instruction.rc()) {
                UpdateCR0(regs.gpr[ra]);
            }
        }
        void OrRegisterWithComplement(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
            u32 rb = instruction.rB();

            regs.gpr[ra] = (regs.gpr[rs] | ~regs.gpr[rb]);
            if (instruction.rc()) {
                UpdateCR0(regs.gpr[ra]);
            }
        }
        void ExtendSignByte(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();

            regs.gpr[ra] = (s8)regs.gpr[rs];

            if (instruction.rc()) {
                UpdateCR0(regs.gpr[ra]);
            }
        }
        void ExtendSignHalfWord(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();

            regs.gpr[ra] = (s16)regs.gpr[rs];

            if (instruction.rc()) {
                UpdateCR0(regs.gpr[ra]);
            }
        }
        void ExtendSignWord(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();

            regs.gpr[ra] = (s32)regs.gpr[rs];

            if (instruction.rc()) {
                UpdateCR0(regs.gpr[ra]);
            }
        }
        void CountLeadingZerosWord(u32& cycles, Instruction instruction) {
            u32 ra = instruction.rA();
            u32 rs = instruction.rS();
#ifdef __GNUC__
            regs.gpr[ra] = __builtin_clz(regs.gpr[rs]);
#elif _MSC_VER
            regs.gpr[ra] = __lzcnt(regs.gpr[rs]);
#endif
            if (instruction.rc()) {
                UpdateCR0(regs.gpr[ra]);
            }
        }
        void RotateLeftWordImmediateThenANDWithMask(u32& cycles, Instruction instruction) {
            auto rs = instruction.rS();
            auto ra = instruction.rA();
            auto sh = instruction.sh();
            auto mb = instruction.mb();
            auto me = instruction.me();

            u32 mask = GetRotateMask(mb, me);
            u32 rotated = ((regs.gpr[rs] << regs.gpr[sh]) | (regs.gpr[rs] >> (32 - regs.gpr[sh]))) & mask;
            regs.gpr[ra] = rotated;

            if (instruction.rc()) {
                UpdateCR0(regs.gpr[ra]);
            }
        }
        void RotateLeftWordThenANDWithMask(u32& cycles, Instruction instruction) {
            auto rs = instruction.rS();
            auto ra = instruction.rA();
            auto rb = instruction.rB();
            auto mb = instruction.mb();
            auto me = instruction.me();

            u32 mask = GetRotateMask(mb, me);
            u32 rotated = ((regs.gpr[rs] << regs.gpr[rb]) | (regs.gpr[rs] >> regs.gpr[rb])) & mask;
            regs.gpr[ra] = rotated;

            if (instruction.rc()) {
                UpdateCR0(regs.gpr[ra]);
            }
        }
        void RotateLeftWordImmediateThenMaskInsert(u32& cycles, Instruction instruction) {
            auto rs = instruction.rS();
            auto ra = instruction.rA();
            auto sh = instruction.sh();
            auto mb = instruction.mb();
            auto me = instruction.me();

            u32 mask = GetRotateMask(mb, me);
            u32 rotated = ((regs.gpr[rs] << regs.gpr[sh]) | (regs.gpr[rs] >> (32 - regs.gpr[sh]))) & mask;
            regs.gpr[ra] = (regs.gpr[ra] & ~mask) | rotated;

            if (instruction.rc()) {
                UpdateCR0(regs.gpr[ra]);
            }
        }
        void ShiftLeftWord(u32& cycles, Instruction instruction) {
            auto ra = instruction.rA();
            auto rs = instruction.rS();
            auto rb = instruction.rB();

            regs.gpr[ra] = (regs.gpr[rs] << (regs.gpr[rb] & 32 ? 0 : regs.gpr[rb] & 31));

            if (instruction.rc()) {
                UpdateCR0(regs.gpr[ra]);
            }
        }
        void ShiftRightWord(u32& cycles, Instruction instruction) {
            auto ra = instruction.rA();
            auto rs = instruction.rS();
            auto rb = instruction.rB();

            regs.gpr[ra] = (regs.gpr[rs] >> (regs.gpr[rb] & 32 ? 0 : regs.gpr[rb] & 31));

            if (instruction.rc()) {
                UpdateCR0(regs.gpr[ra]);
            }
        }
        void ShiftRightAlgebraicWordImmediate(u32& cycles, Instruction instruction) {
            auto ra = instruction.rA();
            auto rs = instruction.rS();
            auto sh = instruction.sh();

            u32 mask = (1 << sh) - 1;

            regs.gpr[ra] = (regs.gpr[rs] >> (sh));
            SetCarry((regs.gpr[rs] & 0x80000000) && (regs.gpr[rs] & mask));

            if (instruction.rc()) {
                UpdateCR0(regs.gpr[ra]);
            }
        }
        void ShiftRightAlgebraicWord(u32& cycles, Instruction instruction) {
            auto ra = instruction.rA();
            auto rs = instruction.rS();
            auto rb = instruction.rB();

            u32 mask = (1 << (regs.gpr[rb] & 31)) - 1;

            regs.gpr[ra] = (regs.gpr[rs] >> (regs.gpr[rb] & 32 ? 31 : regs.gpr[rb] & 31));
            if (regs.gpr[rb] & 32)
                regs.xer.whole |= (regs.gpr[ra] & 1) << 29;
            SetCarry((regs.gpr[rs] & 0x80000000) && (regs.gpr[rs] & mask));

            if (instruction.rc()) {
                UpdateCR0(regs.gpr[ra]);
            }
        }
        void MoveToSpecialPurposeRegister(u32& cycles, Instruction instruction) {
            auto rs = instruction.rS();
            u32 spr = (((instruction.opcode >> 11) & 0x1F) << 5) | ((instruction.opcode >> 16) & 0x1F);
            switch (spr) {
                case 1: { // xer
                    regs.xer.whole = regs.gpr[rs];
                    break;
                }
                case 8: { // lr
                    regs.lr = regs.gpr[rs];
                    break;
                }
                case 9: { // ctr
                    regs.ctr = regs.gpr[rs];
                    break;
                }
                default: {
                    printf("Unknown SPR %d\n", spr);
                    TrapHandler(instruction.opcode);
                    break;
                }
            }
        }
        void MoveFromSpecialPurposeRegister(u32& cycles, Instruction instruction) {
            auto ra = instruction.rD();
            u32 spr = (((instruction.opcode >> 11) & 0x1F) << 5) | ((instruction.opcode >> 16) & 0x1F);
            switch (spr) {
                case 1: { // xer
                    regs.gpr[ra] = regs.xer.whole;
                    break;
                }
                case 8: { // lr
                    regs.gpr[ra] = regs.lr;
                    break;
                }
                case 9: { // ctr
                    regs.gpr[ra] = regs.ctr;
                    break;
                }
                default: {
                    printf("Unknown SPR %d\n", spr);
                    TrapHandler(instruction.opcode);
                    break;
                }
            }
        }
        void MoveToConditionRegisterFields(u32& cycles, Instruction instruction) {
            auto rs = instruction.rS();
            u32 mask = 0;
            auto crm = ((instruction.opcode >> 12) & 0xFF);

            mask += (crm & 128) ? 0xF0000000 : 0;
            mask += (crm & 64)  ? 0x0F000000 : 0;
            mask += (crm & 32)  ? 0x00F00000 : 0;
            mask += (crm & 16)  ? 0x000F0000 : 0;
            mask += (crm & 8)   ? 0x0000F000 : 0;
            mask += (crm & 4)   ? 0x00000F00 : 0;
            mask += (crm & 2)   ? 0x000000F0 : 0;
            mask += (crm & 1)   ? 0x0000000F : 0;

            regs.cr.bits = (regs.gpr[rs] & mask) | ((u32)regs.cr.bits.to_ulong() & ~mask);
        }
        void MoveFromConditionRegister(u32& cycles, Instruction instruction) {
            auto rd = instruction.rD();
            regs.gpr[rd] = (u32)regs.cr.bits.to_ulong();
        }
#pragma endregion

#pragma region FloatingPointInstructions
        void LoadFloatingPointSingle(u32& cycles, Instruction instruction) {
            auto frt = instruction.frD();
            auto ra = instruction.frA();

            auto d = instruction.d();

            u32 address = regs.gpr[ra] + (d);

            regs.fpr[frt] = memory->ReadFloat(address);
        }
        void LoadFloatingPointSingleIndexed(u32& cycles, Instruction instruction) {
            auto ra = instruction.rA();
            auto rb = instruction.rB();
            auto frt = instruction.frD();

            if (ra == 0) {
                printf("LoadFloatingPointSingleIndexed with rA == 0\n");
                cycles = 0; exception_occured = true;
                return;
            }

            u32 address = regs.gpr[ra] + regs.gpr[rb];

            regs.fpr[frt] = memory->ReadFloat(address);
        }
        void LoadFloatingPointSingleWithUpdate(u32& cycles, Instruction instruction) {
            auto ra = instruction.rA();
            auto frt = instruction.frD();

            auto d = instruction.d();

            u32 address = regs.gpr[ra] + (d);

            regs.fpr[frt] = memory->ReadFloat(address);
            regs.gpr[ra] = address;
        }
        void LoadFloatingPointSingleWithUpdateIndexed(u32& cycles, Instruction instruction) {
            auto ra = instruction.rA();
            auto rb = instruction.rB();
            auto frt = instruction.frD();

            if (ra == 0) {
                printf("LoadFloatingPointSingleWithUpdateIndexed with rA == 0\n");
                cycles = 0; exception_occured = true;
                return;
            }

            u32 address = regs.gpr[ra] + regs.gpr[rb];

            regs.fpr[frt] = memory->ReadFloat(address);
            regs.gpr[ra] = address;
        }
        void LoadFloatingPointDouble(u32& cycles, Instruction instruction) {
            auto frt = instruction.frD();
            auto ra = instruction.frA();

            auto d = instruction.d();

            u32 address = regs.gpr[ra] + (d);

            regs.fpr[frt] = memory->ReadDouble(address);
        }
        void LoadFloatingPointDoubleIndexed(u32& cycles, Instruction instruction) {
            auto ra = instruction.rA();
            auto rb = instruction.rB();
            auto frt = instruction.frD();

            u32 address = regs.gpr[ra] + regs.gpr[rb];

            regs.fpr[frt] = memory->ReadDouble(address);
        }
        void LoadFloatingPointDoubleWithUpdate(u32& cycles, Instruction instruction) {
            auto ra = instruction.rA();
            auto frt = instruction.frD();

            auto d = instruction.d();

            if (ra == 0) {
                printf("LoadFloatingPointDoubleWithUpdate with rA == 0\n");
                cycles = 0; exception_occured = true;
                return;
            }

            u32 address = regs.gpr[ra] + (d);

            regs.fpr[frt] = memory->ReadDouble(address);
            regs.gpr[ra] = address;
        }
        void LoadFloatingPointDoubleWithUpdateIndexed(u32& cycles, Instruction instruction) {
            auto ra = instruction.rA();
            auto rb = instruction.rB();
            auto frt = instruction.frD();

            if (ra == 0) {
                printf("LoadFloatingPointDoubleWithUpdateIndexed with rA == 0\n");
                cycles = 0; exception_occured = true;
                return;
            }

            u32 address = regs.gpr[ra] + regs.gpr[rb];

            regs.fpr[frt] = memory->ReadDouble(address);
            regs.gpr[ra] = address;
        }
        void StoreFloatingPointSingle(u32& cycles, Instruction instruction) {
            auto frt = instruction.frD();
            auto ra = instruction.rA();

            auto d = instruction.d();

            u32 address = regs.gpr[ra] + (d);

            memory->WriteFloat(address, regs.fpr[frt]);
        }
        void StoreFloatingPointSingleIndexed(u32& cycles, Instruction instruction) {
            auto ra = instruction.rA();
            auto rb = instruction.rB();
            auto frt = instruction.frD();

            u32 address = regs.gpr[ra] + regs.gpr[rb];

            memory->WriteFloat(address, regs.fpr[frt]);
        }
        void StoreFloatingPointSingleWithUpdate(u32& cycles, Instruction instruction) {
            auto ra = instruction.rA();
            auto frt = instruction.frD();

            auto d = instruction.d();
            

            if (ra == 0) {
                printf("StoreFloatingPointSingleWithUpdate with rA == 0\n");
                cycles = 0; exception_occured = true;
                return;
            }

            u32 address = regs.gpr[ra] + (d);

            memory->WriteFloat(address, regs.fpr[frt]);
            regs.gpr[ra] = address;
        }
        void StoreFloatingPointSingleWithUpdateIndexed(u32& cycles, Instruction instruction) {
            auto ra = instruction.rA();
            auto rb = instruction.rB();
            auto frt = instruction.frD();

            if (ra == 0) {
                printf("StoreFloatingPointSingleWithUpdateIndexed with rA == 0\n");
                cycles = 0; exception_occured = true;
                return;
            }

            u32 address = regs.gpr[ra] + regs.gpr[rb];

            memory->WriteFloat(address, regs.fpr[frt]);
            regs.gpr[ra] = address;
        }
        void StoreFloatingPointDouble(u32& cycles, Instruction instruction) {
            auto frt = instruction.frD();
            auto ra = instruction.rA();

            auto d = instruction.d();

            u32 address = regs.gpr[ra] + (d);

            memory->WriteDouble(address, regs.fpr[frt]);
        }
        void StoreFloatingPointDoubleIndexed(u32& cycles, Instruction instruction) {
            auto ra = instruction.rA();
            auto rb = instruction.rB();
            auto frt = instruction.frD();

            u32 address = regs.gpr[ra] + regs.gpr[rb];

            memory->WriteDouble(address, regs.fpr[frt]);
        }
        void StoreFloatingPointDoubleWithUpdate(u32& cycles, Instruction instruction) {
            auto ra = instruction.rA();
            auto frt = instruction.frD();

            auto d = instruction.d();

            if (ra == 0) {
                printf("StoreFloatingPointDoubleWithUpdate with rA == 0\n");
                cycles = 0; exception_occured = true;
                return;
            }

            u32 address = regs.gpr[ra] + (d);

            memory->WriteDouble(address, regs.fpr[frt]);
            regs.gpr[ra] = address;
        }
        void StoreFloatingPointDoubleWithUpdateIndexed(u32& cycles, Instruction instruction) {
            auto ra = instruction.rA();
            auto rb = instruction.rB();
            auto frt = instruction.frD();

            if (ra == 0) {
                printf("StoreFloatingPointDoubleWithUpdateIndexed with rA == 0\n");
                cycles = 0; exception_occured = true;
                return;
            }

            u32 address = regs.gpr[ra] + regs.gpr[rb];

            memory->WriteDouble(address, regs.fpr[frt]);
            regs.gpr[ra] = address;
        }
        void StoreFloatingPointAsIntegerWordIndexed(u32& cycles, Instruction instruction) {
            auto ra = instruction.rA();
            auto rb = instruction.rB();
            auto frt = instruction.frD();

            u32 address = regs.gpr[ra] + regs.gpr[rb];

            memory->Write32(address, (u32)regs.fpr[frt]);
        }
        void FloatingPointMoveRegister(u32& cycles, Instruction instruction) {
            auto frt = instruction.frD();
            auto frb = instruction.frB();

            regs.fpr[frt] = regs.fpr[frb];

            ConfirmFPResult(instruction.rc(), regs.fpr[frt]);
        }
        void FloatingNegate(u32& cycles, Instruction instruction) {
            auto frt = instruction.frD();
            auto frb = instruction.frB();

            regs.fpr[frt] = -regs.fpr[frb];

            ConfirmFPResult(instruction.rc(), regs.fpr[frt]);
        }
        void FloatingAbs(u32& cycles, Instruction instruction) {
            auto frt = instruction.frD();
            auto frb = instruction.frB();

            regs.fpr[frt] = fabs(regs.fpr[frb]);

            ConfirmFPResult(instruction.rc(), regs.fpr[frt]);
        }
        void FloatingNegativeAbs(u32& cycles, Instruction instruction) {
            auto frt = instruction.frD();
            auto frb = instruction.frB();

            regs.fpr[frt] = -fabs(regs.fpr[frb]);

            ConfirmFPResult(instruction.rc(), regs.fpr[frt]);
        }
        void FloatingAddSingle(u32& cycles, Instruction instruction) {
            auto frt = instruction.frD();
            auto frb = instruction.frB();
            auto fra = instruction.frA();

            regs.fpr[frt] = (float)regs.fpr[frb] + (float)regs.fpr[fra];

            ConfirmFPResult(instruction.rc(), regs.fpr[frt]);
        }
        void FloatingAddDouble(u32& cycles, Instruction instruction) {
            auto frt = instruction.frD();
            auto frb = instruction.frB();
            auto fra = instruction.frA();

            regs.fpr[frt] = regs.fpr[frb] + regs.fpr[fra];

            ConfirmFPResult(instruction.rc(), regs.fpr[frt]);
        }
        void FloatingSubtractSingle(u32& cycles, Instruction instruction) {
            auto frt = instruction.frD();
            auto frb = instruction.frB();
            auto fra = instruction.frA();

            regs.fpr[frt] = (float)regs.fpr[fra] - (float)regs.fpr[frb];

            ConfirmFPResult(instruction.rc(), regs.fpr[frt]);
        }
        void FloatingSubtractDouble(u32& cycles, Instruction instruction) {
            auto frt = instruction.frD();
            auto frb = instruction.frB();
            auto fra = instruction.frA();

            regs.fpr[frt] = regs.fpr[fra] - regs.fpr[frb];

            ConfirmFPResult(instruction.rc(), regs.fpr[frt]);
        }
        void FloatingMultiplySingle(u32& cycles, Instruction instruction) {
            auto frt = instruction.frD();
            auto frb = instruction.frB();
            auto fra = instruction.frA();

            regs.fpr[frt] = (float)regs.fpr[fra] * (float)regs.fpr[frb];

            ConfirmFPResult(instruction.rc(), regs.fpr[frt]);
        }
        void FloatingMultiplyDouble(u32& cycles, Instruction instruction) {
            auto frt = instruction.frD();
            auto frb = instruction.frB();
            auto fra = instruction.frA();

            regs.fpr[frt] = regs.fpr[fra] * regs.fpr[frb];

            ConfirmFPResult(instruction.rc(), regs.fpr[frt]);
        }
        void FloatingDivideSingle(u32& cycles, Instruction instruction) {
            auto frt = instruction.frD();
            auto frb = instruction.frB();
            auto fra = instruction.frA();

            regs.fpr[frt] = (float)regs.fpr[fra] / (float)regs.fpr[frb];

            ConfirmFPResult(instruction.rc(), regs.fpr[frt]);
        }
        void FloatingDivideDouble(u32& cycles, Instruction instruction) {
            auto frt = instruction.frD();
            auto frb = instruction.frB();
            auto fra = instruction.frA();

            regs.fpr[frt] = regs.fpr[fra] / regs.fpr[frb];

            ConfirmFPResult(instruction.rc(), regs.fpr[frt]);
        }
        void FloatingMultiplyAddSingle(u32& cycles, Instruction instruction) {
            auto frt = instruction.frD();
            auto frc = instruction.frC();
            auto frb = instruction.frB();
            auto fra = instruction.frA();

            regs.fpr[frt] = (float)regs.fpr[fra] * (float)regs.fpr[frc] + (float)regs.fpr[frb];

            ConfirmFPResult(instruction.rc(), regs.fpr[frt]);
        }
        void FloatingMultiplyAddDouble(u32& cycles, Instruction instruction) {
            auto frt = instruction.frD();
            auto frc = instruction.frC();
            auto frb = instruction.frB();
            auto fra = instruction.frA();

            regs.fpr[frt] = regs.fpr[fra] * regs.fpr[frc] + regs.fpr[frb];

            ConfirmFPResult(instruction.rc(), regs.fpr[frt]);
        }
        void FloatingMultiplySubtractSingle(u32& cycles, Instruction instruction) {
            auto frt = instruction.frD();
            auto frc = instruction.frC();
            auto frb = instruction.frB();
            auto fra = instruction.frA();

            regs.fpr[frt] = (float)regs.fpr[fra] * (float)regs.fpr[frc] - (float)regs.fpr[frb];

            ConfirmFPResult(instruction.rc(), regs.fpr[frt]);
        }
        void FloatingMultiplySubtractDouble(u32& cycles, Instruction instruction) {
            auto frt = instruction.frD();
            auto frc = instruction.frC();
            auto frb = instruction.frB();
            auto fra = instruction.frA();

            regs.fpr[frt] = regs.fpr[fra] * regs.fpr[frc] - regs.fpr[frb];

            ConfirmFPResult(instruction.rc(), regs.fpr[frt]);
        }
        void FloatingNegativeMultiplyAddSingle(u32& cycles, Instruction instruction) {
            auto frt = instruction.frD();
            auto frc = instruction.frC();
            auto frb = instruction.frB();
            auto fra = instruction.frA();

            regs.fpr[frt] = -((float)regs.fpr[fra] * (float)regs.fpr[frc] + (float)regs.fpr[frb]);

            ConfirmFPResult(instruction.rc(), regs.fpr[frt]);
        }
        void FloatingNegativeMultiplyAddDouble(u32& cycles, Instruction instruction) {
            auto frt = instruction.frD();
            auto frc = instruction.frC();
            auto frb = instruction.frB();
            auto fra = instruction.frA();

            regs.fpr[frt] = -(regs.fpr[fra] * regs.fpr[frc] + regs.fpr[frb]);

            ConfirmFPResult(instruction.rc(), regs.fpr[frt]);
        }
        void FloatingNegativeMultiplySubtractSingle(u32& cycles, Instruction instruction) {
            auto frt = instruction.frD();
            auto frc = instruction.frC();
            auto frb = instruction.frB();
            auto fra = instruction.frA();

            regs.fpr[frt] = -((float)regs.fpr[fra] * (float)regs.fpr[frc] - (float)regs.fpr[frb]);

            ConfirmFPResult(instruction.rc(), regs.fpr[frt]);
        }
        void FloatingNegativeMultiplySubtractDouble(u32& cycles, Instruction instruction) {
            auto frt = instruction.frD();
            auto frc = instruction.frC();
            auto frb = instruction.frB();
            auto fra = instruction.frA();

            regs.fpr[frt] = -(regs.fpr[fra] * regs.fpr[frc] - regs.fpr[frb]);

            ConfirmFPResult(instruction.rc(), regs.fpr[frt]);
        }
        void FloatingRoundToSinglePrecision(u32& cycles, Instruction instruction) {
            auto frt = instruction.frD();
            auto frb = instruction.frB();

            regs.fpr[frt] = (float)regs.fpr[frb];

            ConfirmFPResult(instruction.rc(), regs.fpr[frt]);
        }
        void FloatingConvertToIntegerWord(u32& cycles, Instruction instruction) {
            auto frt = instruction.frD();
            auto frb = instruction.frB();

            regs.fpr[frt] = (s32)regs.fpr[frb];

            ConfirmFPResult(instruction.rc(), regs.fpr[frt]);
        }
        void FloatingConvertToIntegerWordWithRoundTowardZero(u32& cycles, Instruction instruction) {
            auto frt = instruction.frD();
            auto frb = instruction.frB();

            regs.fpr[frt] = (s32)regs.fpr[frb]; // todo: round

            ConfirmFPResult(instruction.rc(), regs.fpr[frt]);
        }
        void FloatingCompareUnordered(u32& cycles, Instruction instruction) {
            auto frb = instruction.frB();
            auto fra = instruction.frA();
            auto crf = instruction.crfD();

            if (regs.fpr[fra] < regs.fpr[frb]) {
                CR_NEGATIVE(crf) = 1;
            } else if (regs.fpr[fra] > regs.fpr[frb]) {
                CR_POSITIVE(crf) = 1;
            } else {
                CR_ZERO(crf) = 1;
            }
            if (isnan(regs.fpr[fra]) || isnan(regs.fpr[frb])) {
                CR_SOV(crf) = 1;
            } else {
                CR_SOV(crf) = 0;
            }
        }
        void FloatingCompareOrdered(u32& cycles, Instruction instruction) {
            auto frb = instruction.frB();
            auto fra = instruction.frA();
            auto crf = instruction.crfD();

            if (regs.fpr[fra] < regs.fpr[frb]) {
                CR_NEGATIVE(crf) = 1;
            } else if (regs.fpr[fra] > regs.fpr[frb]) {
                CR_POSITIVE(crf) = 1;
            } else {
                CR_ZERO(crf) = 1;
            }
            if (isnan(regs.fpr[fra]) || isnan(regs.fpr[frb])) {
                CR_SOV(crf) = 1;
            } else {
                CR_SOV(crf) = 0;
            }
        }
        void MoveFromFPSCR(u32& cycles, Instruction instruction) {
            auto frt = instruction.frD();

            u64 frt_tmp = (s64)regs.fpr[frt] & ((u64)0xFFF80000 << 32);
            frt_tmp |= (u64)regs.fpscr.bits.to_ullong();
            regs.fpr[frt] = (u64)frt_tmp;

            ConfirmFPResult(instruction.rc(), regs.fpr[frt]);
        }
        void MoveToConditionRegisterFromFPSCR(u32& cycles, Instruction instruction) {
            auto crf = instruction.crfD();
            auto cra = instruction.crfS();

            for (size_t i = 0; i < 4; i++) {
                regs.cr.bits[crf * 4 + i] = regs.fpscr.bits[cra * 4 + i];
                // clear all the copied exception bits
                if (regs.fpscr.bits[cra * 4 + i] & FPSCR::FX) {
                    regs.fpscr.bits &= ~FPSCR::FX;
                }
                if (regs.fpscr.bits[cra * 4 + i] & FPSCR::FEX) {
                    regs.fpscr.bits &= ~FPSCR::FEX;
                }
                if (regs.fpscr.bits[cra * 4 + i] & FPSCR::VX) {
                    regs.fpscr.bits &= ~FPSCR::VX;
                }
                if (regs.fpscr.bits[cra * 4 + i] & FPSCR::OX) {
                    regs.fpscr.bits &= ~FPSCR::OX;
                }
                if (regs.fpscr.bits[cra * 4 + i] & FPSCR::UX) {
                    regs.fpscr.bits &= ~FPSCR::UX;
                }
                if (regs.fpscr.bits[cra * 4 + i] & FPSCR::ZX) {
                    regs.fpscr.bits &= ~FPSCR::ZX;
                }
                if (regs.fpscr.bits[cra * 4 + i] & FPSCR::XX) {
                    regs.fpscr.bits &= ~FPSCR::XX;
                }
                if (regs.fpscr.bits[cra * 4 + i] & FPSCR::VXSNAN) {
                    regs.fpscr.bits &= ~FPSCR::VXSNAN;
                }
                if (regs.fpscr.bits[cra * 4 + i] & FPSCR::VXISI) {
                    regs.fpscr.bits &= ~FPSCR::VXISI;
                }
                if (regs.fpscr.bits[cra * 4 + i] & FPSCR::VXIDI) {
                    regs.fpscr.bits &= ~FPSCR::VXIDI;
                }
                if (regs.fpscr.bits[cra * 4 + i] & FPSCR::VXZDZ) {
                    regs.fpscr.bits &= ~FPSCR::VXZDZ;
                }
                if (regs.fpscr.bits[cra * 4 + i] & FPSCR::VXIMZ) {
                    regs.fpscr.bits &= ~FPSCR::VXIMZ;
                }
                if (regs.fpscr.bits[cra * 4 + i] & FPSCR::VXVC) {
                    regs.fpscr.bits &= ~FPSCR::VXVC;
                }
            }
        }
        void MoveToFPSCRFieldImmediate(u32& cycles, Instruction instruction) {
            auto crf = instruction.crfD();
            std::bitset<4> u = (instruction.opcode >> 11) & 15;

            for (size_t i = 0; i < 4; i++) {
                regs.fpscr.bits[crf * 4 + i] = u[i];
            }
            
            if(instruction.rc())
                UpdateCR1();
        }
        void MoveToFPSCRFields(u32& cycles, Instruction instruction) {
            auto rb = (instruction.opcode >> 11) & 31;
            u32 mask = 0;
            auto crm = (instruction.opcode >> 17) & 255;
            mask += (crm & 128) ? 0xF0000000 : 0;
            mask += (crm & 64)  ? 0x0F000000 : 0;
            mask += (crm & 32)  ? 0x00F00000 : 0;
            mask += (crm & 16)  ? 0x000F0000 : 0;
            mask += (crm & 8)   ? 0x0000F000 : 0;
            mask += (crm & 4)   ? 0x00000F00 : 0;
            mask += (crm & 2)   ? 0x000000F0 : 0;
            mask += (crm & 1)   ? 0x0000000F : 0;
            u32 fprval = (u32)regs.fpr[rb];
            regs.fpscr.bits = (fprval & mask) | (fprval & ~mask);
            if(instruction.rc())
                UpdateCR1();
        }
        void MoveToFPSCRBit0(u32& cycles, Instruction instruction) {
            auto crb = instruction.crbD();
            regs.fpscr.bits[crb] = 0;
            if(instruction.rc())
                UpdateCR1();
        }
        void MoveToFPSCRBit1(u32& cycles, Instruction instruction) {
            auto crb = instruction.crbD();
            regs.fpscr.bits[crb] = 1;
            if(instruction.rc())
                UpdateCR1();
        }
#pragma endregion
        void TrapHandler(u32 opcode) {
            // TODO: make a proper exception/trap handler
            printf("Error occured at pc %08X. Opcode was %08X\n", pc, opcode);
            exception_occured = true;
            Reset();
        }

        u32 GetRotateMask(u32 mb, u32 me) {
            u32 m1 = 0xFFFFFFFF >> mb;
            u32 m2 = 0xFFFFFFFF << (31 - me);
            
            return ((mb <= me) ? (m2 & m1) : (m1 | m2));
        }

        u32 GetPC() { return /*bswap_32*/(pc); }
        u32 GetGPR(u32 index) { return /*bswap_32*/(regs.gpr[index]); }
        float GetFPR(u32 index) { return /*bswap_f32*/(regs.fpr[index]); }
        u32 GetXER() { return /*bswap_32*/(regs.xer.whole); }
        u32 GetCTR() { return /*bswap_32*/(regs.ctr); }
        u32 GetLR() { return /*bswap_32*/(regs.lr); }
        std::bitset<32Ui64> GetFPSCR() { return /*bswap_32*/(regs.fpscr.bits); }
        std::bitset<32Ui64> GetCR() { return /*bswap_32*/(regs.cr.bits); }

        void SetPC(u32 value) { pc = /*bswap_32*/(value); }
        void SetGPR(u32 index, u32 value) { regs.gpr[index] = /*bswap_32*/(value); }
        void SetFPR(u32 index, float value) { regs.fpr[index] = /*bswap_f32*/(value); }
        void SetXER(u32 value) { regs.xer.whole = /*bswap_32*/(value); }
        void SetCTR(u32 value) { regs.ctr = /*bswap_32*/(value); }
        void SetLR(u32 value) { regs.lr = /*bswap_32*/(value); }
        void SetFPSCR(std::bitset<32Ui64> value) { regs.fpscr.bits = /*bswap_32*/(value); }
        void SetCR(std::bitset<32Ui64> value) { regs.cr.bits = /*bswap_32*/(value); }
    private:
        std::map<Operation, InstructionFunctionMap> instruction_map;
    };
}