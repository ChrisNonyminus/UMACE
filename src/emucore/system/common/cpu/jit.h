#pragma once

#include <memory>
#include <bitset>

#include <asmjit/asmjit.h>

#include "emucore/common/types.h"
#include "emucore/common/bitfield.h"

namespace Core {
    class JITCommon {
    private:
        asmjit::JitRuntime runtime;
        asmjit::CodeHolder code;
    public:
        JITCommon() {
            code.init(runtime.environment());
        }
        virtual void HandleOp(u32 op) = 0;
    };
} // namespace Core
