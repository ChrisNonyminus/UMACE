#pragma once

#include <memory>
#include <bitset>

#include "emucore/common/types.h"
#include "emucore/common/bitfield.h"

namespace Core {
    class System;
    class HardwareComponent : public std::enable_shared_from_this<HardwareComponent> {
    protected:
        System* system;
    public:
        HardwareComponent(System* system);
        template <typename T>
        std::shared_ptr<T> DynamicCast() {
            return std::dynamic_pointer_cast<T>(shared_from_this());
        }
        virtual int  AcknowledgeInterrupt() = 0;
        void         SetInterrupt(int interrupt_id);
        virtual bool Reset() = 0;
        virtual void Update() = 0;
    };
} // namespace Core
