#pragma once

#include "emucore/common/types.h"

namespace Common
{
    
    //thanks camthesaxman
    #define BIT_RANGE(val, start, end) ((val >> start) & ((1<<(end+1-start))-1))

    //thanks https://blog.codef00.com/2014/12/06/portable-bitfields-using-c11/
    template <size_t LastBit>
    struct MinimumTypeHelper {
        typedef
            typename std::conditional<LastBit == 0 , void,
            typename std::conditional<LastBit <= 8 , uint8_t,
            typename std::conditional<LastBit <= 16, uint16_t,
            typename std::conditional<LastBit <= 32, uint32_t,
            typename std::conditional<LastBit <= 64, uint64_t,
            void>::type>::type>::type>::type>::type type;
    };
    template <size_t Index, size_t Bits = 1>
    class Bitfield {
    private:
        enum {
            Mask = (1u << Bits) - 1u
        };

        typedef typename MinimumTypeHelper<Index + Bits>::type T;
    public:
        template <class T2>
        Bitfield &operator=(T2 value) {
            value_ = (value_ & ~(Mask << Index)) | ((value & Mask) << Index);
            return *this;
        }

        operator T() const             { return (value_ >> Index) & Mask; }
        explicit operator bool() const { return value_ & (Mask << Index); }
        Bitfield &operator++()         { return *this = *this + 1; }
        T operator++(int)              { T r = *this; ++*this; return r; }
        Bitfield &operator--()         { return *this = *this - 1; }
        T operator--(int)              { T r = *this; --*this; return r; }

    private:
        T value_;
    };
    //gonna experiment and see which impl is better
} // namespace Common
