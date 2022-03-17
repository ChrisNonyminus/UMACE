#pragma once

#include <string>
#include <sstream>
#include <stdint.h>

#include <boost/endian/conversion.hpp>

using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;

using s8 = int8_t;
using s16 = int16_t;
using s32 = int32_t;
using s64 = int64_t;

// https://stackoverflow.com/questions/2782725/converting-float-values-from-big-endian-to-little-endian

template <typename T>
inline T bswap(T val) {
    T retVal;
    char* pVal = (char*)&val;
    char* pRetVal = (char*)&retVal;
    int size = sizeof(T);
    for (int i = 0; i < size; i++) {
        pRetVal[size - 1 - i] = pVal[i];
    }

    return retVal;
}

inline u16 bswap_16(u16 x) {
    return boost::endian::endian_reverse(x);
}

inline u32 bswap_32(u32 x) {
    return boost::endian::endian_reverse(x);
}

inline u64 bswap_64(u64 x) {
    return boost::endian::endian_reverse(x);
}

inline float bswap_f32(float x) {
    return bswap<float>(x);
}

inline double bswap_f64(double x) {
    return bswap<double>(x);
}

#define KILO(x) (x * 1000)
#define MEGA(x) (KILO(x) * 1000)
#define GIGA(x) (MEGA(x) * 1000)


#define KB(x) (x * 1024)
#define MB(x) (KB(x) * 1024)
#define GB(x) (MB(x) * 1024)