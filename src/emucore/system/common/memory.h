#pragma once

#include <memory>
#include <atomic>
#include <string>
#include <vector>
#include <sstream>
#include <string.h>

#include "emucore/common/types.h"
#include "emucore/common/logging.h"

namespace Core
{
    // https://github.com/RPCS3/rpcs3/blob/ba1699a8312bf8b4e250452db01d00ff16616f03/rpcs3/Emu/Memory/vm.cpp
    class Memory : public std::enable_shared_from_this<Memory> {
    public:
        Memory(u64 size, u64 pagesize = 0x1000, bool big_endian = false) {
            this->size = size;
            mem.resize(size);
            this->page_size = pagesize;
            this->big_endian = big_endian;
            fprintf(log_file, "Memory is allocated at %p\n", mem.data());
            if (page_size) {
                page_table.resize(size / page_size);
                page_table[0] = (page_size);
                for (size_t i = 1; i < size / page_size; i++) {
                    page_table[i] = (0);
                }
            }
        }
        ~Memory() {
            mem.clear();
        }

        void SetStartAddress(u64 addr) {
            start_addr = addr;
        }

        u64 GetStartAddress() {
            return start_addr;
        }

        u64 GetSize() { return size; }

        bool Allocate(u32 address, u8 flags, u32 size) {
            for (u32 i = address / page_size; i <= (address + size - 1) / page_size; i++) {
                if (page_table[i] != 0) {
                    return false;
                }
            }
            for (u32 i = address / page_size; i <= (address + size - 1) / page_size; i++) {
                page_table[i] = (size > page_size ? page_size : size);
            }
            return true;
        }

        u32 AllocateWithHint(u32 address, u32 size, u32 alignment) {
            u32 start_page_index = 0;
            for (u32 i = address / page_size; i < (size - 1) / page_size; i++) {
                if (page_table[i] == 0) {
                    start_page_index = i;
                    break;
                }
            }
            for (u32 addr = address; addr < 0x1'0000'0000; addr += alignment) {
                if (Allocate(addr, 1, size) == true) {
                    return addr;
                }
            }
            return 0;
        }

        u32 Allocate(u32 size, u32 alignment) {
            u32 start_page_index = 0;
            for (u32 i = 0; i < (size - 1) / page_size; i++) {
                if (page_table[i] == 0) {
                    start_page_index = i;
                    break;
                }
            }
            for (u32 address = start_page_index * page_size; address < 0x1'0000'0000; address += alignment) {
                if (Allocate(address, 1, size) == true) {
                    return address;
                }
            }
            return 0;
        }

        void Free(u32 address) {
            for (u32 i = address / page_size; i <= (address + page_table[address / page_size] - 1) / page_size; i++) {
                page_table[i] = (0);
            }
        }

        u8* GetPtr() {
            return mem.data();
        }

        template <typename T>
        T Read(u32 addr)
        {
            return *(T*)(mem.data() + addr);
        }

        template <typename T>
        void Write(u32 addr, T val)
        {
            *(T*)(mem.data() + addr) = val;
        }

        u8 Read8(u32 addr) {
            return Read<u8>(addr);
        }

        u16 Read16(u32 addr) {
            return big_endian ? bswap_16(Read<u16>(addr)) : Read<u16>(addr);
        }

        u32 Read32(u32 addr) {
            return big_endian ? bswap_32(Read<u32>(addr)) : Read<u32>(addr);
        }

        void Write8(u32 addr, u8 val) {
            Write<u8>(addr, val);
        }

        void Write16(u32 addr, u16 val) {
            if (big_endian) {
                Write<u16>(addr, bswap_16(val));
            } else {
                Write<u16>(addr, val);
            }
        }

        void Write32(u32 addr, u32 val) {
            if (big_endian) {
                Write<u32>(addr, bswap_32(val));
            } else {
                Write<u32>(addr, val);
            }
        }

        float ReadFloat(u32 addr) {
            return big_endian ? bswap_f32(Read<float>(addr)) : Read<float>(addr);
        }

        void WriteFloat(u32 addr, float val) {
            if (big_endian) {
                Write<float>(addr, bswap_f32(val));
            } else {
                Write<float>(addr, val);
            }
        }

        double ReadDouble(u32 addr) {
            return big_endian ? bswap_f64(Read<double>(addr)) : Read<double>(addr);
        }

        void WriteDouble(u32 addr, double val) {
            if (big_endian) {
                Write<double>(addr, bswap_f64(val));
            } else {
                Write<double>(addr, val);
            }
        }

        void MemSet(u32 addr, u8 byte, u32 count) {
            memset(mem.data() + addr, byte, count);
        }

        template <typename T>
        T* ReadSpan(u32 addr, size_t size)
        {
            T* span = (T*)malloc(size);
            memcpy(span, mem.data() + addr, size);
            return span;
        }

        template <typename T>
        void WriteSpan(u32 addr, T* span, size_t size)
        {
            memcpy(mem.data() + addr, span, size);
        }

        void* ReadBuffer(u32 addr, size_t size) {
            void* buffer = malloc(size);
            memcpy(buffer, mem.data() + addr, size);
            return buffer;
        }

        void ReadBuffer(u32 addr, void* buffer, size_t size) {
            memcpy(buffer, mem.data() + addr, size);
        }

        void WriteBuffer(u32 addr, void* buffer, size_t size) {
            memcpy(mem.data() + addr, buffer, size);
        }

        const char* ReadCString(u32 addr) {
            return (const char*)(mem.data() + addr);
        }

        void WriteCString(u32 addr, const char* str) {
            while (*str != '\0') {
                Write<char>(addr, *str);
                addr++;
                str++;
            }
            Write<char>(addr, '\0');
        }

        std::vector<u16> page_table;
        
    private:
        std::vector<u8> mem;
        u64 size;
        u64 page_size;
        u64 start_addr;
        bool big_endian = false;
    };
} // namespace Core
