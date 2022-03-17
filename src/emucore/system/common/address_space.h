#pragma once

#include <map>
#include <string>

#include "memory.h"
#include "hw/io.h"

namespace Core {
    class AddressSpace : public std::enable_shared_from_this<AddressSpace> {
    private:
        std::map<std::string, std::shared_ptr<Memory>> memory_regions;
        std::map<std::string, std::shared_ptr<IOComponent>> io_regions;
        u64 size;
        bool DoesMemoryRegionOverlap(u64 start_addr, u64 size) {
            for (auto& region : memory_regions) {
                if (region.second->GetStartAddress() <= start_addr &&
                    region.second->GetStartAddress() + region.second->GetSize() > start_addr + size) {
                    return true;
                }
            }
            for (auto& region : io_regions) {
                if (region.second->GetStartAddress() <= start_addr &&
                    region.second->GetStartAddress() + region.second->GetSize() > start_addr + size) {
                    return true;
                }
            }
            return false;
        }
    public:
        AddressSpace(u64 _size) : size(_size) {}
        void AddMemoryRegion(const std::string& name, std::shared_ptr<Memory> memory, u64 start_addr) {
            memory_regions[name] = memory;
            if (DoesMemoryRegionOverlap(start_addr, memory->GetSize())) {
                throw std::runtime_error("Memory region overlaps with existing region");
            }
            memory->SetStartAddress(start_addr);
        }
        void AddIORegion(const std::string& name, std::shared_ptr<IOComponent> io, u64 start_addr, u64 size) {
            io_regions[name] = io;
            if (DoesMemoryRegionOverlap(start_addr, io->GetSize())) {
                throw std::runtime_error("IO region overlaps with existing region");
            }
            io->SetStartAddress(start_addr);
            io->SetSize(size);
        }
        
        std::shared_ptr<Memory> GetMemoryRegion(const std::string& name) {
            return memory_regions[name];
        }

        std::shared_ptr<IOComponent> GetIORegion(const std::string& name) {
            return io_regions[name];
        }

        u64 GetSize() { return size; }

        u8 Read8(u64 addr) {
            for (auto& region : memory_regions) {
                if (addr >= region.second->GetStartAddress() && addr < region.second->GetStartAddress() + region.second->GetSize()) {
                    return region.second->Read8(addr - region.second->GetStartAddress());
                }
            }
            for (auto& region : io_regions) {
                if (addr >= region.second->GetStartAddress() && addr < region.second->GetStartAddress() + region.second->GetSize()) {
                    return region.second->Read8(addr - region.second->GetStartAddress());
                }
            }
            return 0;
        }
        
        u16 Read16(u64 addr) {
            for (auto& region : memory_regions) {
                if (addr >= region.second->GetStartAddress() && addr < region.second->GetStartAddress() + region.second->GetSize()) {
                    return region.second->Read16(addr - region.second->GetStartAddress());
                }
            }
            for (auto& region : io_regions) {
                if (addr >= region.second->GetStartAddress() && addr < region.second->GetStartAddress() + region.second->GetSize()) {
                    return region.second->Read16(addr - region.second->GetStartAddress());
                }
            }
            return 0;
        }

        u32 Read32(u64 addr) {
            for (auto& region : memory_regions) {
                if (addr >= region.second->GetStartAddress() && addr < region.second->GetStartAddress() + region.second->GetSize()) {
                    return region.second->Read32(addr - region.second->GetStartAddress());
                }
            }
            for (auto& region : io_regions) {
                if (addr >= region.second->GetStartAddress() && addr < region.second->GetStartAddress() + region.second->GetSize()) {
                    return region.second->Read32(addr - region.second->GetStartAddress());
                }
            }
            return 0;
        }

        void Write8(u64 addr, u8 value) {
            for (auto& region : memory_regions) {
                if (addr >= region.second->GetStartAddress() && addr < region.second->GetStartAddress() + region.second->GetSize()) {
                    region.second->Write8(addr - region.second->GetStartAddress(), value);
                }
            }
            for (auto& region : io_regions) {
                if (addr >= region.second->GetStartAddress() && addr < region.second->GetStartAddress() + region.second->GetSize()) {
                    region.second->Write8(addr - region.second->GetStartAddress(), value);
                }
            }
        }

        void Write16(u64 addr, u16 value) {
            for (auto& region : memory_regions) {
                if (addr >= region.second->GetStartAddress() && addr < region.second->GetStartAddress() + region.second->GetSize()) {
                    region.second->Write16(addr - region.second->GetStartAddress(), value);
                }
            }
            for (auto& region : io_regions) {
                if (addr >= region.second->GetStartAddress() && addr < region.second->GetStartAddress() + region.second->GetSize()) {
                    region.second->Write16(addr - region.second->GetStartAddress(), value);
                }
            }
        }

        void Write32(u64 addr, u32 value) {
            for (auto& region : memory_regions) {
                if (addr >= region.second->GetStartAddress() && addr < region.second->GetStartAddress() + region.second->GetSize()) {
                    region.second->Write32(addr - region.second->GetStartAddress(), value);
                }
            }
            for (auto& region : io_regions) {
                if (addr >= region.second->GetStartAddress() && addr < region.second->GetStartAddress() + region.second->GetSize()) {
                    region.second->Write32(addr - region.second->GetStartAddress(), value);
                }
            }
        }

        float ReadFloat(u64 addr) {
            for (auto& region : memory_regions) {
                if (addr >= region.second->GetStartAddress() && addr < region.second->GetStartAddress() + region.second->GetSize()) {
                    return region.second->ReadFloat(addr - region.second->GetStartAddress());
                }
            }
            for (auto& region : io_regions) {
                if (addr >= region.second->GetStartAddress() && addr < region.second->GetStartAddress() + region.second->GetSize()) {
                    return region.second->ReadFloat(addr - region.second->GetStartAddress());
                }
            }
            return 0.0f;
        }

        void WriteFloat(u64 addr, float value) {
            for (auto& region : memory_regions) {
                if (addr >= region.second->GetStartAddress() && addr < region.second->GetStartAddress() + region.second->GetSize()) {
                    region.second->WriteFloat(addr - region.second->GetStartAddress(), value);
                }
            }
            for (auto& region : io_regions) {
                if (addr >= region.second->GetStartAddress() && addr < region.second->GetStartAddress() + region.second->GetSize()) {
                    region.second->WriteFloat(addr - region.second->GetStartAddress(), value);
                }
            }
        }

        double ReadDouble(u64 addr) {
            for (auto& region : memory_regions) {
                if (addr >= region.second->GetStartAddress() && addr < region.second->GetStartAddress() + region.second->GetSize()) {
                    return region.second->ReadDouble(addr - region.second->GetStartAddress());
                }
            }
            for (auto& region : io_regions) {
                if (addr >= region.second->GetStartAddress() && addr < region.second->GetStartAddress() + region.second->GetSize()) {
                    return region.second->ReadDouble(addr - region.second->GetStartAddress());
                }
            }
            return 0.0;
        }

        void WriteDouble(u64 addr, double value) {
            for (auto& region : memory_regions) {
                if (addr >= region.second->GetStartAddress() && addr < region.second->GetStartAddress() + region.second->GetSize()) {
                    region.second->WriteDouble(addr - region.second->GetStartAddress(), value);
                }
            }
            for (auto& region : io_regions) {
                if (addr >= region.second->GetStartAddress() && addr < region.second->GetStartAddress() + region.second->GetSize()) {
                    region.second->WriteDouble(addr - region.second->GetStartAddress(), value);
                }
            }
        }

        const char* ReadCString(u64 addr) {
            for (auto& region : memory_regions) {
                if (addr >= region.second->GetStartAddress() && addr < region.second->GetStartAddress() + region.second->GetSize()) {
                    return region.second->ReadCString(addr - region.second->GetStartAddress());
                }
            }
            for (auto& region : io_regions) {
                if (addr >= region.second->GetStartAddress() && addr < region.second->GetStartAddress() + region.second->GetSize()) {
                    return region.second->ReadCString(addr - region.second->GetStartAddress());
                }
            }
            return "";
        }

        void WriteCString(u64 addr, const char* value) {
            for (auto& region : memory_regions) {
                if (addr >= region.second->GetStartAddress() && addr < region.second->GetStartAddress() + region.second->GetSize()) {
                    region.second->WriteCString(addr - region.second->GetStartAddress(), value);
                }
            }
            for (auto& region : io_regions) {
                if (addr >= region.second->GetStartAddress() && addr < region.second->GetStartAddress() + region.second->GetSize()) {
                    region.second->WriteCString(addr - region.second->GetStartAddress(), value);
                }
            }
        }
    };
} // namespace Core
