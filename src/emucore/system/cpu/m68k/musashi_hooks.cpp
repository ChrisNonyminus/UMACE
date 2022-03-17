#include "musashi_hooks.h"
#include "m68k_defs.h"

namespace Core {

    System* m68k_system; 
    std::shared_ptr<AddressSpace> musashi_address_space;
    u32 musashi_clock_speed;
    void InitMusashi(std::shared_ptr<AddressSpace> address_space,
                    System* m68k_system_, int cpu_type) {
        musashi_address_space = address_space;
        m68k_system = m68k_system_;
        m68k_init();
        m68k_set_cpu_type(cpu_type);
        m68k_pulse_reset();
        m68k_set_int_ack_callback(MusashiHook_IRQAcknowledge);
    }
} // namespace Core


unsigned int  m68k_read_memory_8(unsigned int address) {
    return Core::musashi_address_space->Read8(address);
}
unsigned int  m68k_read_memory_16(unsigned int address) {
    return Core::musashi_address_space->Read16(address);
}
unsigned int  m68k_read_memory_32(unsigned int address) {
    return Core::musashi_address_space->Read32(address);
}
void m68k_write_memory_8(unsigned int address, unsigned int value) {
    Core::musashi_address_space->Write8(address, value);
}
void m68k_write_memory_16(unsigned int address, unsigned int value) {
    Core::musashi_address_space->Write16(address, value);
}
void m68k_write_memory_32(unsigned int address, unsigned int value) {
    Core::musashi_address_space->Write32(address, value);
}

void MusashiHook_PulseReset() {
    Core::m68k_system->Stop();
}
int MusashiHook_IRQAcknowledge(s32 irq) {
    return Core::m68k_system->IRQ(irq);
}
void MusashiHook_InstructionCallback(u32 pc) {
}