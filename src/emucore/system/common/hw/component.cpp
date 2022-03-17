#include "component.h"
#include "io.h"
#include "../system.h"


namespace Core {
    HardwareComponent::HardwareComponent(System* system)
     : system(system) {}
    void HardwareComponent::SetInterrupt(int interrupt_id) {
        system->SetIRQ(interrupt_id ? interrupt_id : system->GetFreeIRQ(), shared_from_this());
    }
    IOComponent::IOComponent(System* system) : HardwareComponent(system) {}
}