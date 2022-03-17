#pragma once

#include "emucore/common/types.h"

// hook functions for musashi

void MusashiHook_PulseReset();
int MusashiHook_IRQAcknowledge(s32 irq);
void MusashiHook_InstructionCallback(u32 pc);