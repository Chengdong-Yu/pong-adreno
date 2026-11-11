/* SPDX-License-Identifier: MIT */
#pragma once
#include <IOKit/IOTypes.h>
class NativeAdrenoVm;
struct PongProcessVm;
// Page allocation/preparation must run outside the shared GPU gate.
IOReturn PongGpuPrepareProcessVm(PongProcessVm **);
void PongGpuDiscardPreparedProcessVm(PongProcessVm *);
// Serialized by the actual published GPU gate. Consumes the prepared table
// only after reserving a real ASID. No speculative CP preemption.
IOReturn PongGpuCreateProcessVm(PongProcessVm **prepared,PongProcessVm **,NativeAdrenoVm **);
// Detach/retire under the gate, then release the returned table backing
// with DiscardPreparedProcessVm outside it.
IOReturn PongGpuDestroyProcessVm(PongProcessVm *,PongProcessVm **retired);
IOReturn PongGpuDeactivateProcessVm(PongProcessVm *);
IOReturn PongGpuSelectProcessVm(NativeAdrenoVm *);
bool PongGpuProcessSwitchPending();
// Poll only under the gate; Busy permits dropping it. Completed means old
// GPU writes are drained, not that a new page table has been installed.
IOReturn PongGpuPollProcessDrain();
