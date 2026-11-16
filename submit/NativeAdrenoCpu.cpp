/* SPDX-License-Identifier: MIT */
#include "NativeAdrenoMemory.h"
#include <IOKit/IOLib.h>
#include <IOKit/IOLocks.h>
#include <kern/thread_call.h>
#include <vm/vm_map.h>
#include <mach/vm_param.h>
#include <kern/clock.h>
#include <vm/memory_types.h>
/* Darwin ARM exports the underscored routines, as arm/cpu_data.h uses. */
extern "C" void _disable_preemption(void);
extern "C" void _enable_preemption(void);
#define disable_preemption() _disable_preemption()
#define enable_preemption() _enable_preemption()
extern "C" vm_offset_t phystokv(uint64_t pa);
#if PONG_GUEST_22H374
extern "C" vm_map_address_t phystokv_range(uint64_t pa,vm_size_t *max_len);
#endif
extern "C" boolean_t pmap_valid_page(ppnum_t pn);
extern "C" void pmap_set_cache_attributes(ppnum_t pn,unsigned int cacheattr);
extern "C" unsigned int pmap_cache_attributes(ppnum_t pn);
/* Time and bytes of every CPU cache maintenance pass (the board page shows them per submission), and the pages user BOs made
 * write-combined: [0] ticks, [1] bytes, [2] longest pass in ticks, [3] passes, [4] write-combined pages now. */
extern "C" {volatile uint64_t native_adreno_cache_stats[5];}
extern "C" {volatile uint64_t native_adreno_cache_batches[3];}
/* Time spent checking and re-asserting write-combined page attributes (NativeAdrenoReassertWriteCombined), for the board page. */
extern "C" {volatile uint64_t native_adreno_reassert_ticks;}
static void noteMaintenance(uint64_t began,size_t bytes)
{
    const uint64_t ticks=mach_absolute_time()-began;
    __atomic_fetch_add(&native_adreno_cache_stats[0],ticks,__ATOMIC_RELAXED);
    __atomic_fetch_add(&native_adreno_cache_stats[1],bytes,__ATOMIC_RELAXED);
    __atomic_fetch_add(&native_adreno_cache_stats[3],1,__ATOMIC_RELAXED);
    uint64_t longest=__atomic_load_n(&native_adreno_cache_stats[2],__ATOMIC_RELAXED);
    while(ticks>longest && !__atomic_compare_exchange_n(&native_adreno_cache_stats[2],&longest,ticks,true,__ATOMIC_RELAXED,__ATOMIC_RELAXED)) {}
}

/* Darwin24 Pong owns discontiguous DRAM banks. gPhysSize describes only
 * the primary aperture; pmap_valid_page uses the actual managed bank list.
 * Validate every page before changing any attributes, including spans that
 * might otherwise cross a physical hole or a firmware reservation. */
static bool managedSpan(uint64_t physical,size_t bytes)
{
    if(!physical || !bytes || ((physical|bytes)&4095) || physical>UINT64_MAX-bytes)return false;
    for(size_t offset=0;offset<bytes;offset+=4096)
        if(!pmap_valid_page((ppnum_t)((physical+offset)>>12)))return false;
    return true;
}
size_t NativeAdrenoPrepareImportAliases(NativeAdrenoPhysicalSpan *spans,size_t count)
{
#if PONG_GUEST_22H374
    size_t pages=0;
    for(size_t i=0;i<count;++i) {
        auto &span=spans[i];
        if(!managedSpan(span.physical,span.bytes))continue;
        // Pong's managed RAM banks and direct-map translations are fixed at
        // boot. The prepared descriptor keeps these physical pages wired.
        vm_size_t length=span.bytes;
        const uintptr_t alias=phystokv_range(span.physical,&length);
        if(length!=span.bytes || !alias || (alias&4095) || span.bytes>UINTPTR_MAX-alias)continue;
        span.aperture=alias;
        pages+=span.bytes/4096;
    }
    return pages;
#else
    (void)spans;
    (void)count;
    return 0;
#endif
}
bool NativeAdrenoPagesUncached(const NativeAdrenoPhysicalSpan *spans,size_t count)
{
    if(!spans || !count)return false;
    for(size_t i=0;i<count;++i) {
        if(!spans[i].bytes || ((spans[i].physical|spans[i].bytes)&4095) ||
           spans[i].physical>UINT64_MAX-spans[i].bytes)return false;
        for(uint64_t offset=0;offset<spans[i].bytes;offset+=4096) {
            const ppnum_t pn=ppnum_t((spans[i].physical+offset)>>12);
            // Pong has multiple managed DRAM banks: gPhysSize alone must
            // never classify a cached high-bank page as device memory.
            if(pmap_valid_page(pn) && !(pmap_cache_attributes(pn)&VM_MEM_NOT_CACHEABLE))return false;
        }
    }
    return true;
}


IOReturn NativeAdrenoQueryCpu4K(NativeAdrenoCpuState *out)
{
    if(!out)return kIOReturnBadArgument;
    *out={};
    disable_preemption();
    __asm__ volatile("mrs %0, tcr_el1" : "=r"(out->tcr));
    __asm__ volatile("mrs %0, ttbr0_el1" : "=r"(out->ttbr0));
    __asm__ volatile("mrs %0, ttbr1_el1" : "=r"(out->ttbr1));
    __asm__ volatile("mrs %0, sctlr_el1" : "=r"(out->sctlr));
    out->kernel_page_bytes=page_size;
    out->kernel_map_page_bytes=vm_map_page_size(kernel_map);
    enable_preemption();
    bool native=PAGE_SIZE==4096 && out->kernel_page_bytes==4096 && out->kernel_map_page_bytes==4096 &&
        ((out->tcr>>14)&3)==0 && ((out->tcr>>30)&3)==2 &&
        (out->sctlr&1) && (out->ttbr1&UINT64_C(0x0000fffffffff000));
    return native?kIOReturnSuccess:kIOReturnUnsupported;
}
/* Select the operation once per contiguous virtual range. All callers
 * retain their validation and bounded on-core barriers. */
template <NativeAdrenoCacheOperation Operation>
static inline void maintainCacheLines(uintptr_t address, uintptr_t end, size_t line)
{
#pragma clang loop unroll_count(4)
    for (; address < end; address += line) {
        if (Operation == NativeAdrenoClean) {
            __asm__ volatile("dc cvac, %0" :: "r"(address) : "memory");
        } else if (Operation == NativeAdrenoCleanInvalidate) {
            __asm__ volatile("dc civac, %0" :: "r"(address) : "memory");
        } else {
            __asm__ volatile("dc ivac, %0" :: "r"(address) : "memory");
        }
    }
}

static inline void maintainCacheRange(uintptr_t address, uintptr_t end, size_t line,
    NativeAdrenoCacheOperation operation)
{
    switch (operation) {
    case NativeAdrenoClean:
        maintainCacheLines<NativeAdrenoClean>(address, end, line);
        break;
    case NativeAdrenoCleanInvalidate:
        maintainCacheLines<NativeAdrenoCleanInvalidate>(address, end, line);
        break;
    case NativeAdrenoInvalidate:
        maintainCacheLines<NativeAdrenoInvalidate>(address, end, line);
        break;
    }
}

static IOReturn maintainOwnedPages(void *mapping,size_t bytes,NativeAdrenoCacheOperation op)
{
    /* Whole owned pages avoid IVAC corruption of a neighbour's dirty line.
     * No Apple coherent-I/O shortcut is used for these noncoherent DMA paths.
     * Pin this CPU only for a bounded batch, with its own CTR and closing DSB.
     * The 256 MiB BO pool must not keep preemption disabled for the entire
     * maintenance pass (Darwin24's preemption watchdog caught that at boot).
     * Owned pages remain live across batches; a subsequent batch can safely
     * run on another CPU after the preceding core completes its operations. */
    if(!mapping || !bytes || (((uintptr_t)mapping|bytes)&4095) ||
        bytes>UINTPTR_MAX-(uintptr_t)mapping || op>NativeAdrenoInvalidate || op<NativeAdrenoClean)
        return kIOReturnBadArgument;
    uintptr_t address=(uintptr_t)mapping,end=address+bytes;
    constexpr size_t batch_bytes=16*4096;
    while(address<end) {
        const size_t remaining=end-address;
        const uintptr_t batch_end=address+(remaining<batch_bytes?remaining:batch_bytes);
        disable_preemption();
        uint64_t ctr;__asm__ volatile("mrs %0, ctr_el0":"=r"(ctr));
        const size_t line=(size_t)4<<((ctr>>16)&15);
        if(line>4096) {enable_preemption();return kIOReturnUnsupported;}
        __asm__ volatile("dsb sy":::"memory");
        maintainCacheRange(address, batch_end, line, op);
        address = batch_end;
        __asm__ volatile("dsb sy":::"memory");
        enable_preemption();
    }
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoMaintainOwnedPages(void *mapping,size_t bytes,NativeAdrenoCacheOperation op)
{
    const uint64_t began=mach_absolute_time();
    const IOReturn status=maintainOwnedPages(mapping,bytes,op);
    if(!status)noteMaintenance(began,bytes);
    return status;
}
IOReturn NativeAdrenoMaintainPhysicalPages(uint64_t physical,size_t bytes,NativeAdrenoCacheOperation op)
{
    if(!physical || !bytes || ((physical|bytes)&4095) || physical>UINT64_MAX-bytes || op>NativeAdrenoInvalidate || op<NativeAdrenoClean)
        return kIOReturnBadArgument;
    const uint64_t began=mach_absolute_time();
    /* One barrier pair per run of up to 64 pages instead of per page (an imported 3048x2032 surface is 5952 pages): preemption stays off
     * for a bounded run, and the closing DSB on the issuing core completes every operation of that run. */
    for(size_t offset=0;offset<bytes;) {
        disable_preemption();
        uint64_t ctr;__asm__ volatile("mrs %0, ctr_el0":"=r"(ctr));
        const size_t line=(size_t)4<<((ctr>>16)&15);
        if(line>4096) {enable_preemption();return kIOReturnUnsupported;}
        __asm__ volatile("dsb sy":::"memory");
        for(unsigned run=0;run<64 && offset<bytes;++run,offset+=4096) {
            /* A page outside the kernel's DRAM is device memory (the firmware framebuffer the display hand-off imports): the CPU maps it
             * uncached (kIOMapInhibitCache), no cache line of it exists to maintain, and phystokv would panic on it (int-080: illegal PA
             * 0xb8000000). */
            const uint64_t page=physical+offset;
            if(!pmap_valid_page((ppnum_t)(page>>12)))continue;
            const vm_offset_t alias=phystokv(page);
            if(!alias) {__asm__ volatile("dsb sy":::"memory");enable_preemption();return kIOReturnUnsupported;}
            maintainCacheRange(alias, alias + 4096, line, op);
        }
        __asm__ volatile("dsb sy":::"memory");
        enable_preemption();
    }
    noteMaintenance(began,bytes);
    return kIOReturnSuccess;
}
static constexpr size_t kPhysicalPageBytes=4096;
static constexpr size_t kPhysicalBatchBytes=64*kPhysicalPageBytes;

static IOReturn maintainPhysicalRange(const NativeAdrenoPhysicalSpan *spans,size_t count,
    size_t position,size_t end,NativeAdrenoCacheOperation op,uint64_t &batches)
{
    size_t at=0;
    while(at<count && position>=spans[at].offset+spans[at].bytes)++at;
    while(position<end) {
        // A fragmented IOSurface may have thousands of one-page spans.
        // Keep the same cache operations, but complete a bounded 64-page
        // batch across span boundaries, rather than one DSB pair per span.
        disable_preemption();uint64_t ctr;__asm__ volatile("mrs %0, ctr_el0":"=r"(ctr));
        const size_t line=(size_t)4<<((ctr>>16)&15);
        if(line>4096){enable_preemption();return kIOReturnUnsupported;}
        __asm__ volatile("dsb sy":::"memory");
        for(unsigned pages=0;pages<64 && position<end;++pages,position+=4096) {
            while(at<count && position>=spans[at].offset+spans[at].bytes)++at;
            const auto &span=spans[at];
            const size_t within=position-span.offset;
            vm_offset_t alias=span.aperture;
            if(alias) {
                alias+=within;
            } else {
                const uint64_t physical=span.physical+within;
                // Unknown apertures still take the per-page check, including
                // uncached firmware or spans crossing a direct-map boundary.
                if(!pmap_valid_page(ppnum_t(physical>>12)))continue;
                alias=phystokv(physical);
            }
            if(!alias){__asm__ volatile("dsb sy":::"memory");enable_preemption();return kIOReturnUnsupported;}
            maintainCacheRange(alias, alias + 4096, line, op);
        }
        __asm__ volatile("dsb sy":::"memory");enable_preemption();++batches;
    }
    return kIOReturnSuccess;
}

namespace {
struct ParallelCacheWork {
    const NativeAdrenoPhysicalSpan *spans;
    size_t count,start,end;
    NativeAdrenoCacheOperation operation;
    uint64_t batches=0;
    IOReturn status=kIOReturnNotReady;
    bool claimed=false,done=false;
};
IOLock *parallelCacheLock=nullptr;
thread_call_t parallelCacheCall=nullptr;
// Protected by parallelCacheLock. A caller keeps this stack record and
// the pinned pages alive until the helper signals after its final DSB.
ParallelCacheWork *parallelCacheWork=nullptr;

void maintainOnCacheWorker(thread_call_param_t,thread_call_param_t)
{
    IOLockLock(parallelCacheLock);
    auto *work=parallelCacheWork;
    if(!work || work->claimed) {
        IOLockUnlock(parallelCacheLock);
        return;
    }
    work->claimed=true;
    IOLockUnlock(parallelCacheLock);

    const auto status=maintainPhysicalRange(work->spans,work->count,work->start,work->end,
        work->operation,work->batches);
    IOLockLock(parallelCacheLock);
    work->status=status;
    work->done=true;
    IOLockWakeup(parallelCacheLock,&work->done,true);
    IOLockUnlock(parallelCacheLock);
}
}

bool NativeAdrenoStartParallelCacheWorker()
{
    auto *lock=IOLockAlloc();
    if(!lock)return false;
    const auto call=thread_call_allocate_with_options(maintainOnCacheWorker,nullptr,
        THREAD_CALL_PRIORITY_KERNEL_HIGH,THREAD_CALL_OPTIONS_ONCE);
    if(!call) {
        IOLockFree(lock);
        return false;
    }
    parallelCacheLock=lock;
    parallelCacheCall=call;
    return true;
}

static bool tryMaintainParallel(const NativeAdrenoPhysicalSpan *spans,size_t count,
    size_t offset,size_t bytes,NativeAdrenoCacheOperation op,uint64_t &batches,IOReturn &status,
    bool &workerUsed)
{
    if(!parallelCacheCall || !IOLockTryLock(parallelCacheLock))return false;
    if(parallelCacheWork) {
        IOLockUnlock(parallelCacheLock);
        return false;
    }
    const size_t split=offset+((bytes/2)/kPhysicalBatchBytes)*kPhysicalBatchBytes;
    ParallelCacheWork work{spans,count,split,offset+bytes,op};
    // Complete preceding CPU writes before issuing CMO from another core.
    __asm__ volatile("dsb sy":::"memory");
    parallelCacheWork=&work;
    (void)thread_call_enter(parallelCacheCall);
    IOLockUnlock(parallelCacheLock);

    status=maintainPhysicalRange(spans,count,offset,split,op,batches);
    IOLockLock(parallelCacheLock);
    if(!work.claimed) {
        // The worker has not started. Finish locally to bound dispatch delay.
        // A late callback sees no job (or a different caller's current job).
        parallelCacheWork=nullptr;
        IOLockUnlock(parallelCacheLock);
        const auto tail=maintainPhysicalRange(spans,count,split,offset+bytes,op,batches);
        if(!status)status=tail;
        return true;
    }
    // Even an error on the first half must drain the remote CMO.
    while(!work.done)IOLockSleep(parallelCacheLock,&work.done,THREAD_UNINT);
    parallelCacheWork=nullptr;
    IOLockUnlock(parallelCacheLock);
    workerUsed=true;
    batches+=work.batches;
    if(!status)status=work.status;
    return true;
}

IOReturn NativeAdrenoMaintainPhysicalSpans(const NativeAdrenoPhysicalSpan *spans,size_t count,size_t offset,size_t bytes,NativeAdrenoCacheOperation op,bool parallel)
{
    if(!spans || !count || !bytes || ((offset|bytes)&4095) || offset>SIZE_MAX-bytes ||
       op<NativeAdrenoClean || op>NativeAdrenoInvalidate)return kIOReturnBadArgument;
    uint64_t covered=0;
    for(size_t i=0;i<count;++i) {
        const auto &s=spans[i];
        if(s.offset!=covered || !s.physical || !s.bytes || ((s.physical|s.bytes)&4095) ||
           s.physical>UINT64_MAX-s.bytes || covered>UINT64_MAX-s.bytes)return kIOReturnBadArgument;
        covered+=s.bytes;
    }
    if(offset>covered || bytes>covered-offset)return kIOReturnBadArgument;
    const uint64_t began=mach_absolute_time();
    uint64_t batches=0;
    IOReturn status=kIOReturnSuccess;
    bool workerUsed=false;
    // Only prepared imports with validated, physically disjoint aperture spans
    // opt in. Other callers keep the single-lane cache walk.
    const bool attempted=parallel && bytes>=16u*1024u*1024u &&
        tryMaintainParallel(spans,count,offset,bytes,op,batches,status,workerUsed);
    if(!attempted)status=maintainPhysicalRange(spans,count,offset,offset+bytes,op,batches);
    if(status)return status;
    __atomic_fetch_add(&native_adreno_cache_batches[0],1,__ATOMIC_RELAXED);
    __atomic_fetch_add(&native_adreno_cache_batches[1],batches,__ATOMIC_RELAXED);
    __atomic_fetch_add(&native_adreno_cache_batches[2],bytes/4096,__ATOMIC_RELAXED);
    noteMaintenance(began,bytes);
    if(attempted) {
        static uint64_t attempts,assisted;
        const uint64_t sample=__atomic_add_fetch(&attempts,1,__ATOMIC_RELAXED);
        if(workerUsed)__atomic_fetch_add(&assisted,1,__ATOMIC_RELAXED);
        if(sample<=6 || (sample%1024)==0) {
            uint64_t elapsed=0;
            absolutetime_to_nanoseconds(mach_absolute_time()-began,&elapsed);
            IOLog("PONG GPU CACHE WORKER: samples=%llu assisted=%llu bytes=%zu worker=%u elapsed_us=%llu\n",
                (unsigned long long)sample,(unsigned long long)__atomic_load_n(&assisted,__ATOMIC_RELAXED),
                bytes,workerUsed?1u:0u,(unsigned long long)(elapsed/1000));
        }
    }
    return kIOReturnSuccess;
}
/* A user BO's pages become Normal non-cacheable (write-combined) for every CPU mapping the pmap tracks: the kernel's own mapping and each
 * client view. The views did not get the cache mode their mapping asked for (int-070: Turnip's write-combine view left IB words in a CPU
 * cache line), but they do take the page's own attribute. That is the Linux msm model for its WC BOs: CPU writes reach DRAM, the GPU's
 * non-cacheable reads see them and CPU reads see GPU writes without per-submission maintenance. The physical aperture keeps its cacheable
 * block mapping on this kernel (no __ARM_PTE_PHYSMAP__); nothing writes through it, and the caller cleans and invalidates the pages first.
 * Before the pages go back to the VM they get the default attribute again. */
IOReturn NativeAdrenoSetPagesCacheMode(const NativeAdrenoPhysicalSpan *spans,size_t count,bool write_combined)
{
    if(!spans || !count)return kIOReturnBadArgument;
    uint64_t pages=0;
    for(size_t i=0;i<count;++i) {
        const NativeAdrenoPhysicalSpan &span=spans[i];
        if(!managedSpan(span.physical,span.bytes))return kIOReturnBadArgument;
        pages+=span.bytes/4096;
    }
    for(size_t i=0;i<count;++i)
        for(uint64_t offset=0;offset<spans[i].bytes;offset+=4096)
            pmap_set_cache_attributes((ppnum_t)((spans[i].physical+offset)>>12),write_combined?VM_WIMG_WCOMB:VM_WIMG_USE_DEFAULT);
    if(write_combined)__atomic_fetch_add(&native_adreno_cache_stats[4],pages,__ATOMIC_RELAXED);
    else __atomic_fetch_sub(&native_adreno_cache_stats[4],pages,__ATOMIC_RELAXED);
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoReassertWriteCombined(const NativeAdrenoPhysicalSpan *spans,size_t count,uint32_t *reset)
{
    if(!spans || !count || !reset)return kIOReturnBadArgument;
    const uint64_t began=mach_absolute_time();
    for(size_t i=0;i<count;++i) {
        const NativeAdrenoPhysicalSpan &span=spans[i];
        if(!managedSpan(span.physical,span.bytes))return kIOReturnBadArgument;
        for(uint64_t offset=0;offset<span.bytes;offset+=4096) {
            const ppnum_t pn=(ppnum_t)((span.physical+offset)>>12);
            /* From the default attribute the pmap also cleans and invalidates the page (pmap_sync_wimg), so lines a cacheable view
             * dirtied reach DRAM before the GPU reads it. */
            if((pmap_cache_attributes(pn)&VM_WIMG_MASK)!=VM_WIMG_WCOMB) {pmap_set_cache_attributes(pn,VM_WIMG_WCOMB);++*reset;}
        }
    }
    __atomic_fetch_add(&native_adreno_reassert_ticks,mach_absolute_time()-began,__ATOMIC_RELAXED);
    return kIOReturnSuccess;
}
