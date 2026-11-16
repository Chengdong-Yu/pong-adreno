/* SPDX-License-Identifier: MIT */
#include "NativeAdrenoMemory.h"
#include "NativeAdrenoVmAlias.h"
/* Where a call that never returns is stuck: the free/unmap/release path stores a stage number here, the board page shows it. */
extern "C" {volatile uint32_t native_adreno_trace_stage;}
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOLib.h>
#include <kern/task.h>
#include <kern/clock.h>
#include <pexpert/pexpert.h>
extern "C" {volatile uint32_t native_adreno_bo_contig[2];}

static bool outputWidth(unsigned bits)
{return bits==32 || bits==36 || bits==40 || bits==42 || bits==44 || bits==48;}
/* Where the last import stopped (10..19, 19 = done), and for a refused page the physical address and offset: the board log shows them. */
enum : IOReturn {kNativeAdrenoProtectionFailure=2};   /* KERN_PROTECTION_FAILURE, as prepare() reports a range that cannot be wired for writing */
extern "C" {volatile uint32_t native_adreno_pool_mib=0,native_adreno_pool_status=0;volatile uint32_t native_adreno_import_stage;volatile uint64_t native_adreno_import_fail_pa,native_adreno_import_fail_offset;}
/* The pool (see reservePool): a free list of page extents over one contiguous block; best fit, neighbours merged on release. */
extern "C" {volatile uint32_t native_adreno_pool_stats[8];}
/* The pool's write-combine (writeCombinePool): bit 31 once tried, the low bits its status. */
extern "C" {volatile uint32_t native_adreno_pool_wc;}   /* [0] pool MiB, [1] BOs from the pool, [2] fell back, [3] MiB in use, [4] peak MiB, [5] scattered BOs, [6] their spans, [7] import spans */
namespace {
// All allocation/map/free calls are serialized by the existing resource gate.
// Track the whole pool for its lifetime; scattered BOs until their real free.
// This replaces comparing every imported page to every scattered BO span.
#if PONG_GUEST_22H374
extern "C" uint64_t pong_ram_bytes;
extern "C" unsigned pmap_pong_pa_index(uint64_t);
#else
extern "C" uint64_t gPhysBase,gPhysSize;
uint64_t ownedBase,ownedBytes;
#endif
uint64_t *ownedBits;
uint64_t ownedPages;

uint64_t *ownedMap()
{
    static bool tried;
#if PONG_GUEST_22H374
    // gPhysSize covers only the primary bank on Darwin24 Pong.
    const uint64_t ramBytes=pong_ram_bytes;
#else
    const uint64_t ramBytes=gPhysSize;
#endif
    if(!tried && ramBytes) {
        tried=true;
        const uint64_t pages=(ramBytes+4095)/4096;
        const uint64_t bytes=((pages+63)/64)*8;
        if(bytes && bytes<=16u*1024u*1024u) {
            ownedBits=static_cast<uint64_t *>(IOMallocZero(size_t(bytes)));
            if(ownedBits) {
                ownedPages=pages;
#if !PONG_GUEST_22H374
                ownedBase=gPhysBase;
                ownedBytes=gPhysSize;
#endif
            }
        }
    }
    return ownedBits;
}

uint64_t ownedPageIndex(uint64_t physical)
{
#if PONG_GUEST_22H374
    // XNU maps all owned banks to dense indices and rejects physical holes.
    return pmap_pong_pa_index(physical);
#else
    if(physical<ownedBase || physical-ownedBase>=ownedBytes)return UINT64_MAX;
    return (physical-ownedBase)/4096;
#endif
}
void noteOwned(const NativeAdrenoPhysicalSpan *spans,size_t count,bool owned)
{
    uint64_t *bits=ownedMap();if(!bits)return;
    for(size_t i=0;i<count;++i)for(uint64_t offset=0;offset<spans[i].bytes;offset+=4096) {
        const uint64_t at=ownedPageIndex(spans[i].physical+offset);
        if(at>=ownedPages)continue;
        const uint64_t bit=UINT64_C(1)<<(at%64);
        if(owned)__atomic_fetch_or(&bits[at/64],bit,__ATOMIC_RELAXED);
        else __atomic_fetch_and(&bits[at/64],~bit,__ATOMIC_RELAXED);
    }
}
struct PoolExtent {size_t offset,bytes;};
struct Pool {
    IOBufferMemoryDescriptor *descriptor=nullptr;
    unsigned char *virt=nullptr;uint64_t physical=0;size_t bytes=0,used=0,peak=0;
    PoolExtent free[512];unsigned count=0;
    bool write_combined=false;
} pool;
bool poolTake(size_t bytes,size_t *offset) {
    int best=-1;
    for(unsigned i=0;i<pool.count;++i)if(pool.free[i].bytes>=bytes && (best<0 || pool.free[i].bytes<pool.free[best].bytes))best=(int)i;
    if(best<0)return false;
    *offset=pool.free[best].offset;
    pool.free[best].offset+=bytes;pool.free[best].bytes-=bytes;
    if(!pool.free[best].bytes) {for(unsigned i=(unsigned)best;i+1<pool.count;++i)pool.free[i]=pool.free[i+1];--pool.count;}
    pool.used+=bytes;if(pool.used>pool.peak)pool.peak=pool.used;
    native_adreno_pool_stats[3]=(uint32_t)(pool.used>>20);native_adreno_pool_stats[4]=(uint32_t)(pool.peak>>20);
    return true;
}
void poolGive(size_t offset,size_t bytes) {
    unsigned at=0;while(at<pool.count && pool.free[at].offset<offset)++at;
    if(pool.count>=512) {pool.used-=bytes;return;}   /* cannot happen with bounded fragmentation; the extent is lost, not reused */
    for(unsigned i=pool.count;i>at;--i)pool.free[i]=pool.free[i-1];
    pool.free[at]={offset,bytes};++pool.count;
    if(at+1<pool.count && pool.free[at].offset+pool.free[at].bytes==pool.free[at+1].offset) {
        pool.free[at].bytes+=pool.free[at+1].bytes;
        for(unsigned i=at+1;i+1<pool.count;++i)pool.free[i]=pool.free[i+1];
        --pool.count;
    }
    if(at && pool.free[at-1].offset+pool.free[at-1].bytes==pool.free[at].offset) {
        pool.free[at-1].bytes+=pool.free[at].bytes;
        for(unsigned i=at;i+1<pool.count;++i)pool.free[i]=pool.free[i+1];
        --pool.count;
    }
    pool.used-=bytes;native_adreno_pool_stats[3]=(uint32_t)(pool.used>>20);
}
}
bool NativeAdrenoMemory::ownsAnyPage(const NativeAdrenoPhysicalSpan *spans,size_t count)
{
    const uint64_t *bits=ownedMap();if(!bits || !spans || !count)return true;
    for(size_t i=0;i<count;++i)for(uint64_t offset=0;offset<spans[i].bytes;offset+=4096) {
        const uint64_t at=ownedPageIndex(spans[i].physical+offset);
        if(at>=ownedPages)return true;
        if((__atomic_load_n(&bits[at/64],__ATOMIC_RELAXED)>>(at%64))&1)return true;
    }
    return false;
}
size_t NativeAdrenoMemory::poolBytes() {return pool.bytes;}
uint64_t NativeAdrenoMemory::poolPhysical() {return pool.physical;}
/* int-152: a user BO's pages changed attribute one pmap_set_cache_attributes call at a time (each walks the page's mappings, flushes the
 * TLB and cleans the page), on allocation and again on release, under the connection lock: 3 ms a BO on average and 90-130 ms for a
 * surface-sized one, while every other GPU call of every client waited. The pool is the BOs' own block and nothing else maps it, so
 * its pages take the attribute once here and keep it while the pool lives. */
IOReturn NativeAdrenoMemory::writeCombinePool()
{
    if(!pool.descriptor || !pool.virt || !pool.bytes)return kIOReturnNotReady;
    if(pool.write_combined)return kIOReturnSuccess;
    IOReturn status=NativeAdrenoMaintainOwnedPages(pool.virt,pool.bytes,NativeAdrenoCleanInvalidate);
    if(status)return status;
    const NativeAdrenoPhysicalSpan span={0,pool.physical,pool.bytes};
    status=NativeAdrenoSetPagesCacheMode(&span,1,true);
    if(status)return status;
    pool.write_combined=true;return kIOReturnSuccess;
}
IOReturn NativeAdrenoMemory::reservePool(unsigned bits,size_t bytes)
{
    if(pool.descriptor)return kIOReturnBusy;
    if(!bytes || (bytes&4095) || !outputWidth(bits))return kIOReturnBadArgument;
    NativeAdrenoCpuState cpu{};
    IOReturn status=NativeAdrenoQueryCpu4K(&cpu);if(status!=kIOReturnSuccess)return status;
    const uint64_t limit=UINT64_C(1)<<bits;
    IOBufferMemoryDescriptor *descriptor=IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
        static_cast<IOOptionBits>(kIODirectionInOut)|static_cast<IOOptionBits>(kIOMemoryMapperNone)|kIOMemoryPhysicallyContiguous,
        bytes,(limit-1)&~UINT64_C(4095));
    if(!descriptor)return kIOReturnNoMemory;
    status=descriptor->prepare();
    if(status!=kIOReturnSuccess) {descriptor->release();return status;}
    IOByteCount length=0,tail=0;
    const addr64_t first=descriptor->getPhysicalSegment(0,&length,kIOMemoryMapperNone);
    const addr64_t last=descriptor->getPhysicalSegment(bytes-4096,&tail,kIOMemoryMapperNone);
    unsigned char *virt=static_cast<unsigned char *>(descriptor->getBytesNoCopy());
    if(!first || (first&4095) || !virt || ((uintptr_t)virt&4095) || last!=first+bytes-4096 || first>=limit || bytes>limit-first ||
        descriptor->getLength()!=bytes) {
        (void)descriptor->complete();descriptor->release();return kIOReturnUnsupported;
    }
    pool.descriptor=descriptor;pool.virt=virt;pool.physical=first;pool.bytes=bytes;pool.used=pool.peak=0;
    pool.free[0]={0,bytes};pool.count=1;
    const NativeAdrenoPhysicalSpan owned={0,first,bytes};noteOwned(&owned,1,true);
    native_adreno_pool_stats[0]=(uint32_t)(bytes>>20);
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoMemory::allocate(size_t bytes,unsigned bits,bool contiguous,size_t budget,bool pooled)
{
    if(descriptor_ || spans_ || quarantined_)return kIOReturnBusy;
    if(!bytes || (bytes&4095) || !outputWidth(bits) || !budget || budget>256u*1024u*1024u)
        return kIOReturnBadArgument;
    if(bytes>budget)return kIOReturnNoResources;
    NativeAdrenoCpuState cpu{};
    IOReturn status=NativeAdrenoQueryCpu4K(&cpu);if(status!=kIOReturnSuccess)return status;
    uint64_t limit=UINT64_C(1)<<bits;
    if(pooled && pool.descriptor && bytes<=pool.bytes) {
        size_t offset=0;
        if(poolTake(bytes,&offset)) {
            const uint64_t physical=pool.physical+offset;
            if(physical>=limit || bytes>limit-physical) {poolGive(offset,bytes);}
            else {
                spans_=IONewData(NativeAdrenoPhysicalSpan,1);
                if(!spans_) {poolGive(offset,bytes);return kIOReturnNoMemory;}
                span_capacity_=1;span_count_=1;spans_[0]={0,physical,bytes};
                mapping_=pool.virt+offset;bytes_=bytes;pooled_=true;pool_offset_=offset;prepared_=true;
                bzero(mapping_,bytes);
                /* Through a write-combined pool's own mapping the zeros go to DRAM; the barrier completes them. */
                if(pool.write_combined)__asm__ volatile("dsb sy":::"memory");
                else status=NativeAdrenoMaintainOwnedPages(mapping_,bytes,NativeAdrenoClean);
                if(status!=kIOReturnSuccess) {(void)release();return status;}
                native_adreno_pool_stats[1]=native_adreno_pool_stats[1]+1;
                return kIOReturnSuccess;
            }
        }
        native_adreno_pool_stats[2]=native_adreno_pool_stats[2]+1;
    }
    IOOptionBits options=static_cast<IOOptionBits>(kIODirectionInOut)|static_cast<IOOptionBits>(kIOMemoryMapperNone);
    if(contiguous)options|=kIOMemoryPhysicallyContiguous;
    /* Keep the qualified wired kernel mapping. A pageable descriptor keeps
     * a UPL over a VM object while the physical-view client mapping changes
     * its cache attributes, mixing two ownership models on Darwin24. The
     * reduced optional BO pool leaves room for these wired allocations.
     * No physical-address restriction is needed to obtain ordinary pages:
     * validate each resulting span below before any GPU mapping is exposed.
     * This avoids the restricted allocator/compaction path for normal BOs.
     * Firmware/ring/table objects still request contiguous masked memory. */
    /* Upstream01ed692: large optional contiguous searches stall UI callers
     * under memory pressure. Limit the optimization to <=128KiB, with a
     * five-second backoff after failure. Pong retains wired native4K/PA
     * ownership; mandatory firmware/ring allocations are unaffected. */
    static size_t failedBytes=0;static uint64_t failedAt=0;
    uint64_t backoff=0;nanoseconds_to_absolutetime(UINT64_C(5000000000),&backoff);
    const uint64_t now=mach_absolute_time();
    if(failedBytes && now-failedAt>=backoff)failedBytes=0;
    if(pooled && !contiguous && bytes<=128u*1024u) {
        unsigned kib=64;PE_parse_boot_argn("pong_gpu_bo_contig_kib",&kib,sizeof kib);
        if(kib && kib<=128 && bytes>=size_t(kib)*1024 && !(failedBytes && bytes>=failedBytes)) {
            descriptor_=IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
                options|kIOMemoryPhysicallyContiguous,bytes,(limit-1)&~UINT64_C(4095));
            native_adreno_bo_contig[descriptor_?0:1]++;
            if(!descriptor_){if(!failedBytes || bytes<failedBytes)failedBytes=bytes;failedAt=now;}
        }
    }
    if(!descriptor_)descriptor_=contiguous?
        IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,options,bytes,(limit-1)&~UINT64_C(4095)):
        IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task,options,bytes,4096);
    if(!descriptor_)return kIOReturnNoMemory;
    status=descriptor_->prepare();
    if(status!=kIOReturnSuccess) {descriptor_->release();descriptor_=nullptr;return status;}
    prepared_=true;bytes_=bytes;mapping_=descriptor_->getBytesNoCopy();
    status=kIOReturnUnsupported;
    if(!mapping_ || ((uintptr_t)mapping_&4095) || bytes>UINTPTR_MAX-(uintptr_t)mapping_ ||
        descriptor_->getLength()!=bytes)goto failed;
    span_capacity_=bytes/4096;
    spans_=IONewData(NativeAdrenoPhysicalSpan,span_capacity_);
    if(!spans_) {status=kIOReturnNoMemory;goto failed;}
    for(size_t offset=0;offset<bytes;offset+=4096) {
        IOByteCount length=0;
        addr64_t physical=descriptor_->getPhysicalSegment(offset,&length,kIOMemoryMapperNone);
        if(!physical || (physical&4095) || length<4096 || physical>=limit || 4096>limit-physical)
            {status=kIOReturnUnsupported;goto failed;}
        if(contiguous && offset && physical!=spans_[0].physical+offset)
            {status=kIOReturnUnsupported;goto failed;}
        if(span_count_ && physical==spans_[span_count_-1].physical+spans_[span_count_-1].bytes)
            spans_[span_count_-1].bytes+=4096;
        else spans_[span_count_++]={offset,physical,4096};
    }
    bzero(mapping_,bytes);
    status=NativeAdrenoMaintainOwnedPages(mapping_,bytes,NativeAdrenoClean);
    if(status!=kIOReturnSuccess)goto failed;
    native_adreno_pool_stats[5]=native_adreno_pool_stats[5]+1;native_adreno_pool_stats[6]=native_adreno_pool_stats[6]+(uint32_t)span_count_;
    noteOwned(spans_,span_count_,true);owned_=true;
    return kIOReturnSuccess;
failed:
    /* release() deliberately retains descriptor/metadata if complete fails.
     * Preserve the allocation's original failure instead of reporting cleanup
     * as success. No object has been exposed to a GPU or userspace yet. */
    (void)release();return status;
}
static void appendSystemSpan(NativeAdrenoPhysicalSpan *spans,size_t &count,
    size_t offset,uint64_t physical,size_t bytes)
{
    if(count && physical==spans[count-1].physical+spans[count-1].bytes)
        spans[count-1].bytes+=bytes;
    else spans[count++]={offset,physical,bytes};
}

static bool systemSpansFit(const NativeAdrenoPhysicalSpan *spans,size_t count,uint64_t limit)
{
    for(size_t i=0;i<count;++i)
        if(spans[i].physical>=limit || spans[i].bytes>limit-spans[i].physical)return false;
    return true;
}

IOReturn NativeAdrenoMemory::prepareSystem(size_t bytes,NativeAdrenoSystemPages *out)
{
    if(!out || out->descriptor || !bytes || (bytes&4095) || bytes>256u*1024u*1024u)return kIOReturnBadArgument;
    const bool measure=bytes>=32u*1024u*1024u;
    const uint64_t began=measure?mach_absolute_time():0;
    const IOOptionBits options=IOOptionBits(kIODirectionInOut)|IOOptionBits(kIOMemoryMapperNone)|
        IOOptionBits(kIOMemoryPageable)|IOOptionBits(kIOMapWriteCombineCache);
    IOBufferMemoryDescriptor *descriptor=IOBufferMemoryDescriptor::inTaskWithOptions(nullptr,options,bytes,4096);
    if(!descriptor)return kIOReturnNoMemory;
    IOReturn status=descriptor->prepare();
    uint64_t checkpoint=measure?mach_absolute_time():0;
    const uint64_t wireTicks=checkpoint-began;
    uint64_t walkTicks=0,cacheTicks=0,wcTicks=0;
    size_t physicalSpans=0;
    if(status){descriptor->release();return status;}
    if(descriptor->getLength()!=bytes){(void)descriptor->complete();descriptor->release();return kIOReturnUnsupported;}
    /* Darwin24's VM zeroes newly wired pages through the cached physical
     * aperture. On Pong that aperture is a block mapping, not a PV-tracked
     * alias (__ARM_PTE_PHYSMAP__ is off). A WC memory entry does not make a
     * dirty zero-fill line disappear from that alias. Publish and invalidate
     * it BEFORE a client can write commands through its WC view; cleaning it
     * during the first submit can otherwise overwrite those new commands.
     * This one-time handoff stays outside the shared GPU gate. No kernel VA
     * or second physical pager is created for the BO. */
    // Scattered 64 MiB BOs may have 16,384 separate physical spans.
    // Queue a bounded number of spans so the cache helper can share each
    // barrier across pages even when the physical addresses are disjoint.
    // The exact physical spans of a prepared descriptor are stable until
    // complete(). Save the large walk outside the shared device gate.
    const size_t capacity=bytes/4096;
    auto *saved=measure?IONewData(NativeAdrenoPhysicalSpan,capacity):nullptr;
    size_t savedCount=0;
    NativeAdrenoPhysicalSpan spans[64];
    for(size_t offset=0;offset<bytes && !status;) {
        const size_t batchStart=offset;
        size_t count=0;
        while(offset<bytes && count<sizeof spans/sizeof spans[0]) {
            IOByteCount length=0;
            const addr64_t physical=descriptor->getPhysicalSegment(offset,&length,kIOMemoryMapperNone);
            if(!physical || (physical&4095) || length<4096) {status=kIOReturnUnsupported;break;}
            if(length>bytes-offset)length=bytes-offset;
            length&=~IOByteCount(4095);
            spans[count++]={offset-batchStart,physical,size_t(length)};
            if(saved)appendSystemSpan(saved,savedCount,offset,physical,size_t(length));
            offset+=size_t(length);
        }
        if(status)break;

        if(measure) {
            const uint64_t now=mach_absolute_time();
            walkTicks+=now-checkpoint;checkpoint=now;
            physicalSpans+=count;
        }
        status=NativeAdrenoMaintainPhysicalSpans(spans,count,0,offset-batchStart,NativeAdrenoCleanInvalidate);
        if(status)break;
        if(measure) {
            const uint64_t now=mach_absolute_time();
            cacheTicks+=now-checkpoint;checkpoint=now;
        }
        uint32_t reset=0;
        status=NativeAdrenoReassertWriteCombined(spans,count,&reset);
        if(measure) {
            const uint64_t now=mach_absolute_time();
            wcTicks+=now-checkpoint;checkpoint=now;
        }
    }
    if(measure && !status) {
        // One bounded receipt per large allocation, outside the device gate.
        static uint32_t receipts;
        if(__atomic_load_n(&receipts,__ATOMIC_RELAXED)<48 &&
           __atomic_fetch_add(&receipts,1,__ATOMIC_RELAXED)<48) {
            uint64_t wireNs,walkNs,cacheNs,wcNs;
            absolutetime_to_nanoseconds(wireTicks,&wireNs);
            absolutetime_to_nanoseconds(walkTicks,&walkNs);
            absolutetime_to_nanoseconds(cacheTicks,&cacheNs);
            absolutetime_to_nanoseconds(wcTicks,&wcNs);
            IOLog("PONG GPU BO PREP: bytes=%zu spans=%zu wire_us=%llu walk_us=%llu cache_us=%llu wc_us=%llu\n",
                bytes,physicalSpans,wireNs/1000,walkNs/1000,cacheNs/1000,wcNs/1000);
        }
    }
    if(status) {
        if(saved)IODeleteData(saved,NativeAdrenoPhysicalSpan,capacity);
        (void)descriptor->complete();descriptor->release();
        return status;
    }
    *out={descriptor,bytes,saved,savedCount,saved?capacity:0};
    return kIOReturnSuccess;
}
void NativeAdrenoMemory::discardSystem(NativeAdrenoSystemPages *pages)
{
    if(!pages)return;
    // Unadopted pages and detached retired pages are no longer visible to clients.
    if(pages->descriptor) {
        (void)pages->descriptor->complete();
        pages->descriptor->release();
    }
    if(pages->spans)IODeleteData(pages->spans,NativeAdrenoPhysicalSpan,pages->span_capacity);
    *pages={};
}
IOReturn NativeAdrenoMemory::allocateSystem(size_t bytes,unsigned bits,size_t budget,NativeAdrenoSystemPages *prepared)
{
    if(backed() || spans_ || quarantined_)return kIOReturnBusy;
    if(!bytes || (bytes&4095) || bytes>budget || !outputWidth(bits))return kIOReturnBadArgument;
    NativeAdrenoCpuState cpu{};IOReturn status=NativeAdrenoQueryCpu4K(&cpu);if(status)return status;
    NativeAdrenoSystemPages local{};
    if(!prepared || !prepared->descriptor || prepared->bytes!=bytes) {
        status=prepareSystem(bytes,&local);if(status)return status;prepared=&local;
    }
    descriptor_=prepared->descriptor;
    spans_=prepared->spans;
    span_count_=prepared->span_count;
    span_capacity_=prepared->span_capacity;
    *prepared={};
    bytes_=bytes;prepared_=true;system_=true;write_combined_=true;
    const uint64_t limit=UINT64_C(1)<<bits;
    if(spans_) {
        if(!systemSpansFit(spans_,span_count_,limit)) {
            (void)release();return kIOReturnUnsupported;
        }
        static unsigned reported;
        if(__atomic_load_n(&reported,__ATOMIC_RELAXED)<16 &&
           __atomic_fetch_add(&reported,1,__ATOMIC_RELAXED)<16) {
            IOLog("PONG GPU BO SPANS: reused prepared walk bytes=%zu spans=%zu\n",bytes,span_count_);
        }
    } else {
        span_capacity_=bytes/4096;
        spans_=IONewData(NativeAdrenoPhysicalSpan,span_capacity_);
        if(!spans_){(void)release();return kIOReturnNoMemory;}
        for(size_t offset=0;offset<bytes;offset+=4096) {
            IOByteCount length=0;
            const addr64_t physical=descriptor_->getPhysicalSegment(offset,&length,kIOMemoryMapperNone);
            if(!physical || (physical&4095) || length<4096 || physical>=limit || 4096>limit-physical) {
                (void)release();return kIOReturnUnsupported;
            }
            appendSystemSpan(spans_,span_count_,offset,physical,4096);
        }
    }
    // prepareSystem completed the Darwin24 physical-aperture handoff before
    // these pages could be exposed. Keep the same WC VM object for views.
    noteOwned(spans_,span_count_,true);owned_=true;
    native_adreno_pool_stats[5]++;native_adreno_pool_stats[6]+=(uint32_t)span_count_;
    return kIOReturnSuccess;
}
/* An import holds the pages through a memory entry of its own (kIOMemoryPersistent), which references the VM object: the owner may unmap
 * its range while the GPU still uses it (Core Animation frees an image's Shmem once its no-copy buffer is released, before the runtime frees
 * the import), and the object then lives on until the import completes instead of being torn down under a wired UPL (int-233..235: the render
 * thread stuck in vm_deallocate). A range no memory entry can be made of is imported as before; [0] counts the persistent imports, [1] those. */
extern "C" {volatile uint32_t native_adreno_import_persistent[2];}
static IOMemoryDescriptor *importRange(uint64_t address,size_t bytes,IODirection direction,void *task,
    NativeAdrenoTaskPages &pages)
{
    const IOOptionBits options=static_cast<IOOptionBits>(direction)|static_cast<IOOptionBits>(kIOMemoryMapperNone);
    IOMemoryDescriptor *range=IOMemoryDescriptor::withAddressRange(address,bytes,options|kIOMemoryPersistent,static_cast<task_t>(task));
    if(range) {++pages.persistent;return range;}
    range=IOMemoryDescriptor::withAddressRange(address,bytes,options,static_cast<task_t>(task));
    if(range)++pages.fallback;
    return range;
}
#if PONG_GUEST_22H374
static bool canMaintainImportInParallel(const NativeAdrenoTaskPages &pages,size_t aliasPages)
{
    if(pages.bytes<16u*1024u*1024u || aliasPages!=pages.bytes/4096 ||
        !pages.span_count || pages.span_count>UINT32_MAX)return false;
    // Sort only indices with the bounded-stack heap walk used by the GPU VM.
    // Check overlap before any compaction: duplicate PA must stay serial.
    auto *order=IONewData(uint32_t,pages.span_count);
    if(!order)return false;
    for(uint32_t i=0;i<pages.span_count;++i)order[i]=i;
    NativeAdrenoPhysicalIndex<NativeAdrenoPhysicalSpan>::sort(pages.spans,order,pages.span_count);
    bool disjoint=true;
    for(size_t i=1;i<pages.span_count;++i) {
        const auto &previous=pages.spans[order[i-1]];
        if(pages.spans[order[i]].physical<previous.physical+previous.bytes) {
            disjoint=false;
            break;
        }
    }
    IODeleteData(order,uint32_t,pages.span_count);
    return disjoint;
}
#endif

IOReturn NativeAdrenoMemory::prepareTaskPages(void *task,uint64_t address,size_t bytes,unsigned bits,size_t budget,
    NativeAdrenoTaskPages *out)
{
    auto &pages=*out;
    pages.stage=11;
    if(!task || !address || (address&4095) || !bytes || (bytes&4095) || address>UINT64_MAX-bytes || !outputWidth(bits) ||
        !budget || budget>256u*1024u*1024u)return kIOReturnBadArgument;
    pages.stage=12;
    if(bytes>budget)return kIOReturnNoResources;
    NativeAdrenoCpuState cpu{};
    pages.stage=13;
    IOReturn status=NativeAdrenoQueryCpu4K(&cpu);if(status!=kIOReturnSuccess)return status;
    const uint64_t limit=UINT64_C(1)<<bits;
    IOMemoryDescriptor *descriptor=importRange(address,bytes,kIODirectionInOut,task,pages);
    pages.stage=14;
    if(!descriptor)return kIOReturnNoMemory;
    pages.stage=15;
    status=descriptor->prepare();
    if(status==kNativeAdrenoProtectionFailure) {
        // The owner can pin a read-only mapping for GPU reads only.
        descriptor->release();
        descriptor=importRange(address,bytes,kIODirectionOut,task,pages);
        if(!descriptor)return kIOReturnNoMemory;
        status=descriptor->prepare();
        if(!status)pages.read_only=true;
    }
    if(status) {descriptor->release();return status;}
    pages.descriptor=descriptor;
    pages.bytes=bytes;
    pages.stage=16;
    if(descriptor->getLength()!=bytes)return kIOReturnUnsupported;
    pages.stage=17;
    pages.span_capacity=bytes/4096;
    pages.spans=IONewData(NativeAdrenoPhysicalSpan,pages.span_capacity);
    if(!pages.spans)return kIOReturnNoMemory;
    for(size_t offset=0;offset<bytes;offset+=4096) {
        IOByteCount length=0;
        addr64_t physical=descriptor->getPhysicalSegment(offset,&length,kIOMemoryMapperNone);
        if(!physical || (physical&4095) || length<4096 || physical>=limit || 4096>limit-physical) {
            pages.stage=18;pages.fail_pa=physical;pages.fail_offset=offset;
            return kIOReturnUnsupported;
        }
        if(pages.span_count && physical==pages.spans[pages.span_count-1].physical+pages.spans[pages.span_count-1].bytes)
            pages.spans[pages.span_count-1].bytes+=4096;
        else pages.spans[pages.span_count++]={offset,physical,4096};
    }
#if PONG_GUEST_22H374
    const size_t aliasPages=NativeAdrenoPrepareImportAliases(pages.spans,pages.span_count);
    pages.parallel_cache=canMaintainImportInParallel(pages,aliasPages);
    if(bytes==5888u*3200u || bytes==5760u*3200u) {
        static unsigned reports;
        if(__atomic_load_n(&reports,__ATOMIC_RELAXED)<24 &&
           __atomic_fetch_add(&reports,1,__ATOMIC_RELAXED)<24) {
            IOLog("PONG GPU IMPORT APERTURE: bytes=%zu spans=%zu alias_pages=%zu total_pages=%zu parallel=%u\n",
                bytes,pages.span_count,aliasPages,bytes/4096,pages.parallel_cache?1u:0u);
        }
    }
#endif
    return kIOReturnSuccess;
}
void NativeAdrenoMemory::discardTaskPages(NativeAdrenoTaskPages *pages)
{
    if(pages->descriptor) {
        // withAddressRange creates a local IOGeneralMemoryDescriptor;
        // its prepared UPL completes unless it is a remote descriptor.
        // Retain the reference on an unexpected completion error.
        const IOReturn status=pages->descriptor->complete();
        if(!status)pages->descriptor->release();
        else IOLog("Pong Adreno: unpublished import unwire failed 0x%x\n",static_cast<unsigned>(status));
    }
    if(pages->spans)IODeleteData(pages->spans,NativeAdrenoPhysicalSpan,pages->span_capacity);
    *pages={};
}
void NativeAdrenoMemory::recordTaskPreparation(const NativeAdrenoTaskPages &pages)
{
    native_adreno_import_stage=pages.stage;
    native_adreno_import_fail_pa=pages.fail_pa;
    native_adreno_import_fail_offset=pages.fail_offset;
    native_adreno_import_persistent[0]+=pages.persistent;
    native_adreno_import_persistent[1]+=pages.fallback;
}
IOReturn NativeAdrenoMemory::adoptTaskPages(NativeAdrenoTaskPages *pages)
{
    if(descriptor_ || imported_ || spans_ || quarantined_)return kIOReturnBusy;
    if(!pages->descriptor || !pages->spans || !pages->span_count)return kIOReturnBadArgument;
    native_adreno_import_stage=19;
    // Preserve serialization of cache-ownership changes with GPU work.
    // A failed maintenance leaves the pages detached for unlocked cleanup.
    const bool uncached=NativeAdrenoPagesUncached(pages->spans,pages->span_count);
    if(uncached) {
        const IOReturn status=NativeAdrenoMaintainPhysicalSpans(pages->spans,pages->span_count,
            0,pages->bytes,NativeAdrenoCleanInvalidate);
        if(status)return status;
    }

    imported_=pages->descriptor;
    prepared_=true;
    read_only_=pages->read_only;
    owner_uncached_=uncached;
    parallel_cache_=pages->parallel_cache;
    bytes_=pages->bytes;
    spans_=pages->spans;
    span_count_=pages->span_count;
    span_capacity_=pages->span_capacity;
    *pages={};
    native_adreno_pool_stats[7]=native_adreno_pool_stats[7]+(uint32_t)span_count_;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoMemory::importTask(void *task,uint64_t address,size_t bytes,unsigned bits,size_t budget)
{
    if(descriptor_ || imported_ || spans_ || quarantined_)return kIOReturnBusy;
    NativeAdrenoTaskPages pages{};
    IOReturn status=prepareTaskPages(task,address,bytes,bits,budget,&pages);
    recordTaskPreparation(pages);
    if(!status)status=adoptTaskPages(&pages);
    discardTaskPages(&pages);
    if(status)(void)release();
    return status;
}
IOReturn NativeAdrenoMemory::makeWriteCombined()
{
    if(system_ && prepared_ && write_combined_ && !quarantined_)return kIOReturnSuccess;
    if(!(descriptor_ || pooled_) || imported_ || !prepared_ || quarantined_ || device_references_ || write_combined_ || !mapping_ || !span_count_)
        return kIOReturnNotReady;
    if(pooled_ && pool.write_combined) {write_combined_=true;return kIOReturnSuccess;}
    IOReturn status=NativeAdrenoMaintainOwnedPages(mapping_,bytes_,NativeAdrenoCleanInvalidate);
    if(status)return status;
    status=NativeAdrenoSetPagesCacheMode(spans_,span_count_,true);
    if(status)return status;
    write_combined_=true;return kIOReturnSuccess;
}
IOReturn NativeAdrenoMemory::release()
{
    native_adreno_trace_stage=30;
    if(device_references_ || quarantined_)return kIOReturnBusy;
    if(write_combined_ && pooled_ && pool.write_combined)write_combined_=false;   /* the pool's pages keep the attribute */
    if(write_combined_ && !system_) {
        /* The pages go back to the VM cacheable, as they came. */
        const IOReturn status=NativeAdrenoSetPagesCacheMode(spans_,span_count_,false);
        if(status) {quarantined_=true;return status;}
        write_combined_=false;
    }
    if(imported_) {
        if(prepared_) {
            IOReturn status=imported_->complete();
            if(status!=kIOReturnSuccess) {quarantined_=true;return status;}
            prepared_=false;
        }
        imported_->release();imported_=nullptr;
    }
    if(descriptor_) {
        if(prepared_) {
            native_adreno_trace_stage=31;
            IOReturn status=descriptor_->complete();
            native_adreno_trace_stage=32;
            if(status!=kIOReturnSuccess) {quarantined_=true;return status;}
            prepared_=false;
        }
        native_adreno_trace_stage=33;
        descriptor_->release();descriptor_=nullptr;
        native_adreno_trace_stage=34;
    }
    if(owned_) {noteOwned(spans_,span_count_,false);owned_=false;}
    if(pooled_) {poolGive(pool_offset_,bytes_);pooled_=false;prepared_=false;pool_offset_=0;}
    if(spans_)IODeleteData(spans_,NativeAdrenoPhysicalSpan,span_capacity_);
    mapping_=nullptr;spans_=nullptr;bytes_=span_count_=span_capacity_=0;read_only_=false;system_=false;write_combined_=false;owner_uncached_=false;explicit_sync_=false;explicit_acquire_=false;
    parallel_cache_=false;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoMemory::releaseSystemInto(NativeAdrenoSystemPages *out)
{
    if(!out || out->descriptor || !system_ || !descriptor_ || !prepared_ ||
       imported_ || device_references_ || quarantined_)return release();
    out->descriptor=descriptor_;out->bytes=bytes_;
    descriptor_=nullptr;prepared_=false;
    return release(); // Remove spans/ownership under the gate; VM after it.
}
IOReturn NativeAdrenoMemory::holdForDevice()
{
    if(!backed() || !prepared_ || !span_count_ || quarantined_)return kIOReturnNotReady;
    if(device_references_==UINT32_MAX)return kIOReturnNoResources;
    ++device_references_;return kIOReturnSuccess;
}
IOReturn NativeAdrenoMemory::retireAfterQuiescence()
{
    if(!device_references_)return kIOReturnBadArgument;
    if(quarantined_)return kIOReturnNotReady;
    --device_references_;return kIOReturnSuccess;
}
IOReturn NativeAdrenoMemory::publishForDevice(size_t offset,size_t bytes,bool writes,bool include_write_combined,
    NativeAdrenoCacheWork *work)
{
    if(!backed() || !prepared_ || quarantined_)return kIOReturnNotReady;
    if(!bytes || ((offset|bytes)&4095) || offset>bytes_ || bytes>bytes_-offset)return kIOReturnBadArgument;
    if(imported_ && owner_uncached_)return kIOReturnSuccess;
    /* If an alias still caches a WC BO, reach its lines via the physical pages. */
    if(!imported_ && write_combined_ && !include_write_combined)return kIOReturnSuccess;
    const auto op=writes?NativeAdrenoCleanInvalidate:NativeAdrenoClean;
    const IOReturn result=imported_ || write_combined_?maintainImported(offset,bytes,op):
        NativeAdrenoMaintainOwnedPages(static_cast<unsigned char *>(mapping_)+offset,bytes,op);
    if(!result && work){work->bytes+=bytes;++work->mappings;}
    return result;
}
IOReturn NativeAdrenoMemory::acquireDeviceWrites(size_t offset,size_t bytes,NativeAdrenoCacheWork *work)
{
    if(!backed() || !prepared_ || quarantined_)return kIOReturnNotReady;
    if(!bytes || ((offset|bytes)&4095) || offset>bytes_ || bytes>bytes_-offset)return kIOReturnBadArgument;
    /* Imported pages may hold the owner's unrelated dirty lines next to the GPU's writes: clean as well as invalidate. */
    if(imported_?owner_uncached_:write_combined_)return kIOReturnSuccess;
    const IOReturn result=imported_?maintainImported(offset,bytes,NativeAdrenoCleanInvalidate):
        NativeAdrenoMaintainOwnedPages(static_cast<unsigned char *>(mapping_)+offset,bytes,NativeAdrenoInvalidate);
    if(!result && work){work->bytes+=bytes;++work->mappings;}
    return result;
}
/* Imported pages have no kernel mapping here (and write-combined ones only a non-cacheable one); maintain each span's part of
 * [offset, offset+bytes) by physical address. */
IOReturn NativeAdrenoMemory::maintainImported(size_t offset,size_t bytes,NativeAdrenoCacheOperation op)
{
    return NativeAdrenoMaintainPhysicalSpans(spans_,span_count_,offset,bytes,op,parallel_cache_);
}
