/* SPDX-License-Identifier: MIT */
/* IOKit surface for an already-running GPU context. The board code (kits/sheng) brings the GPU up, then hands this file
 * the live VM, submit object, ring, clock and optional timer; one IOService is published and each IOServiceOpen creates a
 * connection that mounts its own NativeAdrenoUserClient core. Up to NativeAdrenoMaxClients connections (one per Metal process) share
 * the VM, the submit object and the ring: each core has its own handles, its own queue range on the ring and its own window of the
 * one GPU address space (the render server the large one), and closing a connection waits for its own jobs only, then unmaps and
 * frees everything it created. Not yet done: per-process GPU page tables (the windows are enforced by the driver, not by the SMMU)
 * and a reset path. This is not the Metal or Turnip ABI; it carries NativeAdrenoABI.h unchanged.
 * BO CPU mappings are handed out as uncached physical-range mappings to match the GPU's non-cacheable PTEs.
 * The connection class is named AGXDeviceUserClient on purpose: the original sandbox profiles of the processes that use Metal
 * allow IOServiceOpen for that user-client class name, and nothing else in this system can create one (the Apple AGX kext is
 * never loaded without an Apple GPU), so the name only decides whether the Adreno Metal plugin may open its own driver. */
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOService.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IOUserClient.h>
#include <string.h>
#include <kern/thread.h>
#include <kern/thread_call.h>
#include <kern/clock.h>
#include <mach/vm_param.h>
#include <pexpert/pexpert.h>
#include <sys/proc_info.h>
#include "NativeAdrenoUserClient.h"
#include "../PongAdrenoScanout.h"
#include "NativeGpuTrace.inc"
extern "C" unsigned kauth_cred_getuid(void *);
extern "C" int proc_selfpid(void);
extern "C" unsigned int vm_page_free_count;
extern "C" unsigned int vm_page_free_target;
extern "C" const NativeAdrenoStatus *pong_adreno_native_status(void);
extern "C" bool pong_adreno_native_healthy(void);

extern "C" void proc_selfname(char *, int);
/* The plugin's telemetry table (rows of text the Metal bridge keeps), sent as markers: 0xC000|row starts a row, 0x8000|(a<<7)|b appends two characters. */
extern "C" {char native_adreno_metal_rows[48][96];volatile uint32_t native_adreno_metal_rows_seq;}

extern "C" {
/* What the processes that reached this driver did, for the boot diagnostic page. Plain counters and short strings. */
struct NativeAdrenoMetalDiag {
    uint32_t opens, busy, bad, closes, reports, property_reads, foreign_opens, foreign_type, name_reads, calls, last_selector, last_status, ext_entries, ext_selector, ext_status, ext_in, ext_out, tunnel_calls, probes, sel_bits[8], mb_maps, mb_calls, marks_total, marks[16], chk_macf, chk_res, chk_type, type_pass_lo, type_pass_hi, attach_status, inflight, inflight_since_calls, forced_unmaps;
    char opener[3][17];      /* process names of the latest connections, newest first */
    char reader[3][17];      /* process names that read a registry property of the accelerator, newest first */
    char name_reader[17];    /* latest process that read MetalPluginName specifically */
    char starter[3][17];     /* processes that sent plugin marker 1 (the plugin's initWithAcceleratorPort: started), newest first */
    char report[64];         /* the plugin's own result line (SetMetadata text), latest */
    char report2[64];        /* the plugin's Turnip-phase line (text starting "T " or "W "), latest */
    uint32_t grants;         /* opens granted to the render server by this driver in place of the sandbox verdict (the view in GpuPowerProbe.cpp ends before this) */
} native_adreno_metal_diag;
/* BO CPU views (clientMemoryForType for a BO token): [0] calls, [1] new descriptors, [2] served from the view cache, [3] refused (not a BO / out of
 * resources), [4] last refusal's IOReturn, [5] threads inside now. int-115 showed a backboardd thread in IOConnectMapMemory64 ~650 times a second. */
extern "C" {volatile uint32_t native_adreno_view_diag[6],native_adreno_view_kept;}   /* kept: BoCpuUnmap calls that left the view for another lease */
}

/* The open type the Adreno plugin passes to IOServiceOpen ('AG'); it lets the driver tell the plugin from the original IOGPU library
 * (type 1 plus an API value) and from any other client of the accelerator. */
enum : UInt32 {NativeAdrenoPluginOpenType=0x4147,NativeAdrenoTunnelMagic=0x4e544e4c /* 'NTNL' */};
/* The display hand-off (upstream GopFramebuffer, PongAdrenoScanout here) reaches the plugin through this already admitted connection: the
 * original sandbox profiles admit this class but not the presenter's own service. Methods 0x7000 + the presenter's NGP_METHOD numbers (mode,
 * wait for a frame, complete it) are forwarded unchanged, without the GPU lock (a wait sleeps); memory type NativeAdrenoScanoutType is the
 * writable scanout. Nothing here touches the GPU. */
enum : UInt32 {NativeAdrenoHandoffBase=0x7000,NativeAdrenoHandoffLast=0x7004,NativeAdrenoScanoutType=0x5343,
               NativeAdrenoHandoffOpenType=0x4148};
/* Open type 0x4148 is a connection that serves only the hand-off: it does not mount a GPU core, so it does not need (or take) a
 * GPU connection slot (the Vulkan driver holds one for as long as the Metal device lives). Same class, so the same sandbox admission. */
extern "C" IOReturn native_gop_handoff_call(uint32_t op,task_t task,const void *in,size_t inSize,void *out,size_t outSize);
/* Export (int-147): method NativeAdrenoExportMethod hands one in-order piece of a blob to the board's UFS log (kits/sheng/BootLog.c), which writes
 * the newest complete blob to its export area. The render server sends its pipeline caches this way, so a later image can be built with them.
 * Input: {generation, total, offset, bytes} then the bytes. No GPU state is touched; any connection of the render server may send it. */
enum : UInt32 {NativeAdrenoExportMethod=0x7100,NativeAdrenoExportMaxPiece=3968};
/* Pong has no Sheng persistent shader-export area. Keep the optional
 * extension's actual absent-backend result without a weak unresolved symbol
 * in Darwin22's static kernel link. This does not affect BO/submit/Metal. */
static constexpr int (*boot_log_export_put)(uint32_t,uint32_t,uint32_t,const void *,uint32_t)=nullptr;
extern "C" {volatile uint32_t native_adreno_export_stats[4];}   /* [0] pieces taken, [1] refused, [2] last generation, [3] its total bytes */
extern "C" IOMemoryDescriptor *native_gop_handoff_memory(void);

/* Mailbox: a last-resort command channel for a sandbox that admits no external-method selector at all. It is one shared page (memory
 * type NativeAdrenoMailboxType through IOConnectMapMemory64, which the selector filter does not cover) and a kernel poller that runs
 * whatever the live connection writes into it. Layout (little endian dwords): +0 magic, +4 request seq, +8 response seq, +12 method,
 * +16 status, +20 input bytes, +24 output bytes, input at +0x40 (max 0x700), output at +0x800 (max 0x700). */
enum : UInt32 {NativeAdrenoMailboxType=0xC0DE,NativeAdrenoMailboxMagic=0x4d41494c /* 'MAIL' */};
namespace {
IOBufferMemoryDescriptor *gMailbox=nullptr;
volatile uint32_t *gMailboxWords=nullptr;
volatile bool gPollerStarted=false;
}
class AGXDeviceUserClient;
/* The render server's connection serves the mailbox; it is retained while set (under g.lock), so the poller never runs on a freed object. */
static AGXDeviceUserClient *gMailboxOwner=nullptr;
void nativeAdrenoMailboxPoller(void *,wait_result_t);
static IOReturn runCore(AGXDeviceUserClient *,uint32_t method,const void *input,size_t input_size,void *output,size_t output_size);

/* The two sandbox gates IOServiceOpen applies after newUserClient (XNU IOUserClient.cpp is_io_service_open_extended): the MACF open check on
 * the client object and the sandbox's filter resolver. They are run here for the caller too, only to record which one refuses it. */
extern "C" int mac_iokit_check_open(void *cred,void *client,unsigned type);
extern "C" void *kauth_cred_get(void);
extern const struct io_filter_callbacks *gIOUCFilterCallbacks;

extern "C" volatile uint32_t native_adreno_trace_stage;
extern "C" {
/* One entry per IOServiceOpen of the accelerator (not markers): who, which type, what the two sandbox gates said. */
struct NativeAdrenoOpenRecord {char name[20]; uint32_t type, macf, res, attach, foreign;};
struct NativeAdrenoOpenRecord native_adreno_open_log[8];
volatile uint32_t native_adreno_open_log_count;
/* One row per process name that opened the plugin type (first eight names): opens granted, refused (sandbox, cap or no free slot),
 * closes, the slot of its live connection (0xff none), the highest GPU address its mappings reached (MiB) and its last refusal. */
struct NativeAdrenoClientRecord {char name[20]; uint32_t opens, refused, closes, slot, peak_mib, last_refusal;};
struct NativeAdrenoClientRecord native_adreno_client_log[NativeAdrenoMaxClients];
/* The last text line (the plugin's failure reason) each process other than the render server streamed: process name and a 64-byte line. */
struct NativeAdrenoClientText {char name[20]; char text[64];};
struct NativeAdrenoClientText native_adreno_client_text[NativeAdrenoMaxClients];
/* Why the latest mount of a connection failed: [0] 1 no free slot, 2 the core's open, 3 the window scope; [1] its IOReturn; [2] the slot; [3] failures so far. */
volatile uint32_t native_adreno_mount_fail[4];
/* Processes that read MetalPluginName (the Metal discovery), first twelve names, with counts. */
struct NativeAdrenoAskerRecord {char name[20]; uint32_t reads;};
struct NativeAdrenoAskerRecord native_adreno_metal_askers[12];
}
/* ABI1 still shares one32-bit space. Keep both UI reservations unchanged;
 * lease the remaining VA by live process demand instead of wasting fourteen
 * fixed128MiB windows. These are VA leases, never wired RAM reservations. */
struct NativeAdrenoClientWindow {uint64_t start=0,bytes=0;};

namespace {
void shiftName(char names[3][17]) {
    memcpy(names[2], names[1], 17);
    memcpy(names[1], names[0], 17);
    proc_selfname(names[0], 17);
}
struct Published {
    bool published=false;
    NativeAdrenoVm *vm=nullptr;
    NativeAdrenoSubmit *submit=nullptr;
    NativeAdrenoRing *ring=nullptr;
    NativeAdrenoRingClock clock{};
    const NativeAdrenoTimer *timer=nullptr;
    const NativeAdrenoAbiDeviceInfo *info=nullptr;
    IOLock *lock=nullptr;
    // Serialize detached task UPLs so concurrent import requests cannot
    // multiply the unpublished wiring budget. Close does not take this lock.
    IOLock *import_lock=nullptr;
    thread_call_t unwire_call=nullptr;
    bool unwire_busy=false;
    thread_call_t prewarm_call=nullptr;
    NativeAdrenoSystemPages *prepared_spares[4]={};
    bool prewarm_pending=false;
    IOService *service=nullptr;
    bool slots[NativeAdrenoMaxClients]{};
    NativeAdrenoClientWindow windows[NativeAdrenoMaxClients]{};
    bool kernel_client=false;
    uint32_t input_bits=0,output_bits=0;
    uint32_t opened=0,closed=0;
} g;
static NativeAdrenoClientWindow clientWindow(unsigned slot,const char *name)
{
    if(slot==0)return {0x20000000ull,0x50000000ull};
    if(slot==1)return {0x70000000ull,0x20000000ull};
    bool poster=false;
    for(unsigned at=0;at<20 && name[at];++at)
        if(!strncmp(name+at,"Poster",6) || !strncmp(name+at,"Wallpaper",9)) {poster=true;break;}
    // MercuryPoster hit real device-VA OOM with128MiB while importing its
    // FP16 targets. Prefer384MiB for posters,256MiB for input/preferences,
    //128MiB otherwise. Search actual holes; never relocate a live mapping.
    uint64_t preferred=poster?0x18000000ull:
        (!strcmp(name,"InputUI") || !strcmp(name,"Preferences"))?0x10000000ull:0x08000000ull;
    for(uint64_t amount=preferred;amount>=0x04000000ull;amount-=0x04000000ull) {
        uint64_t start=0x90000000ull;
        for(unsigned pass=0;pass<=NativeAdrenoMaxClients && start+amount<=0x100000000ull;++pass) {
            uint64_t next=start;
            for(unsigned i=2;i<NativeAdrenoMaxClients;++i)if(g.slots[i]) {
                const auto &window=g.windows[i];
                if(start<window.start+window.bytes && window.start<start+amount && next<window.start+window.bytes)
                    next=window.start+window.bytes;
            }
            if(next==start)return {start,amount};
            start=next;
        }
    }
    return {};
}
}

class NativeAdrenoGpuPublished : public IOService {
    OSDeclareDefaultStructors(NativeAdrenoGpuPublished)
public:
    IOReturn newUserClient(task_t owningTask,void *securityID,UInt32 type,
        OSDictionary *properties,IOUserClient **handler) APPLE_KEXT_OVERRIDE;
};
OSDefineMetaClassAndStructors(NativeAdrenoGpuPublished,IOService)

class AGXDeviceUserClient : public IOUserClient {
    OSDeclareDefaultStructors(AGXDeviceUserClient)
    bool live_=false;
    bool closing_=false;
    bool handoffOnly_=false;
    IOBufferMemoryDescriptor *traceSnapshots_[2]{};
    task_t handoffTask_=nullptr;
    bool kernel_=false;
    NativeAdrenoUserClient *core_=nullptr;   /* own core; deleted in free(), so a call that raced the close sees live_ false, never freed memory */
    unsigned slot_=NativeAdrenoMaxClients;
    char name_[20]{};
    /* One user-view descriptor per BO token, reused by every clientMemoryForType call. IOConnectUnmapMemory64 asks clientMemoryForType for
     * the descriptor again and unmaps only a mapping that belongs to THAT descriptor (kIOMapReference); a fresh descriptor per call made the
     * unmap a silent no-op, the user PTEs outlived BoCpuUnmap, and BoFree handed pages with a live user mapping to the VM:
     * pmap_assert_free panicked the freeing core and the panic screen parked it (the "BoFree hang", dead touch). */
    struct View {uint32_t token;IOMemoryDescriptor *descriptor;};
    /* One view per live BO token. The cache must hold every BO the client can have (NativeAdrenoClientMaxObjects): a BO whose view is not
     * cached makes IOConnectUnmapMemory64 (which asks clientMemoryForType again) a no-op, the user mapping outlives BoFree and the freed pages panic
     * pmap_assert_free. A cache of 64 held until the render server's hundreds of mapped BOs (int-087). */
    enum {MaxViews=NativeAdrenoClientMaxObjects};
    View *views_=nullptr;   /* on the heap: 1024 entries would make the client object too big for the kernel allocator's fixed-size classes */
    IOReturn dropViews(uint32_t token);
    static IOReturn action(OSObject *target,void *reference,IOExternalMethodArguments *arguments);
    IOReturn run(uint32_t method,const void *input,size_t input_size,void *output,size_t output_size);
public:
    void setHandoffOnly(task_t task) {handoffOnly_=true;handoffTask_=task;}
    IOReturn mount(bool kernel);
    friend IOReturn runCore(AGXDeviceUserClient *,uint32_t,const void *,size_t,void *,size_t);
    friend void nativeAdrenoMailboxPoller(void *,wait_result_t);
    IOReturn clientClose() APPLE_KEXT_OVERRIDE;
    void free() APPLE_KEXT_OVERRIDE;
    IOReturn clientMemoryForType(UInt32 type,IOOptionBits *options,IOMemoryDescriptor **memory) APPLE_KEXT_OVERRIDE;
    IOReturn externalMethod(uint32_t selector,IOExternalMethodArguments *arguments,
        IOExternalMethodDispatch *dispatch,OSObject *target,void *reference) APPLE_KEXT_OVERRIDE;
};
OSDefineMetaClassAndStructors(AGXDeviceUserClient,IOUserClient)

void nativeAdrenoMailboxPoller(void *,wait_result_t)
{
    thread_set_thread_name(current_thread(),"sheng-gpu-mailbox");
    uint32_t last=0;
    for(;;) {
        IOSleep(1);
        if(!gMailboxOwner || !gMailboxWords)continue;
        const uint32_t request=gMailboxWords[1];
        if(request==last || request==gMailboxWords[2])continue;
        last=request;
        const uint32_t method=gMailboxWords[3],in_size=gMailboxWords[5],out_size=gMailboxWords[6];
        IOReturn status=kIOReturnBadArgument;
        if(in_size<=0x700 && out_size<=0x700) {
            uint8_t input[0x700],output[0x700];
            memcpy(input,const_cast<uint32_t *>(gMailboxWords)+0x10,in_size);
            IOLockLock(g.lock);
            AGXDeviceUserClient *owner=gMailboxOwner;
            if(owner)owner->retain();
            IOLockUnlock(g.lock);
            status=owner?runCore(owner,method,input,in_size,output,out_size):kIOReturnNotReady;
            if(owner)owner->release();
            if(!status)memcpy(const_cast<uint32_t *>(gMailboxWords)+0x200,output,out_size);
        }
        native_adreno_metal_diag.mb_calls=native_adreno_metal_diag.mb_calls+1;
        gMailboxWords[4]=uint32_t(status);
        __asm__ volatile("dmb sy" ::: "memory");
        gMailboxWords[2]=request;
        __asm__ volatile("dmb sy" ::: "memory");
    }
}

/* The client log row of name (created for the first eight names); null when the table is full. Under g.lock. */
static NativeAdrenoClientRecord *clientRecord(const char *name)
{
    for(auto &record:native_adreno_client_log) {
        if(!record.name[0]) {strlcpy(record.name,name,sizeof record.name);record.slot=0xff;return &record;}
        if(!strncmp(record.name,name,sizeof record.name))return &record;
    }
    return nullptr;
}
IOReturn AGXDeviceUserClient::mount(bool kernel)
{
    proc_selfname(name_,sizeof name_);
    NativeAdrenoUserClient *core=new NativeAdrenoUserClient;
    if(!core)return kIOReturnNoMemory;
    IOLockLock(g.lock);
    bool ok=false;
    /* The render server takes slot 0 (the large window) when free; every other process, or a second render-server connection, the next free one. */
    unsigned slot=NativeAdrenoMaxClients;
    if(kernel)slot=g.kernel_client?NativeAdrenoMaxClients:0;
    else {
        if(!strncmp(name_,"backboardd",sizeof name_) && !g.slots[0])slot=0;
        if(!strncmp(name_,"SpringBoard",sizeof name_) && !g.slots[1])slot=1;
        for(unsigned i=2;slot==NativeAdrenoMaxClients && i<NativeAdrenoMaxClients;++i)if(!g.slots[i])slot=i;
    }
    const IOReturn opened=slot<NativeAdrenoMaxClients?core->open(NativeAdrenoAbiVersion,g.vm,g.submit,g.ring,g.input_bits,g.output_bits):kIOReturnNoSpace;
    IOReturn result=opened;
    if(opened!=kIOReturnSuccess) {native_adreno_mount_fail[0]=slot<NativeAdrenoMaxClients?2:1;native_adreno_mount_fail[1]=uint32_t(opened);native_adreno_mount_fail[2]=slot;native_adreno_mount_fail[3]=native_adreno_mount_fail[3]+1;}
    if(opened==kIOReturnSuccess) {
        if(g.timer)core->attachTimer(g.timer);
        if(g.info)core->attachDeviceInfo(g.info);
        if(const auto *status=pong_adreno_native_status())core->attachStatus(status);
        /* Queue ranges: the kernel client keeps range 0 alone (no user slot is live while the board's self-test runs); users get their slot's. */
        const NativeAdrenoClientWindow window=kernel?NativeAdrenoClientWindow{}:clientWindow(slot,name_);
        const IOReturn scoped=kernel?core->setScope(0,0,0):window.bytes?
            core->setScope(slot*NativeAdrenoAbiMaxQueues,window.start,window.bytes):kIOReturnNoSpace;
        result=scoped;
        if(scoped==kIOReturnSuccess) {
            core_=core;core=nullptr;slot_=slot;kernel_=kernel;live_=true;ok=true;
            if(kernel)g.kernel_client=true;else {
                g.windows[slot]=window;g.slots[slot]=true;
                IOLog("NativeAdreno: %s VA lease=%uMiB start=0x%llx slot=%u\n",name_,unsigned(window.bytes>>20),(unsigned long long)window.start,slot);
            }
            ++g.opened;
            if(NativeAdrenoClientRecord *record=clientRecord(name_))record->slot=slot;
        } else {
            native_adreno_mount_fail[0]=3;native_adreno_mount_fail[1]=uint32_t(scoped);native_adreno_mount_fail[2]=slot;native_adreno_mount_fail[3]=native_adreno_mount_fail[3]+1;
            core->releaseAll(&g.clock,0);
        }
    }
    /* The mailbox page and its poller serve the render server's connection only. */
    if(ok && !kernel && slot==0 && !strncmp(name_,"backboardd",sizeof name_)) {
        if(!gMailbox) {
            gMailbox=IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task,kIODirectionInOut,4096,4096);
            if(gMailbox) {gMailboxWords=static_cast<volatile uint32_t *>(gMailbox->getBytesNoCopy());memset(const_cast<uint32_t *>(gMailboxWords),0,4096);}
        }
        if(gMailboxWords) {
            memset(const_cast<uint32_t *>(gMailboxWords),0,4096);
            gMailboxWords[0]=NativeAdrenoMailboxMagic;
            if(!gPollerStarted) {
                thread_t thread=nullptr;
                if(kernel_thread_start(nativeAdrenoMailboxPoller,nullptr,&thread)==KERN_SUCCESS) {thread_deallocate(thread);gPollerStarted=true;}
            }
        }
        if(gMailboxOwner)gMailboxOwner->release();
        retain();gMailboxOwner=this;
    }
    IOLockUnlock(g.lock);
    if(core)delete core;
    if(result)IOLog("Pong Adreno: client %s mount failed slot=%u opened=0x%x result=0x%x\n",name_,slot,opened,result);
    return result;
}
/* Removes the cached view(s) of token (0 = all) and tears down every user mapping of them, so the BO's pages are never freed while a user
 * PTE still points at them, whether or not the client unmapped. */
IOReturn AGXDeviceUserClient::dropViews(uint32_t token)
{
    for(;;) {
        IOMemoryDescriptor *view=nullptr;
        uint32_t viewToken=0;
        IOLockLock(g.lock);
        if(views_)for(size_t i=0;i<MaxViews;++i) {View &v=views_[i];if(v.descriptor && (!token || v.token==token)) {view=v.descriptor;viewToken=v.token;v.descriptor=nullptr;v.token=0;break;}}
        IOLockUnlock(g.lock);
        if(!view)return kIOReturnSuccess;
        /* OSPtr is a raw retained pointer in this archive (shared pointers
         * are not enabled). The old boolean loop leaked that reference and
         * never unmapped its PTEs. Explicitly revoke before freeing BO pages,
         * including client death with maps still owned by an IOKit port. */
        while(IOMemoryMap *mapping=removeMappingForDescriptor(view)) {
            const IOReturn status=mapping->unmap();
            mapping->release();
            if(status) {
                // Keep the backing object alive on an actual VM failure.
                IOLockLock(g.lock);
                if(views_)for(size_t i=0;i<MaxViews;++i)if(!views_[i].descriptor) {
                    views_[i]={viewToken,view};view=nullptr;break;
                }
                IOLockUnlock(g.lock);
                return status;
            }
            native_adreno_metal_diag.forced_unmaps=native_adreno_metal_diag.forced_unmaps+1;
        }
        view->release();
    }
}
IOReturn AGXDeviceUserClient::clientClose()
{
    if(handoffOnly_ && handoffTask_) {pong_scanout_disconnect(handoffTask_);handoffTask_=nullptr;}
    IOLockLock(g.lock);
    closing_=true;
    IOLockUnlock(g.lock);
    const IOReturn unmap=dropViews(0);
    if(unmap)return unmap;
    IOLockLock(g.lock);
    IOReturn status=kIOReturnSuccess;
    AGXDeviceUserClient *mailbox=nullptr;
    if(gMailboxOwner==this) {mailbox=gMailboxOwner;gMailboxOwner=nullptr;}
    if(live_) {
        const uint64_t peak=core_->windowPeak();
        /* This client's unmaps alter the same table used by every other
         * client. Drain globally, yielding the gate between polls. Waiting
         * for two seconds inside releaseAll used to block every GPU caller
         * and could still unmap while another client's work was in flight. */
        const uint64_t began=g.clock.now_us(g.clock.cookie);
        for(;;) {
            status=pong_adreno_native_healthy()?g.submit->waitAll(&g.clock,0):kIOReturnNotReady;
            if(status!=kIOReturnTimeout || g.clock.now_us(g.clock.cookie)-began>=500000)break;
            IOLockUnlock(g.lock);
            if(g.clock.idle)g.clock.idle(g.clock.cookie);else IOSleep(1);
            IOLockLock(g.lock);
        }
        if(!status)status=core_->releaseAll(&g.clock,0);
        /* A failed teardown leaves the objects live and the slot taken: nothing is forced or reused. */
        if(!status) {
            live_=false;
            if(kernel_)g.kernel_client=false;else if(slot_<NativeAdrenoMaxClients)g.slots[slot_]=false;
            ++g.closed;native_adreno_metal_diag.closes=native_adreno_metal_diag.closes+1;
            if(NativeAdrenoClientRecord *record=clientRecord(name_)) {
                record->closes=record->closes+1;record->slot=0xff;
                if(peak>>20>record->peak_mib)record->peak_mib=uint32_t(peak>>20);
            }
        }
    }
    IOLockUnlock(g.lock);
    if(mailbox)mailbox->release();
    if(!status)terminate();
    return status;
}
void AGXDeviceUserClient::free()
{
    if(handoffOnly_ && handoffTask_) {pong_scanout_disconnect(handoffTask_);handoffTask_=nullptr;}
    for(auto *snapshot:traceSnapshots_)if(snapshot)snapshot->release();
    (void)dropViews(0);
    if(views_) {IODelete(views_,View,MaxViews);views_=nullptr;}
    /* Still live only when a teardown failed: its objects stay referenced by the VM and the ring, so the core is kept, not freed. */
    if(core_ && !live_) {delete core_;core_=nullptr;}
    IOUserClient::free();
}
IOReturn AGXDeviceUserClient::clientMemoryForType(UInt32 type,IOOptionBits *options,IOMemoryDescriptor **memory)
{
    if(memory)*memory=nullptr;
    if(!memory || !options)return kIOReturnBadArgument;
    IOReturn status=kIOReturnNotFound;
    // Root-only snapshots use ordinary IOKit mapped memory, avoiding a new
    // UFS/export region or large inline Mach replies. Each returned buffer
    // owns its copied data until the helper unmaps it; no GPU gate is held.
    if(type==0x4754 || type==0x4755) {
        if(kauth_cred_getuid(kauth_cred_get())!=0)return kIOReturnNotPrivileged;
        if(!native_gpu_trace_enabled())return kIOReturnNotReady;
        const unsigned index=type==0x4754?0:1;
        if(traceSnapshots_[index]){traceSnapshots_[index]->retain();*memory=traceSnapshots_[index];*options=kIOMapAnywhere|kIOMapReadOnly;return kIOReturnSuccess;}
        const uint32_t capacity=type==0x4754?sizeof(NativeGpuTraceHeader)+
            (NativeGpuTraceCapacity+NativeGpuTraceMetadata+(gpuPixels?NativeGpuCaptureRecords*NativeGpuPixelSlots:0))*sizeof(NativeGpuTraceRecord):
            16+NativeGpuPixelSlots*(sizeof(NativeGpuPixelHeader)+NativeGpuPixelBytes);
        auto buffer=IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task,kIODirectionInOut,capacity,4096);
        if(!buffer)return kIOReturnNoMemory;
        const uint32_t size=type==0x4754?native_gpu_trace_snapshot(buffer->getBytesNoCopy(),capacity):
            native_gpu_pixel_snapshot(buffer->getBytesNoCopy(),capacity);
        if(!size){buffer->release();return kIOReturnNotReady;}
        buffer->setLength(size);traceSnapshots_[index]=buffer;buffer->retain();*memory=buffer;*options=kIOMapAnywhere|kIOMapReadOnly;return kIOReturnSuccess;
    }
    if(type==NativeAdrenoScanoutType) {
        if(!live_ && !handoffOnly_)return kIOReturnNotReady;
        IOMemoryDescriptor *scanout=native_gop_handoff_memory();
        if(!scanout)return kIOReturnNotReady;
        *memory=scanout;                        /* already retained for the caller */
        *options=kIOMapAnywhere|kIOMapInhibitCache;   /* firmware framebuffer: device memory, never cached by the CPU */
        return kIOReturnSuccess;
    }
    if(type==NativeAdrenoMailboxType) {
        if(live_ && gMailbox && gMailboxOwner==this) {
            gMailbox->retain();
            *memory=gMailbox;
            *options=kIOMapAnywhere;   /* ordinary cached memory: both sides are CPUs, so it is coherent */
            native_adreno_metal_diag.mb_maps=native_adreno_metal_diag.mb_maps+1;
            return kIOReturnSuccess;
        }
        return kIOReturnNotReady;
    }
    native_adreno_view_diag[0]=native_adreno_view_diag[0]+1;
    /* [5]: threads inside this function right now (a thread stuck here, on the gate or in a mapping, shows as a non-zero count that stays). */
    struct Inside {Inside() {__atomic_fetch_add(&native_adreno_view_diag[5],1,__ATOMIC_RELAXED);} ~Inside() {__atomic_fetch_sub(&native_adreno_view_diag[5],1,__ATOMIC_RELAXED);}} inside;
    IOLockLock(g.lock);
    if(closing_) {IOLockUnlock(g.lock);return kIOReturnNotReady;}
    if(!views_)views_=IONewZero(View,MaxViews);
    if(!views_) {IOLockUnlock(g.lock);return kIOReturnNoMemory;}
    for(size_t i=0;i<MaxViews;++i)if(views_[i].descriptor && views_[i].token==type) {
        views_[i].descriptor->retain();*memory=views_[i].descriptor;
        NativeAdrenoMemory *bo=live_?core_->memoryForToken(type):nullptr;
        *options=kIOMapAnywhere|((bo && bo->system())?0:kIOMapInhibitCache);
        native_adreno_view_diag[2]=native_adreno_view_diag[2]+1;
        IOLockUnlock(g.lock);
        return kIOReturnSuccess;
    }
    NativeAdrenoMemory *backing=live_?core_->memoryForToken(type):nullptr;
    if(backing && backing->system() && backing->descriptor()) {
        // Map the VM object whose prepare() owns the pages/cache attribute,
        // rather than creating a second physical pager over a wired UPL.
        for(size_t i=0;i<MaxViews;++i)if(!views_[i].descriptor) {
            IOMemoryDescriptor *descriptor=backing->descriptor();
            descriptor->retain();views_[i]={type,descriptor};
            descriptor->retain();*memory=descriptor;*options=kIOMapAnywhere;
            status=kIOReturnSuccess;native_adreno_view_diag[1]++;break;
        }
    } else if(backing) {
        /* The BO's pages are cached in the kernel and the GPU's PTEs are non-cacheable, so the client needs an uncached
         * view. A mapping of the buffer descriptor with a different cache mode is not reliable, so describe the same raw
         * physical spans (the ones the SMMU maps) and ask for kIOMapInhibitCache, exactly how the board code maps the
         * ring and the GMU windows. The descriptor does not retain the pages: the client must unmap before BoCpuUnmap/BoFree. */
        /* A BO's pages can come scattered, one span per page (a 64 MiB BO is 16384 of them): the descriptor takes them all, as many as the GPU's
         * own table accepts. A fixed 64-entry array made every scattered BO above 256 KiB fail to map (VK_ERROR_MEMORY_MAP_FAILED, int-084),
         * only when memory happened to be fragmented. The descriptor copies the ranges. */
        enum {MaxSpans=65536};
        const size_t count=backing->spanCount();
        if(!count || count>MaxSpans)status=kIOReturnNoResources;
        else {
            IOAddressRange *ranges=IONewData(IOAddressRange,count);
            if(!ranges)status=kIOReturnNoMemory;
            else {
                for(size_t i=0;i<count;++i)ranges[i]={backing->spans()[i].physical,backing->spans()[i].bytes};
                IOMemoryDescriptor *descriptor=IOMemoryDescriptor::withAddressRanges(ranges,(UInt32)count,
                    static_cast<IOOptionBits>(kIODirectionInOut)|static_cast<IOOptionBits>(kIOMemoryTypePhysical64),nullptr);
                IODeleteData(ranges,IOAddressRange,count);
                if(!descriptor)status=kIOReturnNoMemory;
                else {
                    bool cached=false;
                    for(size_t i=0;i<MaxViews;++i)if(!views_[i].descriptor) {views_[i].token=type;views_[i].descriptor=descriptor;descriptor->retain();cached=true;break;}
                    if(!cached) {
                        /* Never hand out a view that cannot be torn down again. */
                        descriptor->release();
                        status=kIOReturnNoResources;
                    } else {
                        *memory=descriptor;
                        *options=kIOMapAnywhere|kIOMapInhibitCache;
                        native_adreno_view_diag[1]=native_adreno_view_diag[1]+1;
                        status=kIOReturnSuccess;
                    }
                }
            }
        }
    }
    IOLockUnlock(g.lock);
    if(status){native_adreno_view_diag[3]=native_adreno_view_diag[3]+1;native_adreno_view_diag[4]=(uint32_t)status;}
    return status;
}
/* Fence waits as the callers see them (int-147; method 6's own statistics count every poll): [0] waits, [1] us waited in total, [2] pauses
 * taken outside the connection lock, [3] the longest wait in us. */

/* memlog also enables bounded diagnostics for the native memory selectors.
 * Take elapsed and lock timestamps on the GPU clock, and emit after unlock. */
static bool gBoTiming;
struct BoTiming {
    struct Receipts { unsigned initial, slow; };
    const char *operation=nullptr;
    Receipts *receipts=nullptr;
    uint64_t start=0, acquired=0, held=0, waited=0, prep=0, bytes=0;
    uint64_t drain=0, unwire=0;
    unsigned drain_passes=0;
    bool unmapped_system=false,deferred_unwire=false;
    size_t spans=0;

    explicit BoTiming(uint32_t method)
    {
        static Receipts counts[4];
        if(!gBoTiming)return;
        unsigned index;
        switch(method) {
        case NativeAdrenoMethodBoAlloc: operation="alloc";index=0;break;
        case NativeAdrenoMethodBoImport:operation="import";index=1;break;
        case NativeAdrenoMethodBoMap:   operation="map";index=2;break;
        case NativeAdrenoMethodBoFree:  operation="free";index=3;break;
        default:return;
        }
        receipts=&counts[index];
        start=now();
    }
    static uint64_t now() {return g.clock.now_us(g.clock.cookie);}
    void lock()
    {
        const uint64_t began=operation?now():0;
        IOLockLock(g.lock);
        if(operation) {acquired=now();waited+=acquired-began;}
    }
    void unlock()
    {
        if(operation)held+=now()-acquired;
        IOLockUnlock(g.lock);
    }
    void extent(NativeAdrenoUserClient *client,const void *handle)
    {
        if(!operation)return;
        uint64_t key=0;
        memcpy(&key,handle,sizeof key);
        if(auto *memory=client->memoryOf(key)) {
            bytes=memory->bytes();
            spans=memory->spanCount();
        }
    }
    static bool take(unsigned &count,unsigned limit)
    {
        unsigned used=__atomic_load_n(&count,__ATOMIC_RELAXED);
        while(used<limit) {
            if(__atomic_compare_exchange_n(&count,&used,used+1,true,
                __ATOMIC_RELAXED,__ATOMIC_RELAXED))return true;
        }
        return false;
    }
    void report(IOReturn status) const
    {
        if(!operation)return;
        const uint64_t elapsed=now()-start;
        if(!take(receipts->initial,2) &&
           (!(status || elapsed>=4000) || !take(receipts->slow,12)))return;
        char caller[20]={};
        proc_selfname(caller,sizeof caller);
        IOLog("PONG GPU MEM: %s %s pid=%d bytes=%llu spans=%zu elapsed_us=%llu lock_wait_us=%llu lock_hold_us=%llu prep_us=%llu status=0x%x\n",
            operation,caller,proc_selfpid(),(unsigned long long)bytes,spans,
            (unsigned long long)elapsed,(unsigned long long)waited,
            (unsigned long long)held,(unsigned long long)prep,
            static_cast<unsigned>(status));
        if(!strcmp(operation,"free"))
            IOLog("PONG GPU MEM FREE DETAIL: pid=%d bytes=%llu drain_us=%llu passes=%u unwire_us=%llu unmapped_system=%u deferred=%u\n",
                proc_selfpid(),(unsigned long long)bytes,(unsigned long long)drain,drain_passes,
                (unsigned long long)unwire,unmapped_system?1u:0u,deferred_unwire?1u:0u);
    }
};

// A detached system descriptor owns the wired pages until completion. Only
// the BO-free path can offer one, after GPU unmap and CPU view retirement.
static void unwireSystemOnWorker(thread_call_param_t,thread_call_param_t descriptor)
{
    auto *buffer=static_cast<IOBufferMemoryDescriptor *>(descriptor);
    const size_t bytes=buffer->getLength();
    NativeAdrenoSystemPages pages{buffer,bytes};
    const uint64_t began=gBoTiming?BoTiming::now():0;
    NativeAdrenoMemory::discardSystem(&pages);
    if(gBoTiming) {
        static unsigned reported;
        const uint64_t elapsed=BoTiming::now()-began;
        if(BoTiming::take(reported,8))
            IOLog("PONG GPU MEM UNWIRE: bytes=%zu worker_us=%llu\n",
                bytes,(unsigned long long)elapsed);
    }
    __atomic_store_n(&g.unwire_busy,false,__ATOMIC_RELEASE);
}
static bool deferLargeRenderServerUnwire(NativeAdrenoSystemPages &pages,const char *name)
{
    constexpr size_t mib=1024*1024;
    if(!g.unwire_call || !pages.descriptor || pages.bytes<32*mib || pages.bytes>64*mib ||
       strcmp(name,"backboardd"))return false;
    // One slot includes work in flight. Preserve the synchronous path when
    // it is occupied so deferred teardown cannot grow the wired footprint.
    bool idle=false;
    if(!__atomic_compare_exchange_n(&g.unwire_busy,&idle,true,false,
        __ATOMIC_ACQUIRE,__ATOMIC_RELAXED))return false;
    auto *descriptor=pages.descriptor;
    pages={};
    thread_call_enter1(g.unwire_call,descriptor);
    return true;
}

static constexpr size_t kRenderSpareBytes=64u*1024u*1024u;
static constexpr unsigned kRenderSpareRefillDelayMs=0;

static bool renderSpareHasHeadroom()
{
    const unsigned freePages=__atomic_load_n(&vm_page_free_count,__ATOMIC_RELAXED);
    const unsigned target=__atomic_load_n(&vm_page_free_target,__ATOMIC_RELAXED);
    return freePages>target && freePages-target>kRenderSpareBytes/page_size;
}

static bool renderSpareNeedsFill()
{
    for(auto &slot:g.prepared_spares)if(!__atomic_load_n(&slot,__ATOMIC_ACQUIRE))return true;
    return false;
}

static void scheduleRenderSpare(unsigned delayMs)
{
    if(!g.prewarm_call || !renderSpareNeedsFill())return;
    bool idle=false;
    if(!__atomic_compare_exchange_n(&g.prewarm_pending,&idle,true,false,
        __ATOMIC_ACQ_REL,__ATOMIC_RELAXED))return;
    if(!delayMs) {
        thread_call_enter(g.prewarm_call);
        return;
    }
    uint64_t deadline;
    clock_interval_to_deadline(delayMs,kMillisecondScale,&deadline);
    thread_call_enter1_delayed(g.prewarm_call,nullptr,deadline);
}

static void prepareRenderSpare(thread_call_param_t,thread_call_param_t)
{
    bool published=false;
    if(pong_adreno_native_healthy() && renderSpareHasHeadroom() && renderSpareNeedsFill()) {
        const uint64_t began=gBoTiming?BoTiming::now():0;
        auto *pages=IONew(NativeAdrenoSystemPages,1);
        IOReturn status=kIOReturnNoMemory;
        if(pages) {
            *pages={};
            status=NativeAdrenoMemory::prepareSystem(kRenderSpareBytes,pages);
            if(!status) {
                // Transfer the entire backing, including any prepared spans.
                for(auto &slot:g.prepared_spares) {
                    NativeAdrenoSystemPages *expected=nullptr;
                    if(__atomic_compare_exchange_n(&slot,&expected,pages,false,
                        __ATOMIC_RELEASE,__ATOMIC_RELAXED)) {
                        published=true;
                        break;
                    }
                }
            }
            if(!published) {
                NativeAdrenoMemory::discardSystem(pages);
                IODelete(pages,NativeAdrenoSystemPages,1);
            }
        }
        static unsigned reported;
        if(gBoTiming && BoTiming::take(reported,16)) {
            IOLog("PONG GPU PREWARM: 64 MiB status=0x%x ready=%u elapsed_us=%llu\n",
                static_cast<unsigned>(status),published?1u:0u,
                (unsigned long long)(BoTiming::now()-began));
        }
    }
    __atomic_store_n(&g.prewarm_pending,false,__ATOMIC_RELEASE);
    // Fill the other slot serially, including when a racing consumer
    // took the freshly published buffer before pending was cleared.
    if(published)scheduleRenderSpare(kRenderSpareRefillDelayMs);
}

#if PONG_GUEST_22H374
extern "C" void thread_get_thread_name(thread_t thread,char *name);
#endif

static void reportRenderSpareRequest(bool consumed,IOReturn status)
{
    const auto thread=current_thread();
    const char *role="other";
#if PONG_GUEST_22H374
    char name[MAXTHREADNAMESIZE]={};
    thread_get_thread_name(thread,name);
    if(!strcmp(name,"metal2vulkan-room"))role="room";
#endif
    IOLog("PONG GPU PREWARM: 64 MiB %s pid=%d status=0x%x role=%s tid=%llu\n",
        consumed?"consumed":"direct miss",proc_selfpid(),static_cast<unsigned>(status),
        role,static_cast<unsigned long long>(thread_tid(thread)));
}

static bool takeRenderSpare(NativeAdrenoSystemPages &pages)
{
    NativeAdrenoSystemPages *claimed=nullptr;
    for(auto &slot:g.prepared_spares) {
        claimed=__atomic_exchange_n(&slot,nullptr,__ATOMIC_ACQ_REL);
        if(claimed)break;
    }
    if(!claimed)return false;
    pages=*claimed;
    *claimed={};
    IODelete(claimed,NativeAdrenoSystemPages,1);
    static unsigned reported;
    if(gBoTiming && BoTiming::take(reported,24)) {
        reportRenderSpareRequest(true,kIOReturnSuccess);
    }
    scheduleRenderSpare(kRenderSpareRefillDelayMs);
    return true;
}

extern "C" {volatile uint64_t native_adreno_wait_outer[4];}
static IOReturn runCore(AGXDeviceUserClient *connection,uint32_t method,const void *input,size_t input_size,void *output,size_t output_size)
{
    if(method==NativeAdrenoMethodSetMetadata && input && input_size==sizeof(NativeAdrenoAbiMetadata)) {
        const NativeAdrenoAbiMetadata *meta=static_cast<const NativeAdrenoAbiMetadata *>(input);
        /* The raw test's line and the Turnip phase's line ("T ..." result or "W ..." watchdog sample) are kept apart, so neither hides the other. */
        const bool turnip=meta->bytes>=2 && (meta->text[0]=='T' || meta->text[0]=='W') && meta->text[1]==' ';
        char *line=turnip?native_adreno_metal_diag.report2:native_adreno_metal_diag.report;
        uint32_t n=meta->bytes<sizeof(native_adreno_metal_diag.report)-1?meta->bytes:uint32_t(sizeof(native_adreno_metal_diag.report)-1);
        memset(line,0,sizeof(native_adreno_metal_diag.report));
        memcpy(line,meta->text,n);
        native_adreno_metal_diag.reports=native_adreno_metal_diag.reports+1;
    }
    /* A call that is still running when the boot page is photographed shows as inflight = its method number + 1 (0 = none), so a hang in the
     * kernel is attributable to one ABI method. */
    if(!pong_adreno_native_healthy())return kIOReturnNotReady;
    native_adreno_metal_diag.inflight=method+1;
    native_adreno_metal_diag.inflight_since_calls=native_adreno_metal_diag.calls;
    IOReturn status;
    if(method==NativeAdrenoMethodWaitFence && connection->live_ && input && input_size==sizeof(NativeAdrenoAbiWaitFence) && g.clock.now_us) {
        /* int-147: a fence wait polls under the lock but pauses outside it. int-146 slept inside the lock (up to the caller's 1 ms slice per
         * call), so every other thread's submit, map or allocation waited for the GPU to finish unrelated work: a frame could not be queued
         * while the previous one ran. Each poll is the core's own zero-timeout wait (same checks, retirement and statistics); the pause is the
         * board's (until the next GPU interrupt, at most 100 us). */
        NativeAdrenoAbiWaitFence poll{};
        memcpy(&poll,input,sizeof(poll));
        const uint64_t requested_us=poll.timeout_ns/1000;
        /* Vulkan may request UINT64_MAX. Preserve retryable timeout semantics
         * without pinning one IOKit request forever if the GPU stops. */
        const uint64_t timeout_us=requested_us>500000?500000:requested_us;
        const uint64_t began=g.clock.now_us(g.clock.cookie);
        poll.timeout_ns=0;
        uint64_t waited=0,gate_max=0,dispatch_max=0,sleep_max=0;
        for(;;) {
            waited=g.clock.now_us(g.clock.cookie)-began;
            // Upstream dd43abd: unfinished CP work needs no global device
            // lock on every poll. Only the advisory fence-page read is
            // outside it; completion, timeout, failure and ownership checks
            // still run through the existing locked dispatch below.
            if(waited<timeout_us && poll.fence<=UINT32_MAX &&
               __atomic_load_n(&connection->live_,__ATOMIC_ACQUIRE) &&
               !__atomic_load_n(&connection->closing_,__ATOMIC_ACQUIRE) && g.ring) {
                const uint32_t fence=uint32_t(poll.fence);
                const uint64_t sleep_start=g.clock.now_us(g.clock.cookie);
                const bool paused=g.clock.idle_if_fence_pending
                    ? g.clock.idle_if_fence_pending(g.clock.cookie,fence)
                    : g.ring->fencePending(fence) && g.clock.idle && !g.clock.idle(g.clock.cookie);
                if(paused) {
                    const uint64_t elapsed=g.clock.now_us(g.clock.cookie)-sleep_start;
                    if(elapsed>sleep_max)sleep_max=elapsed;
                    native_adreno_wait_outer[2]=native_adreno_wait_outer[2]+1;
                    continue;
                }
            }
            /* The connection's own core while it is live; a close that ran between two polls ends the wait. */
            const uint64_t gate_start=g.clock.now_us(g.clock.cookie);
            IOLockLock(g.lock);
            const uint64_t gate_end=g.clock.now_us(g.clock.cookie);
            if(gate_end-gate_start>gate_max)gate_max=gate_end-gate_start;
            status=connection->live_ && !connection->closing_ && pong_adreno_native_healthy()?
                connection->core_->dispatch(method,&poll,sizeof(poll),output,output_size,&g.clock):kIOReturnNotReady;
            IOLockUnlock(g.lock);
            const uint64_t dispatch_end=g.clock.now_us(g.clock.cookie);
            if(dispatch_end-gate_end>dispatch_max)dispatch_max=dispatch_end-gate_end;
            waited=dispatch_end-began;
            if(status!=kIOReturnTimeout || waited>=timeout_us)break;
            native_adreno_wait_outer[2]=native_adreno_wait_outer[2]+1;
            if(!g.clock.idle || g.clock.idle(g.clock.cookie))break;
        }
        native_adreno_wait_outer[0]=native_adreno_wait_outer[0]+1;native_adreno_wait_outer[1]=native_adreno_wait_outer[1]+waited;
        if(waited>native_adreno_wait_outer[3])native_adreno_wait_outer[3]=waited;
        static unsigned slow_receipts;
        if(waited>=16000 && __atomic_load_n(&slow_receipts,__ATOMIC_RELAXED)<48 &&
           __atomic_fetch_add(&slow_receipts,1,__ATOMIC_RELAXED)<48) {
            IOLog("[gpu waiter] pid=%d fence=%llu us=%llu gate_max_us=%llu dispatch_max_us=%llu sleep_max_us=%llu status=0x%x\n",
                proc_selfpid(),(unsigned long long)poll.fence,(unsigned long long)waited,
                (unsigned long long)gate_max,(unsigned long long)dispatch_max,(unsigned long long)sleep_max,
                static_cast<unsigned>(status));
        }
    } else {
        BoTiming timing(method);
        NativeAdrenoSystemPages pages{},freed{};
        const bool allocating=method==NativeAdrenoMethodBoAlloc && input && output &&
            input_size==sizeof(NativeAdrenoAbiBoAlloc) && output_size==sizeof(NativeAdrenoAbiBoAllocReply);
        if(allocating) {
            NativeAdrenoAbiBoAlloc request{};memcpy(&request,input,sizeof(request));
            timing.bytes=request.bytes;
            if(!request.contiguous && request.bytes && !(request.bytes&4095) &&
               request.bytes<=(UINT64_C(1)<<NativeAdrenoAbiMaxBoBytesShift) &&
               request.align_log2>=12 && request.align_log2<=24) {
                timing.lock();
                const bool live=connection->live_ && !connection->closing_;
                timing.unlock();
                if(!live) {timing.report(kIOReturnNotReady);return kIOReturnNotReady;}
                // Zero-fill, wire and apply cache mode outside the shared
                // device gate; unrelated submits and completions keep moving.
                const uint64_t prepStart=timing.operation?BoTiming::now():0;
                const bool eligible=g.prewarm_call && request.bytes==kRenderSpareBytes &&
                    !strcmp(connection->name_,"backboardd");
                const bool reused=eligible && takeRenderSpare(pages);
                const IOReturn prepared=reused?kIOReturnSuccess:
                    NativeAdrenoMemory::prepareSystem(size_t(request.bytes),&pages);
                if(eligible && !reused) {
                    static unsigned misses;
                    if(gBoTiming && BoTiming::take(misses,24)) {
                        reportRenderSpareRequest(false,prepared);
                    }
                    scheduleRenderSpare(kRenderSpareRefillDelayMs);
                }
                if(timing.operation)timing.prep=BoTiming::now()-prepStart;
                if(prepared) {timing.report(prepared);return prepared;}
            }
        }
        timing.lock();
        status=connection->live_ && !connection->closing_?kIOReturnSuccess:kIOReturnNotReady;
        if(!status && method==NativeAdrenoMethodBoFree) {
            if(input && input_size==sizeof(NativeAdrenoAbiBoFree) && output_size==0) {
                NativeAdrenoAbiBoFree request{};
                memcpy(&request,input,sizeof request);
                timing.unmapped_system=connection->core_->canFreeUnmappedSystemBo(request.handle);
            }
            /* The client's mapped resources may be referenced by its IBs, not just
             * the IB BO itself. Drain its whole queue range before a free;
             * disjoint clients no longer hold up each other's releases.
             * Poll under the gate, but never sleep with the gate held: other
             * clients must be able to observe/retire their own completions. */
            const uint64_t began=g.clock.now_us(g.clock.cookie);
            for(;;) {
                ++timing.drain_passes;
                status=g.submit->retireCompleted();
                // Keep the initial authoritative sweep/error propagation.
                // Unpublished private pages have no GPU address to drain.
                if(!status && !timing.unmapped_system && connection->core_->ownRangeOutstanding())
                    status=kIOReturnTimeout;
                if(status!=kIOReturnTimeout || g.clock.now_us(g.clock.cookie)-began>=500000)break;
                timing.unlock();
                if(g.clock.idle)g.clock.idle(g.clock.cookie);else IOSleep(1);
                timing.lock();
                if(!connection->live_ || connection->closing_ || !pong_adreno_native_healthy()) {
                    status=kIOReturnNotReady;break;
                }
            }
            if(timing.operation)timing.drain=BoTiming::now()-began;
        }
        if(!status) {
            if(input && ((method==NativeAdrenoMethodBoMap && input_size==sizeof(NativeAdrenoAbiBoMap)) ||
                (method==NativeAdrenoMethodBoFree && input_size==sizeof(NativeAdrenoAbiBoFree))))
                timing.extent(connection->core_,input);
            connection->core_->offerSystemPages(pages.descriptor?&pages:nullptr);
            if(method==NativeAdrenoMethodBoFree)connection->core_->collectFreedPages(&freed);
            status=connection->core_->dispatch(method,input,input_size,output,output_size,&g.clock);
            connection->core_->offerSystemPages(nullptr);
            connection->core_->collectFreedPages(nullptr);
            if(allocating && !status)timing.extent(connection->core_,output);
        }
        if(method==NativeAdrenoMethodBoMap && !status && connection->live_ && !connection->kernel_)
            if(NativeAdrenoClientRecord *record=clientRecord(connection->name_))
                if(connection->core_->windowPeak()>>20>record->peak_mib)record->peak_mib=uint32_t(connection->core_->windowPeak()>>20);
        timing.unlock();
        NativeAdrenoMemory::discardSystem(&pages);
        timing.deferred_unwire=deferLargeRenderServerUnwire(freed,connection->name_);
        if(!timing.deferred_unwire) {
            const uint64_t unwireStart=timing.operation && freed.descriptor?BoTiming::now():0;
            NativeAdrenoMemory::discardSystem(&freed);
            if(unwireStart)timing.unwire=BoTiming::now()-unwireStart;
        }
        timing.report(status);
    }
    native_adreno_metal_diag.inflight=0;
    native_adreno_metal_diag.calls=native_adreno_metal_diag.calls+1;
    native_adreno_metal_diag.last_selector=method;
    native_adreno_metal_diag.last_status=uint32_t(status);
    return status;
}
#if PONG_GUEST_22H374
// These receipts bypass the normal Metal startup telemetry budget. Failed
// polls, admission retries and documented host-sync fallbacks cannot use it.
static bool takeDesktopProducerReceipt(unsigned &count)
{
    char caller[20]={};
    proc_selfname(caller,sizeof caller);
    if(strcmp(caller,"backboardd"))return false;

    constexpr unsigned limit=8;
    unsigned used=__atomic_load_n(&count,__ATOMIC_RELAXED);
    while(used<limit) {
        if(__atomic_compare_exchange_n(&count,&used,used+1,true,__ATOMIC_RELAXED,__ATOMIC_RELAXED))
            return true;
    }
    return false;
}

/* A native fence wait returning success is evidence of CP completion.
 * Sample advancing fence IDs without reporting application frames or panel latches. */
static void reportDesktopFenceProgress(uint32_t method,const void *input,size_t input_size,IOReturn status)
{
    if(status || method!=NativeAdrenoMethodWaitFence || !input ||
       input_size!=sizeof(NativeAdrenoAbiWaitFence))return;

    NativeAdrenoAbiWaitFence request{};
    memcpy(&request,input,sizeof request);
    if(!request.fence || request.fence>UINT32_MAX)return;

    static unsigned receipts;
    static uint32_t reportedFence;
    const auto fence=static_cast<uint32_t>(request.fence);
    uint32_t previous=__atomic_load_n(&reportedFence,__ATOMIC_RELAXED);
    if(previous && (fence<=previous || fence-previous<128))return;
    if(takeDesktopProducerReceipt(receipts) &&
       __atomic_compare_exchange_n(&reportedFence,&previous,fence,false,__ATOMIC_RELAXED,__ATOMIC_RELAXED)) {
        IOLog("PONG GPU VERIFIED: backboardd pid=%d completed_fence=%u\n",proc_selfpid(),fence);
    }
}

static void reportDesktopProducerFailure(uint32_t method,const void *input,size_t input_size,IOReturn status)
{
    if(!status || !input)return;
    static unsigned acquireReceipts,submitReceipts,fenceReceipts;

    if(method==NativeAdrenoMethodSyncHostRange || method==NativeAdrenoMethodSyncHostBatch) {
        const bool batch=method==NativeAdrenoMethodSyncHostBatch;
        if(status==kIOReturnUnsupported || (!batch && status==kIOReturnNotFound))return;

        uint32_t phase=0,count=1;
        uint64_t bytes=0;
        if(batch && input_size==sizeof(NativeAdrenoAbiSyncHostBatch)) {
            memcpy(&phase,static_cast<const uint8_t *>(input)+offsetof(NativeAdrenoAbiSyncHostBatch,after),sizeof phase);
            memcpy(&count,input,sizeof count);
        } else if(!batch && input_size==sizeof(NativeAdrenoAbiSyncHostRange)) {
            NativeAdrenoAbiSyncHostRange request{};
            memcpy(&request,input,sizeof request);
            phase=request.after;
            bytes=request.bytes;
        }
        if(phase!=1 || !takeDesktopProducerReceipt(acquireReceipts))return;
        IOLog("PONG GPU PRODUCER: pid=%d host-acquire method=%u ranges=%u range_bytes=%llu status=0x%x\n",
            proc_selfpid(),method,count,(unsigned long long)bytes,static_cast<unsigned>(status));
        return;
    }

    if(method==NativeAdrenoMethodSubmit || method==NativeAdrenoMethodSubmitBatch) {
        const size_t expected=method==NativeAdrenoMethodSubmit?
            sizeof(NativeAdrenoAbiSubmit):sizeof(NativeAdrenoAbiSubmitBatch);
        if(input_size!=expected || status==kIOReturnBusy || status==kIOReturnTimeout ||
           !takeDesktopProducerReceipt(submitReceipts))return;
        uint32_t count=0;
        uint64_t queue=0,waitFence=0;
        static_assert(offsetof(NativeAdrenoAbiSubmit,queue)==offsetof(NativeAdrenoAbiSubmitBatch,queue) &&
            offsetof(NativeAdrenoAbiSubmit,wait_fence)==offsetof(NativeAdrenoAbiSubmitBatch,wait_fence),
            "Native Adreno shared submit header");
        const auto *raw=static_cast<const uint8_t *>(input);
        memcpy(&count,raw,sizeof count);
        memcpy(&queue,raw+offsetof(NativeAdrenoAbiSubmit,queue),sizeof queue);
        memcpy(&waitFence,raw+offsetof(NativeAdrenoAbiSubmit,wait_fence),sizeof waitFence);
        IOLog("PONG GPU PRODUCER: pid=%d submit method=%u queue=%llu buffers=%u wait_fence=%llu status=0x%x\n",
            proc_selfpid(),method,(unsigned long long)queue,count,(unsigned long long)waitFence,
            static_cast<unsigned>(status));
        return;
    }

    if(method==NativeAdrenoMethodWaitFence && input_size==sizeof(NativeAdrenoAbiWaitFence) &&
       status!=kIOReturnTimeout && takeDesktopProducerReceipt(fenceReceipts)) {
        NativeAdrenoAbiWaitFence request{};
        memcpy(&request,input,sizeof request);
        IOLog("PONG GPU PRODUCER: pid=%d fence-wait fence=%llu timeout_ns=%llu status=0x%x\n",
            proc_selfpid(),(unsigned long long)request.fence,(unsigned long long)request.timeout_ns,
            static_cast<unsigned>(status));
    }
}
#endif

IOReturn AGXDeviceUserClient::run(uint32_t method,const void *input,size_t input_size,void *output,size_t output_size)
{
    if(method==NativeGpuTraceMethod) {
        if(!input || input_size!=sizeof(NativeGpuTraceRecord) || output_size)return kIOReturnBadArgument;
        const auto *record=static_cast<const NativeGpuTraceRecord *>(input);
        if(record->event<30 || record->event>63)return kIOReturnBadArgument;
        native_gpu_trace_record(record->event,proc_selfpid(),record->value,12);return kIOReturnSuccess;
    }
    if(method==NativeGpuPixelMethod) {
        if(!input || input_size!=sizeof(NativeGpuPixelPiece) || output_size)return kIOReturnBadArgument;
        return native_gpu_pixel_put(static_cast<const NativeGpuPixelPiece *>(input),proc_selfpid())?kIOReturnNoResources:kIOReturnSuccess;
    }
    if(method>=NativeAdrenoHandoffBase && method<=NativeAdrenoHandoffLast)
        return native_gop_handoff_call(method-NativeAdrenoHandoffBase,current_task(),input,input_size,output,output_size);
    if(method==NativeAdrenoExportMethod) {
        struct Piece {uint32_t generation,total,offset,bytes;};
        char caller[20]={};proc_selfname(caller,sizeof caller);
        Piece piece{};
        if(input && input_size>=sizeof(piece))memcpy(&piece,input,sizeof(piece));
        const bool valid=!output_size && input && input_size>=sizeof(piece) && piece.bytes && piece.bytes<=NativeAdrenoExportMaxPiece &&
            input_size==sizeof(piece)+piece.bytes && !strncmp(caller,"backboardd",sizeof caller);
        const int stored=valid && boot_log_export_put?boot_log_export_put(piece.generation,piece.total,piece.offset,
            static_cast<const uint8_t *>(input)+sizeof(piece),piece.bytes):-1;
        if(stored) {native_adreno_export_stats[1]=native_adreno_export_stats[1]+1;return kIOReturnBadArgument;}
        native_adreno_export_stats[0]=native_adreno_export_stats[0]+1;
        native_adreno_export_stats[2]=piece.generation;native_adreno_export_stats[3]=piece.total;
        return kIOReturnSuccess;
    }
    if(method==NativeAdrenoMethodBoCpuUnmap && input && input_size==sizeof(NativeAdrenoAbiBoCpuUnmap)) {
        /* The view is shared by every CPU mapping of the BO, and dropViews removes all of them. A BO mapped twice (the view cache served 207
         * second requests in int-177) lost both mappings at the first unmap, and the other holder's next store hit an unmapped page: the
         * render server's crash in the command stream writes of vkCmdCopyBuffer. Only the last lease takes the view down; the earlier
         * ones leave their mapping until it, or the client unmaps it itself (IOConnectUnmapMemory64 works on the cached view). */
        const uint32_t token=static_cast<const NativeAdrenoAbiBoCpuUnmap *>(input)->token;
        IOLockLock(g.lock);
        const uint32_t leases=live_?core_->mappingLeases(token):0;
        IOLockUnlock(g.lock);
        if(leases>1)native_adreno_view_kept=native_adreno_view_kept+1;
        else {
            const IOReturn status=dropViews(token);
            if(status)return status;
        }
    }
    if(method==NativeAdrenoMethodBoImport) {
        /* The pages belong to the task making this call (an external method runs in the caller's context). */
        if(!input || input_size!=sizeof(NativeAdrenoAbiBoImport) || !output || output_size!=sizeof(NativeAdrenoAbiBoAllocReply))
            return kIOReturnBadArgument;
        NativeAdrenoAbiBoImport request{};
        memcpy(&request,input,sizeof(request));
        NativeAdrenoAbiBoAllocReply reply{};
        BoTiming timing(method);
        timing.bytes=request.bytes;
        native_adreno_metal_diag.inflight=method+1;
        NativeAdrenoTaskPages pages{};
        const task_t task=current_task();
        IOLockLock(g.import_lock);
        timing.lock();
        const bool live=live_ && !closing_;
        timing.unlock();
        IOReturn status=kIOReturnNotReady;
        if(live) {
            const uint64_t prepStart=timing.operation?BoTiming::now():0;
            const IOReturn preparation=NativeAdrenoMemory::prepareTaskPages(task,request.address,
                (size_t)request.bytes,g.output_bits,(size_t)request.bytes,&pages);
            if(timing.operation)timing.prep=BoTiming::now()-prepStart;
            timing.lock();
            // No handle belongs to the detached pages until adoption. Even if
            // clientClose finished during the wire, the IOKit call retains us.
            if(live_ && !closing_)status=core_->importBo(&request,task,&reply,&pages,preparation);
            if(!status)timing.extent(core_,&reply.handle);
            timing.unlock();
        }
        NativeAdrenoMemory::discardTaskPages(&pages);
        IOLockUnlock(g.import_lock);
        timing.report(status);
        native_adreno_metal_diag.inflight=0;
        native_adreno_metal_diag.calls=native_adreno_metal_diag.calls+1;
        native_adreno_metal_diag.last_selector=method;
        native_adreno_metal_diag.last_status=uint32_t(status);
        if(!status)memcpy(output,&reply,sizeof(reply));
        return status;
    }
    const IOReturn status=runCore(this,method,input,input_size,output,output_size);
#if PONG_GUEST_22H374
    reportDesktopProducerFailure(method,input,input_size,status);
    reportDesktopFenceProgress(method,input,input_size,status);
#endif
    return status;
}
IOReturn AGXDeviceUserClient::action(OSObject *target,void *,IOExternalMethodArguments *arguments)
{
    AGXDeviceUserClient *connection=OSDynamicCast(AGXDeviceUserClient,target);
    if(!connection || !arguments)return kIOReturnBadArgument;
    return connection->run(arguments->selector,arguments->structureInput,arguments->structureInputSize,
        arguments->structureOutput,arguments->structureOutputSize);
}
IOReturn AGXDeviceUserClient::externalMethod(uint32_t selector,IOExternalMethodArguments *arguments,
    IOExternalMethodDispatch *,OSObject *target,void *reference)
{
    if((selector==NativeGpuTraceMethod || selector==NativeGpuPixelMethod) && arguments)
        return run(selector,arguments->structureInput,arguments->structureInputSize,
                   arguments->structureOutput,arguments->structureOutputSize);
    /* One entry per ABI method; sizes are the exact wire records. */
    static const IOExternalMethodDispatch table[]={
        {&action,0,0,0,sizeof(NativeAdrenoAbiParams)},
        {&action,0,sizeof(NativeAdrenoAbiBoAlloc),0,sizeof(NativeAdrenoAbiBoAllocReply)},
        {&action,0,sizeof(NativeAdrenoAbiBoMap),0,sizeof(NativeAdrenoAbiBoMapReply)},
        {&action,0,sizeof(NativeAdrenoAbiBoFree),0,0},
        {&action,0,sizeof(NativeAdrenoAbiSubmit),0,sizeof(NativeAdrenoAbiSubmitReply)},
        {&action,0,sizeof(NativeAdrenoAbiWaitFence),0,0},
        {&action,0,0,0,sizeof(NativeAdrenoAbiTimestamp)},
        {&action,0,0,0,sizeof(NativeAdrenoAbiStatus)},
        {&action,0,sizeof(NativeAdrenoAbiMetadata),0,0},
        {&action,0,sizeof(NativeAdrenoAbiMetadata),0,sizeof(NativeAdrenoAbiMetadata)},
        {&action,0,sizeof(NativeAdrenoAbiQueue),0,sizeof(NativeAdrenoAbiQueueReply)},
        {&action,0,sizeof(NativeAdrenoAbiQueue),0,0},
        {&action,0,sizeof(NativeAdrenoAbiBoCpuMap),0,sizeof(NativeAdrenoAbiBoCpuMapReply)},
        {&action,0,sizeof(NativeAdrenoAbiBoCpuUnmap),0,0},
        {&action,0,0,0,sizeof(NativeAdrenoAbiDeviceInfo)},
        {&action,0,sizeof(NativeAdrenoAbiBoImport),0,sizeof(NativeAdrenoAbiBoAllocReply)},
        {&action,0,sizeof(NativeAdrenoAbiSubmitBatch),0,sizeof(NativeAdrenoAbiSubmitReply)},
        {&action,0,sizeof(NativeAdrenoAbiPriority),0,0},
        {&action,0,sizeof(NativeAdrenoAbiSyncHostRange),0,0},
        {&action,0,sizeof(NativeAdrenoAbiSyncHostBatch),0,sizeof(NativeAdrenoAbiSyncHostBatchReply)}};
    native_adreno_metal_diag.ext_entries=native_adreno_metal_diag.ext_entries+1;
    if(selector<256)native_adreno_metal_diag.sel_bits[selector>>5]|=1u<<(selector&31);
    if(selector>=NativeAdrenoHandoffBase && selector<=NativeAdrenoHandoffLast) {
        if(!handoffOnly_ || !arguments)return kIOReturnNotPermitted;
        return run(selector,arguments->structureInput,arguments->structureInputSize,
            arguments->structureOutput,arguments->structureOutputSize);
    }
    /* Tunnel. The sandbox resolves a filter policy for this class at open time and then admits only the selectors its profile lists,
     * so which selector numbers reach this object is decided outside the driver. A tunnel call carries the real ABI method in its
     * first eight bytes and may use ANY admitted selector: {magic, method, payload...} in, the method's exact output out. Method
     * 0xffffffff is a probe that only proves the selector was admitted. */
    if(arguments && arguments->structureInput && arguments->structureInputSize>=8 &&
        static_cast<const uint32_t *>(arguments->structureInput)[0]==NativeAdrenoTunnelMagic) {
        const uint32_t *header=static_cast<const uint32_t *>(arguments->structureInput);
        if(header[1]==0xffffffffu) {
            native_adreno_metal_diag.probes=native_adreno_metal_diag.probes+1;
            return kIOReturnSuccess;
        }
        native_adreno_metal_diag.tunnel_calls=native_adreno_metal_diag.tunnel_calls+1;
        const IOReturn tunneled=run(header[1],reinterpret_cast<const uint8_t *>(arguments->structureInput)+8,
            arguments->structureInputSize-8,arguments->structureOutput,arguments->structureOutputSize);
        native_adreno_metal_diag.ext_status=uint32_t(tunneled);
        return tunneled;
    }
    native_adreno_metal_diag.ext_selector=selector;
    native_adreno_metal_diag.ext_in=arguments?arguments->structureInputSize:0xffff;
    native_adreno_metal_diag.ext_out=arguments?arguments->structureOutputSize:0xffff;
    if(selector<1 || selector>sizeof(table)/sizeof(table[0])) {
        native_adreno_metal_diag.ext_status=uint32_t(kIOReturnUnsupported);
        return kIOReturnUnsupported;
    }
    const IOReturn result=IOUserClient::externalMethod(selector,arguments,const_cast<IOExternalMethodDispatch *>(&table[selector-1]),
        target?target:this,reference);
    native_adreno_metal_diag.ext_status=uint32_t(result);
    return result;
}
/* limit > 0 caps the connections ever granted: a plugin that crashes its host process on every launch (the process respawns and
 * loads it again) must not be able to keep a system process in a crash loop, so after the cap further opens are refused. */
extern "C" IOReturn native_adreno_new_user_client(IOService *provider,task_t owningTask,void *securityID,UInt32 type,
    OSDictionary *properties,IOUserClient **handler,unsigned limit)
{
    if(handler)*handler=nullptr;
    if(!handler || !g.published || (type!=0x4754 && !pong_adreno_native_healthy()))return kIOReturnNotReady;
    if(type==0x4754 && kauth_cred_getuid(kauth_cred_get())!=0)return kIOReturnNotPrivileged;
    /* Progress markers: open types 0x4147xxxx (xxxx nonzero) are not connections. The plugin sends one before and after each step so
     * that a sandbox that admits neither selectors nor mapping, or a crash, still leaves a trace of how far it got. */
    if((type>>16)==NativeAdrenoPluginOpenType && (type&0xffff)) {
        const uint32_t value=type&0xffff,slot=native_adreno_metal_diag.marks_total&15;
        /* Text stream into report2, one character per marker: 0x7fff starts a line, 0x70cc appends character cc. It needs no connection,
         * so the plugin's Turnip watchdog can report while Turnip itself holds its connection slot. Not counted as markers. */
        if(value>=0x8000) {
            static unsigned row=48,length;
            if(value>=0xC000) {
                row=value&0x3f;length=0;
                if(row<48)memset(native_adreno_metal_rows[row],0,sizeof(native_adreno_metal_rows[row]));
                native_adreno_metal_rows_seq=native_adreno_metal_rows_seq+1;
            } else if(row<48) {
                const char pair[2]={char((value>>7)&0x7f),char(value&0x7f)};
                for(unsigned i=0;i<2;++i)if(pair[i] && length<sizeof(native_adreno_metal_rows[row])-1)native_adreno_metal_rows[row][length++]=pair[i];
            }
            return kIOReturnUnsupported;
        }
        if(value>=0x7000 && value<0x8000) {
            /* 0x7ffe starts a line in report (GPU7) instead: the Metal bridge's result, after the raw test's line has been read. */
            static uint32_t length;
            static char *line=native_adreno_metal_diag.report2;
            static_assert(sizeof(native_adreno_metal_diag.report)==sizeof(native_adreno_metal_diag.report2),"stream lines");
            if(value==0x7fff || value==0x7ffe) {
                line=value==0x7fff?native_adreno_metal_diag.report2:native_adreno_metal_diag.report;
                /* A line from a process other than the render server goes to that process's own row (native_adreno_client_text): the render server
                 * rewrites report2 every ten seconds, so the reason another process's device failed would be gone from it. */
                char who[20]={};
                proc_selfname(who,sizeof who);
                if(value==0x7fff && strncmp(who,"backboardd",sizeof who)) {
                    for(auto &entry:native_adreno_client_text) {
                        if(!entry.name[0]) strlcpy(entry.name,who,sizeof entry.name);
                        if(!strncmp(entry.name,who,sizeof entry.name)) {line=entry.text;break;}
                    }
                }
                memset(line,0,sizeof(native_adreno_metal_diag.report2));length=0;
            }
            else if(value<0x7100 && length<sizeof(native_adreno_metal_diag.report2)-1) {line[length++]=char(value&0xff);line[length]=0;}
            return kIOReturnUnsupported;
        }
        native_adreno_metal_diag.marks[slot]=value;
        native_adreno_metal_diag.marks_total=native_adreno_metal_diag.marks_total+1;
        if(value==1) {
            memcpy(native_adreno_metal_diag.starter[2],native_adreno_metal_diag.starter[1],17);
            memcpy(native_adreno_metal_diag.starter[1],native_adreno_metal_diag.starter[0],17);
            proc_selfname(native_adreno_metal_diag.starter[0],17);
        }
        return kIOReturnUnsupported;
    }
    shiftName(native_adreno_metal_diag.opener);
    AGXDeviceUserClient *connection=new AGXDeviceUserClient;
    if(!connection)return kIOReturnNoMemory;
    if(!connection->initWithTask(owningTask,securityID,type,properties)) {
        connection->release();
        native_adreno_metal_diag.bad=native_adreno_metal_diag.bad+1;
        return kIOReturnNoResources;
    }
    /* IOService::newUserClient attaches and starts what it creates (a client without a provider chain is also a reason for the sandbox to
     * refuse). Then the two sandbox gates IOServiceOpen would apply afterwards are run here for every opener and logged with the
     * process name and open type, so the boot page shows which processes the sandbox lets reach this class and with which type. */
    unsigned attach_state=0;
    if(provider) {
        const bool attached=connection->attach(provider);
        attach_state=attached?1:2;
        if(attached && !connection->start(provider)) {
            connection->detach(provider);
            attach_state=3;
        }
    }
    native_adreno_metal_diag.attach_status=attach_state;
    const uint32_t macf=uint32_t(mac_iokit_check_open(kauth_cred_get(),connection,type));
    uint32_t resolver=0xfeedfeed;
    if(gIOUCFilterCallbacks && gIOUCFilterCallbacks->io_filter_resolver) {
        io_filter_policy_t policy=0;
        const IOReturn resolved=gIOUCFilterCallbacks->io_filter_resolver(owningTask,connection,type,&policy);
        resolver=uint32_t(resolved);
        if(resolved==kIOReturnSuccess && policy && gIOUCFilterCallbacks->io_filter_release)gIOUCFilterCallbacks->io_filter_release(policy);
    }
    native_adreno_metal_diag.chk_type=type;
    native_adreno_metal_diag.chk_macf=macf;
    native_adreno_metal_diag.chk_res=resolver;
    {
        NativeAdrenoOpenRecord &record=native_adreno_open_log[native_adreno_open_log_count&7];
        proc_selfname(record.name,sizeof(record.name));
        record.type=type;record.macf=macf;record.res=resolver;record.attach=attach_state;record.foreign=type!=NativeAdrenoPluginOpenType;
        native_adreno_open_log_count=native_adreno_open_log_count+1;
    }
    if(type==0x4754) {
        if(attach_state!=1){connection->release();return kIOReturnNotReady;}
        *handler=connection;return kIOReturnSuccess; // root read-only snapshots, no GPU/scanout slot
    }
    if(type==NativeAdrenoHandoffOpenType) {
        const IOReturn handoff=pong_scanout_connect(owningTask);
        if(handoff) {connection->detach(provider);connection->release();return handoff;}
        connection->setHandoffOnly(owningTask);
        *handler=connection;
        return kIOReturnSuccess;
    }
    /* Only the Adreno plugin's open (type NativeAdrenoPluginOpenType) is served. The original IOGPU library opens the accelerator with type 1
     * (and an API value in the upper bits) for an IOGPUDeviceUserClient, which this driver is not; refuse it cleanly. */
    if(type!=NativeAdrenoPluginOpenType) {
        connection->release();
        native_adreno_metal_diag.foreign_opens=native_adreno_metal_diag.foreign_opens+1;
        native_adreno_metal_diag.foreign_type=type;
        return kIOReturnUnsupported;
    }
    /* Every process that runs the Adreno plugin opens with its type: on Apple hardware every Metal process opens the AGX user client, and the
     * original profiles here name no class of this driver for the render server (MACF 1 for the plugin's types at every boot since int-038).
     * The plugin's two open types are granted here, by this driver: the grant is a property of the client object that the kernel's open path
     * (native_sandbox_grant in patches/xnu-ipados18/0001-pong-darwin24.patch) honours in place of the sandbox verdict. The verdict is still taken above and logged.
     * Which processes run the plugin at all is the plugin's allow-list (NativeAdrenoMetal.clients). */
    const bool kernel=owningTask==kernel_task;
    char opener[20]={};
    proc_selfname(opener,sizeof opener);
    if(!kernel && (macf || resolver!=uint32_t(kIOReturnSuccess))) {
        connection->setProperty("NativeSandboxGrant",kOSBooleanTrue);
        native_adreno_metal_diag.grants=native_adreno_metal_diag.grants+1;
    }
    /* Bound live connections, not historical opens. A closed ABI preflight or
     * an exited wallpaper process must return its capacity. The VM also
     * enforces the global fixed slot limit and keeps failed teardowns leased. */
    const unsigned cap=strncmp(opener,"backboardd",sizeof opener)?(limit && limit<8?limit:8):limit;
    IOLockLock(g.lock);
    NativeAdrenoClientRecord *record=kernel?nullptr:clientRecord(opener);
    const bool capped=cap && record && record->opens>=record->closes && record->opens-record->closes>=cap;
    IOLockUnlock(g.lock);
    IOReturn refusal=kIOReturnSuccess;
    if(capped)refusal=kIOReturnBusy;
    else refusal=connection->mount(kernel);
    if(refusal) {
        connection->release();
        native_adreno_metal_diag.busy=native_adreno_metal_diag.busy+1;
        IOLockLock(g.lock);
        if(record) {record->refused=record->refused+1;record->last_refusal=uint32_t(refusal);}
        IOLockUnlock(g.lock);
        return refusal;
    }
    IOLockLock(g.lock);
    if(record)record->opens=record->opens+1;
    IOLockUnlock(g.lock);
    native_adreno_metal_diag.opens=native_adreno_metal_diag.opens+1;
    *handler=connection;
    return kIOReturnSuccess;
}
extern "C" void native_adreno_note_property_read(const char *key)
{
    if(key && !strcmp(key,"MetalPluginName")) {
        proc_selfname(native_adreno_metal_diag.name_reader,17);
        native_adreno_metal_diag.name_reads=native_adreno_metal_diag.name_reads+1;
        char name[20]={};
        proc_selfname(name,sizeof name);
        /* Unlocked: the discovery runs at process start, a lost count is harmless and a row is claimed by its first byte. */
        for(auto &asker:native_adreno_metal_askers) {
            if(!asker.name[0]) {strlcpy(asker.name,name,sizeof asker.name);asker.reads=1;break;}
            if(!strncmp(asker.name,name,sizeof asker.name)) {asker.reads=asker.reads+1;break;}
        }
    }
    shiftName(native_adreno_metal_diag.reader);
    native_adreno_metal_diag.property_reads=native_adreno_metal_diag.property_reads+1;
}
IOReturn NativeAdrenoGpuPublished::newUserClient(task_t owningTask,void *securityID,UInt32 type,
    OSDictionary *properties,IOUserClient **handler)
{
    return native_adreno_new_user_client(this,owningTask,securityID,type,properties,handler,0);
}

/* Publishes the service once. Everything passed in must outlive the system (the board probe keeps them in statics) and
 * the VM, submit object and ring must already be attached to the live GPU context. The core must not be open. */
extern "C" IOReturn native_adreno_publish(NativeAdrenoVm *vm,NativeAdrenoSubmit *submit,NativeAdrenoRing *ring,
    const NativeAdrenoRingClock *clock,const NativeAdrenoTimer *timer,const NativeAdrenoAbiDeviceInfo *info,
    uint32_t input_bits,uint32_t output_bits)
{
    if(g.published || !vm || !submit || !ring || !clock || !clock->now_us)return kIOReturnBadArgument;
    gpuTraceInit();
    g.lock=IOLockAlloc();
    g.import_lock=IOLockAlloc();
    NativeAdrenoGpuPublished *service=new NativeAdrenoGpuPublished;
    if(!g.lock || !g.import_lock || !service || !service->init() || !service->attach(IOService::getPlatform())) {
        if(service)service->release();
        if(g.import_lock) {IOLockFree(g.import_lock);g.import_lock=nullptr;}
        if(g.lock) {IOLockFree(g.lock);g.lock=nullptr;}
        return kIOReturnNoMemory;
    }
    g.vm=vm;g.submit=submit;g.ring=ring;g.clock=*clock;g.timer=timer;g.info=info;g.input_bits=input_bits;g.output_bits=output_bits;
    unsigned memlog=0;
    gBoTiming=PE_parse_boot_argn("memlog",&memlog,sizeof memlog) && memlog==1;
    g.unwire_call=thread_call_allocate_with_options(unwireSystemOnWorker,nullptr,
        THREAD_CALL_PRIORITY_USER,THREAD_CALL_OPTIONS_ONCE);
    g.service=service;g.published=true;
    char board[16]={};
    unsigned prewarm=1;
    (void)PE_parse_boot_argn("pong_bo_prewarm",&prewarm,sizeof prewarm);
    const bool thor=PE_parse_boot_argn("arp0",board,sizeof board) && !strcmp(board,"thor");
    if(thor && NativeAdrenoStartParallelCacheWorker() && gBoTiming) {
        IOLog("PONG GPU CACHE WORKER: ready for disjoint imports priority=kernel_high\n");
    }
    if(prewarm && thor) {
        g.prewarm_call=thread_call_allocate_with_options(prepareRenderSpare,nullptr,
            THREAD_CALL_PRIORITY_LOW,THREAD_CALL_OPTIONS_ONCE);
        if(g.prewarm_call && gBoTiming) {
            IOLog("PONG GPU PREWARM: policy slots=%zu refill_delay_ms=%u\n",
                sizeof(g.prepared_spares)/sizeof(g.prepared_spares[0]),kRenderSpareRefillDelayMs);
        }
        scheduleRenderSpare(2000);
    }
    gpuTraceEvent(GpuTraceConfig,0,1,0,0,input_bits,output_bits,0);
    service->setName("NativeAdrenoGpu");
    service->registerService();
    return kIOReturnSuccess;
}
extern "C" uint32_t native_adreno_publish_stats(uint32_t *closed)
{
    if(closed)*closed=g.closed;
    return g.opened;
}
extern "C" IOService *native_adreno_published_service()
{
    return g.service;
}
