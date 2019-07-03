#ifndef __GPU_DATA_LOADER__
#define __GPU_DATA_LOADER__

#include <queue>
#include <unordered_map>
#include <vector>

#include "debug/GPUDataLoader.hh"
#include "mem/mem_object.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "params/GpuDataLoader.hh"

#define pageSizeBits (12)
#define pageSize (1<<pageSizeBits)
#define memReqUnit (64)
#define memReqNum ((pageSize+(memReqUnit-1))/memReqUnit)

#define FINITE_GPU_MEM_CAPACITY (1)

class GpuDataLoader : public MemObject
{
public:
    class CoreSidePort : public MasterPort
    {
    public:
        CoreSidePort(const std::string& if_name, GpuDataLoader *_owner) :
            MasterPort(if_name, _owner), owner(_owner), portName(if_name)
        {
            if_sent_failed = false;
        }
        bool sendPacket(PacketPtr pkt);

    private:
        /* to check if sendPacket is ok or not */
        bool if_sent_failed;
        GpuDataLoader *owner;
        std::string portName;

    protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        void recvRangeChange() override;
    };

    typedef enum {
        NotExist = 0,
        Loading,
        Done,
        Invalid
    } GpuDataLoaderState;

    GpuDataLoader(GpuDataLoaderParams *params);

    BaseMasterPort& getMasterPort (const std::string& if_name,
                                  PortID idx = InvalidPortID);

    /* Data loaing */
    std::unordered_map<Addr,uint8_t*> gathered_data;
    std::unordered_map<Addr,uint32_t> gathered_data_pos;
    void loadPageFromCPU(Addr cpuPageAddr);
    void savePageToGPU(Addr cpuPageAddr, Addr gpuPageAddr, uint8_t *data);

    /* Data moving back */
    void loadPageFromGPU(Addr gpuPageAddr);
    void savePageToCPU(Addr cpuPageAddr, Addr gpuPageAddr, uint8_t* data);

    bool isPageLoaded(Addr cpuPageAddr) {
        return (gpuAddrMap.find(cpuPageAddr) != gpuAddrMap.end());
    }
    GpuDataLoaderState getState(Addr corePageAddr) {
        if (stateMap.find(corePageAddr) == stateMap.end()) {
            return NotExist;
        }
        return stateMap[corePageAddr];
    }

    /* Evict gpu pages */
    bool evictingAll;
    bool evictAllDone;
    void evictAllPages();
    bool isEvictingAll() { return evictingAll; }
    bool isEvictAllDone() { return evictAllDone; }

    Addr getGpuPageAddr(Addr cpuPageAddr) {
        return (cpuPageAddr | 0x100000000);
    }
    Addr getCpuPageAddr(Addr gpuPageAddr) {
        return (gpuPageAddr & ~(0x100000000));
    }

    /* Dirty pages */
    typedef enum {
        Shared = 0,
        Dirty
    } PageStatus;
    std::unordered_map<Addr,PageStatus> pageStatusMap;
    void setPageStatus(Addr pageAddr, PageStatus _status);

    /* Static methods and members */
    static GpuDataLoader *gpuDataLoader;
    static void setInstance(GpuDataLoader *instance) {
        GpuDataLoader::gpuDataLoader = instance;
    }
    static GpuDataLoader* getInstance() { return gpuDataLoader; }

    /* Coherence between Mem Ctrl and Ruby Cache */
    typedef enum {
        RubyPhysMem = 0,
        MemCtrls0 = 1,
        MemCtrls1 = 2
    } AbstractMemType;

    bool isSetAbstractMem = false;
    std::vector<uint8_t*> abstractMem;
    std::vector<AddrRange> abstractMemRange;
    void setAbstractMem(std::string name, uint8_t* pmem, AddrRange range);
    void makeDataCoherent(Addr addr, uint64_t size);
    void syncToMainMem(Addr addr, const uint8_t* data, uint64_t size);

private:
    CoreSidePort cpuMemPort;
    CoreSidePort gpuMemPort;

    enum LoaderAct {
        MOVE_GPU = 0,
        MOVE_CPU,
        LoaderActNum
    };

    typedef enum {
        CPU = 0,
        GPU,
        Unknown
    } ProcUnit;

    /* Buffer packets */
    std::vector< std::queue<PacketPtr> > blockedPackets;

    typedef enum {
        FROM_CPU = 0,
        FROM_GPU,
        TO_CPU,
        TO_GPU,
        LoaderTransNum
    } LoaderTrans;

    std::vector< std::unordered_map<Addr,uint32_t> > recvRespNum;

    unsigned long gpuAddrMask;
    /* Address Hash Map
     * cpu page-aligned address, gpu page-aligned address
     * */
    std::unordered_map<Addr,Addr> gpuAddrMap;

    /* gpu page-aligned address, cpu page-aligned address */
    std::unordered_map<Addr,Addr> cpuAddrMap;

    /* cpu page-aligned address, gpu load state for address */
    std::unordered_map<Addr,GpuDataLoaderState> stateMap;

    /* evict page, move back to cpu */
    EventFunctionWrapper checkEvictEvent;
    void checkEvictDone();
    void evictPage(Addr gpuPageAddr);
    void evictPage(Addr gpuPageAddr, Addr nextLoadedPageAddr);
    /* <gpuPageAddr, cpuPageAddr> record which cpu page
     * should be loaded after evicting the gpu page */
    std::unordered_map<Addr,Addr> toLoadPageAfterEvict;

    bool loadingDone;

    bool handleResponse(PacketPtr pkt, std::string& port);

    System* const system;
    const MasterID masterId;

    struct GDLSenderState : public Packet::SenderState
    {
        GDLSenderState()
        {
            targetCore = Unknown;
            trans = LoaderTransNum;
        }
        ProcUnit targetCore;
        LoaderTrans trans;
    };

    void regStats() override;

    /* Mechanism of Evicting Pages when full */
    typedef enum {
        SC_TRUE,
        SC_FALSE,
        SC_INVALID
    } SC_Status;
    //typedef Addr Capacity;
    uint64_t gpuMemCapacity;
    Addr baseAddr;
    /* <CPU Page Addr> */
    std::vector<Addr> secondChanceVec;
    std::vector<SC_Status> secondChanceStatus;
    uint32_t secondChanceInd;
    void handleMissingPage(Addr cpuPageAddr);

protected:

    AddrRange cpuMemRange;
    AddrRange gpuMemRange;

public:
    // Statistics
    Stats::Scalar numGPUAccesses;
    Stats::Scalar numCPUAccesses;
    Stats::Scalar numPageMisses;
    Stats::Scalar numPageHits;
    Stats::Scalar numEvictedPages;
    Stats::Scalar numUniqPagesLoaded;
};

#endif
