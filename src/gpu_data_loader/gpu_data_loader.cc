#include "gpu_data_loader/gpu_data_loader.hh"

#include "base/chunk_generator.hh"
#include "debug/GPUDataLoader.hh"
#include "mem/abstract_mem.hh"
#include "sim/system.hh"

#define pageSizeBits (12)
#define pageSize (1<<pageSizeBits)

#define DUMPDATA(Flag, Data) \
do \
{ \
    DPRINTF(Flag, "%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x\n", \
                  Data[0], Data[1], Data[2], Data[3], \
                  Data[4], Data[5], Data[6], Data[7], \
                  Data[8], Data[9], Data[10],Data[11], \
                  Data[12],Data[13],Data[14], Data[15]);\
}while (0)

GpuDataLoader* GpuDataLoader::gpuDataLoader = NULL;

GpuDataLoader::GpuDataLoader(GpuDataLoaderParams *params):
    MemObject(params),
    cpuMemPort(params->name+"cpu_side_mem_port", this),
    gpuMemPort(params->name+"gpu_side_mem_port", this),
    checkEvictEvent([this]{ checkEvictDone(); },
                    "Check evicting all gpu page is done",
                    false, Event::CPU_Tick_Pri),
    loadingDone(false),
    system(params->sys),
    masterId(params->sys->getMasterId(this)),
    cpuMemRange(params->cpuMemRange),
    gpuMemRange(params->gpuMemRange)
{
    abstractMem.resize(3);
    abstractMemRange.resize(3);
    gpuAddrMask = 0x100000000;

    blockedPackets.resize(LoaderActNum);
    recvRespNum.resize(LoaderTransNum);

    evictingAll = false;
    evictAllDone = false;

    gpuMemCapacity = 0x100000000;
    baseAddr = 0x100000000;
    secondChanceInd = 0;

    num_gpu_mem_access = 0;
    avgMemAccessLatency = 0;
}

GpuDataLoader*
GpuDataLoaderParams::create()
{
    GpuDataLoader *instance = new GpuDataLoader(this);
    GpuDataLoader::setInstance(instance);
    return instance;
}

BaseMasterPort&
GpuDataLoader::getMasterPort (const std::string& if_name,
                             PortID idx)
{
    if (if_name == "cpu_side_mem_port") {
        return cpuMemPort;
    }
    else if (if_name == "gpu_side_mem_port") {
        return gpuMemPort;
    }
    else {
        return MemObject::getMasterPort(if_name, idx);
    }
}

bool
GpuDataLoader::handleResponse(PacketPtr pkt, std::string& port)
{
    DPRINTF(GPUDataLoader, "GpuDataLoader::handleResponse %s\n", port);

    auto senderState
        = dynamic_cast<GpuDataLoader::GDLSenderState*>(pkt->senderState);
    // FROM_CPU
    if (port == this->name()+"cpu_side_mem_port"
            && senderState->trans == FROM_CPU) {
        DPRINTF(GPUDataLoader,
                "GpuDataLoader::handleResponse FROM CPU packet "
                "with Addr %#x and size %u\n",
                pkt->getAddr(), pkt->getSize());
        Addr cpuPageAddr = pkt->getAddr() & (~(pageSize-1));
        recvRespNum[FROM_CPU][cpuPageAddr] += 1;

        DPRINTF(GPUDataLoader, "%s cpuPageAddr %#x, num %u/%u, offset %u\n",
                                __func__, cpuPageAddr,
                                recvRespNum[FROM_CPU][cpuPageAddr],
                                memReqNum, pkt->getAddr()-cpuPageAddr);
        DUMPDATA(GPUDataLoader, pkt->getPtr<uint8_t>());

        /* Gather Data from multiple packets */
        if (gathered_data.find(cpuPageAddr) == gathered_data.end()) {
            gathered_data[cpuPageAddr] = new uint8_t[pageSize];
            gathered_data_pos[cpuPageAddr] = 0;
        }
        uint64_t offset = pkt->getAddr() - cpuPageAddr;
        DPRINTF(GPUDataLoader, "adding offset: %x\n",
                               gathered_data[cpuPageAddr]+offset);
        memcpy(gathered_data[cpuPageAddr] + offset,
               pkt->getPtr<uint8_t>(),
               pkt->getSize());
        //gathered_data_pos[cpuPageAddr] += pkt->getSize();

        if (recvRespNum[FROM_CPU][cpuPageAddr] == memReqNum) {
            savePageToGPU(cpuPageAddr,
                          getGpuPageAddr(cpuPageAddr),
                          gathered_data[cpuPageAddr]);
        }
        //auto senderState = pkt->popSenderState();
        auto dataPtr = pkt->getPtr<uint8_t>();
        delete dataPtr;
        delete pkt;
        delete senderState;
    }
    // TO GPU
    else if (port == this->name()+"gpu_side_mem_port"
             && senderState->trans == TO_GPU) {
        Addr gpuPageAddr = pkt->getAddr() & (~(pageSize-1));
        Addr cpuPageAddr = getCpuPageAddr(gpuPageAddr);
        recvRespNum[TO_GPU][gpuPageAddr] += 1;

        DPRINTF(GPUDataLoader, "%s gpuPageAddr %#x, num %u/%u, offset %u\n",
                                __func__, gpuPageAddr,
                                recvRespNum[TO_GPU][gpuPageAddr], memReqNum,
                                pkt->getAddr()-gpuPageAddr);
        if (recvRespNum[TO_GPU][gpuPageAddr] == memReqNum) {
            stateMap[cpuPageAddr] = Done;
            gathered_data_pos[cpuPageAddr] = 0;

            /* means there is at least one page still in gpu memory */
            evictingAll = false;
            evictAllDone = false;

            /* set the page as shared */
            pageStatusMap[gpuPageAddr] = Shared;

            #if FINITE_GPU_MEM_CAPACITY
            /* clear loading after evict status */
            if (toLoadPageAfterEvict.find(gpuPageAddr)
                    != toLoadPageAfterEvict.end()) {
                toLoadPageAfterEvict.erase(gpuPageAddr);
            }
            #endif
            sumPageLoadLatency += curTick()-tickRecord[cpuPageAddr];
            avgPageLoadLatency = sumPageLoadLatency.value()
                                    / numUniqPagesLoaded.value();
            DPRINTF(GPUDataLoader, "page load latency %u\n",
                                   curTick() - tickRecord[cpuPageAddr]);
        }

        // update Stats
        assert(tickRecord.find(cpuPageAddr) != tickRecord.end());

        //auto senderState = pkt->popSenderState();
        delete pkt;
        delete senderState;
    }
    // FROM GPU
    else if (port == this->name()+"gpu_side_mem_port"
            && senderState->trans == FROM_GPU) {

        DPRINTF(GPUDataLoader, "GpuDataLoader::handleResponse FROM GPU packet "
                               "with Addr %#x and size %u\n",
                               pkt->getAddr(), pkt->getSize());
        Addr gpuPageAddr = pkt->getAddr() & (~(pageSize-1));
        recvRespNum[FROM_GPU][gpuPageAddr] += 1;

        DPRINTF(GPUDataLoader, "%s gpuPageAddr %#x, num %u/%u, offset %u\n",
                                __func__, gpuPageAddr,
                                recvRespNum[FROM_GPU][gpuPageAddr],
                                memReqNum, pkt->getAddr()-gpuPageAddr);
        DUMPDATA(GPUDataLoader, pkt->getPtr<uint8_t>());

        /* Gather Data from multiple packets */
        if (gathered_data.find(gpuPageAddr) == gathered_data.end()) {
            gathered_data[gpuPageAddr] = new uint8_t[pageSize];
            gathered_data_pos[gpuPageAddr] = 0;
        }
        uint64_t offset = pkt->getAddr() - gpuPageAddr;
        DPRINTF(GPUDataLoader, "adding offset: %x\n",
                                gathered_data[gpuPageAddr]+offset);
        memcpy(gathered_data[gpuPageAddr] + offset,
               pkt->getPtr<uint8_t>(),
               pkt->getSize());

        if (recvRespNum[FROM_GPU][gpuPageAddr] == memReqNum) {
            savePageToCPU(getCpuPageAddr(gpuPageAddr),
                          gpuPageAddr,
                          gathered_data[gpuPageAddr]);
        }
        //auto senderState = pkt->popSenderState();
        auto dataPtr = pkt->getPtr<uint8_t>();
        delete dataPtr;
        delete pkt;
        delete senderState;

    }
    // TO CPU
    else if (port == this->name()+"cpu_side_mem_port"
            && senderState->trans == TO_CPU) {
        Addr cpuPageAddr = pkt->getAddr() & (~(pageSize-1));
        Addr gpuPageAddr = getGpuPageAddr(cpuPageAddr);
        recvRespNum[TO_CPU][cpuPageAddr] += 1;

        DPRINTF(GPUDataLoader, "%s cpuPageAddr %#x, num %u/%u, offset %u\n",
                                __func__, cpuPageAddr,
                                recvRespNum[TO_CPU][cpuPageAddr], memReqNum,
                                pkt->getAddr()-cpuPageAddr);
        if (recvRespNum[TO_CPU][cpuPageAddr] == memReqNum) {
            stateMap[gpuPageAddr] = Done;
            stateMap[cpuPageAddr] = Invalid;
            gathered_data_pos[gpuPageAddr] = 0;

            #if FINITE_GPU_MEM_CAPACITY
            if (toLoadPageAfterEvict.find(gpuPageAddr)
                    != toLoadPageAfterEvict.end()) {
                Addr nextLoadPageAddr = toLoadPageAfterEvict[gpuPageAddr];
                loadPageFromCPU(nextLoadPageAddr);
                //toLoadPageAfterEvict.erase(gpuPageAddr);
            }
            #endif

            sumPageEvictLatency += curTick()-tickRecord[gpuPageAddr];
            avgPageEvictLatency = sumPageEvictLatency.value()
                                    / numEvictedPages.value();
            DPRINTF(GPUDataLoader, "page evict latency %u\n",
                                   curTick() - tickRecord[gpuPageAddr]);
        }

        // make mem_ctrls0 and ruby phys mem synchronized
        // it might be temporal, should make cpu cahce all
        // invalid afterwards so that it will read from
        // main memory
        if (!this->isSetAbstractMem) {
            uint8_t *ruby_phys_mem
                = AbstractMemory::getAbstractMem("system.ruby.phys_mem");
            uint8_t *mem_ctrls0_mem
                = AbstractMemory::getAbstractMem("system.mem_ctrls0");
            uint8_t *mem_ctrls1_mem
                = AbstractMemory::getAbstractMem("system.mem_ctrls1");
            AddrRange ruby_phys_mem_range
                = AbstractMemory::getAbstractMemRange("system.ruby.phys_mem");
            AddrRange mem_ctrls0_mem_range
                = AbstractMemory::getAbstractMemRange("system.mem_ctrls0");
            AddrRange mem_ctrls1_mem_range
                = AbstractMemory::getAbstractMemRange("system.mem_ctrls1");

            this->setAbstractMem("system.ruby.phys_mem",
                                          ruby_phys_mem,
                                          ruby_phys_mem_range);
            this->setAbstractMem("system.mem_ctrls0",
                                          mem_ctrls0_mem,
                                          mem_ctrls0_mem_range);
            this->setAbstractMem("system.mem_ctrls1",
                                          mem_ctrls1_mem,
                                          mem_ctrls1_mem_range);
        }
        DPRINTF(GPUDataLoader, "%s making coherent: addr %#x: %u\n",
                               __func__,
                               pkt->getAddr(),
                               pageStatusMap[gpuPageAddr]);
        //uint64_t offset = pkt->getAddr() - gpuPageAddr;

        if (pageStatusMap[gpuPageAddr] == Dirty)
            this->makeDataCoherent(pkt->getAddr(), pkt->getSize());


        //auto senderState = pkt->popSenderState();
        delete pkt;
        delete senderState;

    }
    else {
        // should not happen...
        assert(true);
    }
    return true;
}

void
GpuDataLoader::loadPageFromCPU(Addr cpuPageAddr)
{
    // Make sure that it is a valid page-aligned address
    assert( (cpuPageAddr & ((0x1 << pageSizeBits)-1)) == 0x0 );
    cpuPageAddr &= ~( (0x1 << pageSizeBits)-1 );

    // if already exists, just return directly
    // TODO: will remove the first condition
    if (gpuAddrMap.find(cpuPageAddr) != gpuAddrMap.end()
    #if FINITE_GPU_MEM_CAPACITY
     && toLoadPageAfterEvict.find(gpuAddrMap[cpuPageAddr])
            == toLoadPageAfterEvict.end()
    #endif
    ) {
        return;
    }

    // just or with gpuAddrMask to get valid gpu address
    // temporarily set
    // TODO: get correct gpuPageAddr:
    // gpuPageAddr = gpuAddrMap[cpuPageAddr]
    Addr gpuPageAddr = cpuPageAddr | gpuAddrMask;

    // make a correspondance for cpu and gpu address
    gpuAddrMap[cpuPageAddr] = gpuPageAddr;
    cpuAddrMap[gpuPageAddr] = cpuPageAddr;
    stateMap[cpuPageAddr] = Loading;

    // send a packet to memory controller
    for (ChunkGenerator gen(cpuPageAddr, pageSize, memReqUnit);
         !gen.done(); gen.next()) {
        RequestPtr req = NULL;
        Request::Flags flag = 0;
        //Tick delay = 0;

        req = std::make_shared<Request>(
                gen.addr()/*cpuPageAddr*/, gen.size()/*pageSize*/,
                flag, masterId);

        DPRINTF(GPUDataLoader, "%s gen addr %#x and gen size %u\n",
                               __func__, gen.addr(), gen.size());

        Packet::Command cmd = MemCmd::ReadReq;
        PacketPtr pkt = new Packet(req, cmd);

        uint8_t *data = new uint8_t[gen.size()];
        if (data) {
            pkt->dataStatic(data);
        }
        else {
            assert(true);
        }

        // should add sender state
        GDLSenderState *reqState = new GDLSenderState();
        reqState->targetCore = CPU;
        reqState->trans = FROM_CPU;
        pkt->senderState = reqState;

        // send to memory controller
        if (!this->cpuMemPort.sendPacket(pkt))
            blockedPackets[MOVE_CPU].push(pkt);

    }
    recvRespNum[FROM_CPU][cpuPageAddr] = 0;

    numUniqPagesLoaded++;
    tickRecord[cpuPageAddr] = curTick();
}

void
GpuDataLoader::savePageToGPU(Addr cpuPageAddr, Addr gpuPageAddr, uint8_t *data)
{
    // Make sure that it is a valid page-aligned address
    assert( (gpuPageAddr & ((0x1 << pageSizeBits)-1)) == 0x0 );
    gpuPageAddr &= ~( (0x1 << pageSizeBits)-1 );

    // send a packet to memory controller
    for (ChunkGenerator gen(gpuPageAddr, pageSize, memReqUnit);
         !gen.done(); gen.next()) {
        RequestPtr req = NULL;
        Request::Flags flag = 0;
        //Tick delay = 0;

        req = std::make_shared<Request>(
                gen.addr()/*gpuPageAddr*/, gen.size()/*pageSize*/,
                flag, masterId);

        DPRINTF(GPUDataLoader, "%s gen addr %#x and gen size %u\n",
                               __func__, gen.addr(), gen.size());

        Packet::Command cmd = MemCmd::WriteReq;
        PacketPtr pkt = new Packet(req, cmd);

        if (data) {
            auto offset = gen.addr()-gpuPageAddr;
            pkt->dataStatic(data+offset);
        }
        else {
            assert(true);
        }

        // should add sender state
        GDLSenderState *reqState = new GDLSenderState();
        reqState->targetCore = GPU;
        reqState->trans = TO_GPU;
        pkt->senderState = reqState;

        // send to memory controller
        if (!this->gpuMemPort.sendPacket(pkt))
            blockedPackets[MOVE_GPU].push(pkt);
    }
    recvRespNum[TO_GPU][gpuPageAddr] = 0;

}

void
GpuDataLoader::loadPageFromGPU(Addr gpuPageAddr)
{
    // Make sure that it is a valid page-aligned address
    assert( (gpuPageAddr & ((0x1 << pageSizeBits)-1)) == 0x0 );
    Addr cpuPageAddr = getCpuPageAddr(gpuPageAddr);

    // if already exists, just return directly
    if (gpuAddrMap.find(cpuPageAddr) != gpuAddrMap.end()) {
        assert(true);
    }

    //
    assert(stateMap[gpuPageAddr] != Loading);
    stateMap[gpuPageAddr] = Loading;

    // send a packet to memory controller
    for (ChunkGenerator gen(gpuPageAddr, pageSize, memReqUnit);
         !gen.done(); gen.next()) {
        RequestPtr req = NULL;
        Request::Flags flag = 0;
        //Tick delay = 0;

        req = std::make_shared<Request>(
                gen.addr()/*gpuPageAddr*/, gen.size()/*pageSize*/,
                flag, masterId);

        DPRINTF(GPUDataLoader, "%s gen addr %#x and gen size %u\n",
                               __func__, gen.addr(), gen.size());

        Packet::Command cmd = MemCmd::ReadReq;
        PacketPtr pkt = new Packet(req, cmd);

        uint8_t *data = new uint8_t[gen.size()];
        if (data) {
            pkt->dataStatic(data);
        }
        else {
            assert(true);
        }

        // should add sender state
        GDLSenderState *reqState = new GDLSenderState();
        reqState->targetCore = GPU;
        reqState->trans = FROM_GPU;
        pkt->senderState = reqState;

        // send to memory controller
        if (!this->gpuMemPort.sendPacket(pkt))
            blockedPackets[MOVE_GPU].push(pkt);

    }
    recvRespNum[FROM_GPU][gpuPageAddr] = 0;

    numGPUAccesses++;
    tickRecord[gpuPageAddr] = curTick();
}

void
GpuDataLoader::savePageToCPU(Addr cpuPageAddr, Addr gpuPageAddr, uint8_t *data)
{
    // Make sure that it is a valid page-aligned address
    assert( (gpuPageAddr & ((0x1 << pageSizeBits)-1)) == 0x0 );

    // send a packet to memory controller
    for (ChunkGenerator gen(cpuPageAddr, pageSize, memReqUnit);
         !gen.done(); gen.next()) {
        RequestPtr req = NULL;
        Request::Flags flag = 0;
        //Tick delay = 0;

        req = std::make_shared<Request>(
                gen.addr()/*cpuPageAddr*/, gen.size()/*pageSize*/,
                flag, masterId);

        DPRINTF(GPUDataLoader, "%s gen addr %#x and gen size %u\n",
                               __func__, gen.addr(), gen.size());

        Packet::Command cmd = MemCmd::WriteReq;
        PacketPtr pkt = new Packet(req, cmd);

        if (data) {
            auto offset = gen.addr()-cpuPageAddr;
            pkt->dataStatic(data+offset);
        }
        else {
            assert(true);
        }

        // should add sender state
        GDLSenderState *reqState = new GDLSenderState();
        reqState->targetCore = CPU;
        reqState->trans = TO_CPU;
        pkt->senderState = reqState;

        // send to memory controller
        if (!this->cpuMemPort.sendPacket(pkt))
            blockedPackets[MOVE_CPU].push(pkt);
    }
    recvRespNum[TO_CPU][cpuPageAddr] = 0;

    numCPUAccesses++;
}

void
GpuDataLoader::checkEvictDone()
{
    bool allDirtyPagesEvicted = true;
    std::vector<Addr> eraseAddrVec;
    DPRINTF(GPUDataLoader, "%s\n", __func__);
    /* gpuAddrMap: <cpuAddr, gpuAddr> */
    for (auto addrPair=gpuAddrMap.begin();
         addrPair!=gpuAddrMap.end(); addrPair++) {
        Addr gpuAddr = addrPair->second;
        if (stateMap[addrPair->second] == Done &&
            stateMap[addrPair->first] == Invalid) {
            //gpuAddrMap.erase(addrPair->first);
            eraseAddrVec.push_back(addrPair->first);
            cpuAddrMap.erase(addrPair->second);
            stateMap.erase(addrPair->first);
            stateMap.erase(addrPair->second);

            pageStatusMap.erase(addrPair->second);
        }
        /* check gpu memory status is dirty or not */
        else if ( pageStatusMap.find(gpuAddr) != pageStatusMap.end()
            && pageStatusMap[gpuAddr] == Dirty) {
            allDirtyPagesEvicted = false;
        }
        //else {
        //    if (!checkEvictEvent.scheduled())
        //        schedule(&checkEvictEvent, curTick()+clockPeriod());
        //    return;
        //}
    }

    for (auto _addr : eraseAddrVec) {
        gpuAddrMap.erase(_addr);
    }

    if (!allDirtyPagesEvicted) {
        if (!checkEvictEvent.scheduled())
            schedule(&checkEvictEvent, curTick()+clockPeriod());
        return;
    }

    evictingAll = false;
    evictAllDone = true;
}

void
GpuDataLoader::evictPage(Addr gpuPageAddr)
{
    /* will load target page from gpu and
     * move back to cpu */
    DPRINTF(GPUDataLoader, "%s for addr %#x\n", gpuPageAddr);
    loadPageFromGPU(gpuPageAddr);
    numEvictedPages++;
}

void
GpuDataLoader::evictPage(Addr gpuPageAddr, Addr nextLoadedPageAddr)
{
    DPRINTF(GPUDataLoader, "%s for addr %#x\n", gpuPageAddr);
    toLoadPageAfterEvict[gpuPageAddr] = nextLoadedPageAddr;
    loadPageFromGPU(gpuPageAddr);
    numEvictedPages++;
}

void
GpuDataLoader::evictAllPages()
{
    DPRINTF(GPUDataLoader, "%s\n", __func__);

    evictingAll = true;
    evictAllDone = false;
    for (auto addrPair : gpuAddrMap) {
        if (stateMap[addrPair.first] == Done) {
            evictPage(addrPair.second);
        }
    }

    schedule(&checkEvictEvent, curTick()+clockPeriod());
}

void
GpuDataLoader::setPageStatus(Addr pageAddr, PageStatus _status)
{
    DPRINTF(GPUDataLoader, "%s for addr %#x as %u\n",
                           __func__, pageAddr, _status);
    pageAddr &= ~((0x1 << pageSizeBits)-1);
    pageStatusMap[pageAddr] = _status;

    DPRINTF(GPUDataLoader, "%s for addr %#x as %u\n",
                           __func__, pageAddr, _status);
}

void
GpuDataLoader::setAbstractMem(std::string name,
                              uint8_t* pmem,
                              AddrRange range)
{
    uint8_t type;
    if (name == "system.mem_ctrls1") {
        type = MemCtrls1;
    }
    else if (name == "system.mem_ctrls0") {
        type = MemCtrls0;
    }
    else if (name == "system.ruby.phys_mem") {
        type = RubyPhysMem;
    }
    abstractMem[type] = pmem;
    abstractMemRange[type] = range;
    isSetAbstractMem = true;
}

void
GpuDataLoader::makeDataCoherent(Addr addr, uint64_t size)
{
    DPRINTF(GPUDataLoader, "%s making data synchronized: addr %#x, size %u\n",
                           __func__, addr, size);

    if (abstractMemRange[MemCtrls1].contains(addr)) {
        uint8_t* dst = abstractMem[RubyPhysMem] + addr
                     - abstractMemRange[RubyPhysMem].start();
        uint8_t* src = abstractMem[MemCtrls1] + addr
                     - abstractMemRange[MemCtrls1].start();
        memcpy(dst, src, size);
    }
    else if (abstractMemRange[MemCtrls0].contains(addr)) {
        uint8_t* dst = abstractMem[RubyPhysMem] + addr
                     - abstractMemRange[RubyPhysMem].start();
        uint8_t* src = abstractMem[MemCtrls0] + addr
                     - abstractMemRange[MemCtrls0].start();
        memcpy(dst, src, size);
    }

}

void
GpuDataLoader::syncToMainMem(Addr addr, const uint8_t* data, uint64_t size)
{
    DPRINTF(GPUDataLoader, "%s: addr %#x, size %u\n", __func__, addr, size);

    if (abstractMemRange[MemCtrls1].contains(addr)) {
        uint8_t* dst = abstractMem[MemCtrls1] + addr
                     - abstractMemRange[MemCtrls1].start();
        memcpy(dst, data, size);
    }
    else if (abstractMemRange[MemCtrls0].contains(addr)) {
        uint8_t* dst = abstractMem[MemCtrls0] + addr
                     - abstractMemRange[MemCtrls0].start();
        memcpy(dst, data, size);
    }
}


bool
GpuDataLoader::CoreSidePort::sendPacket(PacketPtr pkt)
{
    assert(pkt != nullptr);
    if (if_sent_failed) {
        return false;
    }
    DPRINTF(GPUDataLoader,
            "%s gen addr %#x and gen size %u with Read/Write %u\n",
            __func__, pkt->getAddr(), pkt->getSize(), pkt->isWrite());
    DUMPDATA(GPUDataLoader, pkt->getPtr<uint8_t>());

    if (!if_sent_failed && !sendTimingReq(pkt)) {
        if_sent_failed = true;
        DPRINTF(GPUDataLoader, "%s sendTimingReq for addr %#x failed\n",
                             __func__, pkt->getAddr());
        return false;
    }
    else {
        DPRINTF(GPUDataLoader, "%s sendTimingReq for addr %#x succeed\n",
                             __func__, pkt->getAddr());
        return true;
    }
}

bool
GpuDataLoader::CoreSidePort::recvTimingResp(PacketPtr pkt)
{
    return owner->handleResponse(pkt, portName);
}

void
GpuDataLoader::CoreSidePort::recvReqRetry()
{
    assert(owner->blockedPackets[MOVE_CPU].size() > 0 ||
           owner->blockedPackets[MOVE_GPU].size() > 0);

    DPRINTF(GPUDataLoader, "%s\n", __func__);

    /* Figure out what the port is */
    LoaderAct act;
    if (portName.find("cpu_side_mem_port") != std::string::npos) {
        act = GpuDataLoader::MOVE_CPU;
    }
    else {
        act = GpuDataLoader::MOVE_GPU;
    }

    /* reset the flag */
    if_sent_failed = false;

    //std::queue<PacketPtr> copyBlockedPackets(owner->blockedPackets);
    //for ( int i = 0; i < owner->blockedPackets.size(); i++ )
    //    owner->blockedPackets.pop();

    PacketPtr pkt;

    uint32_t blockedPktSize = owner->blockedPackets[act].size();
    for ( int i = 0; i < blockedPktSize; i++ ) {
        pkt = owner->blockedPackets[act].front();
        if (this->sendPacket(pkt)) {
            owner->blockedPackets[act].pop();
        }
        else {
            return;
        }
    }

}

void
GpuDataLoader::CoreSidePort::recvRangeChange()
{
    // do nothing
    return;
}


void
GpuDataLoader::regStats()
{
    //using namespace Stats;
    MemObject::regStats();

    numGPUAccesses
        .name(name() + ".numGPUAccesses")
        .desc("Number of GPU access to memory");
    numCPUAccesses
        .name(name() + ".numCPUAccesses")
        .desc("Number of CPU access to memory");
    numPageMisses
        .name(name() + ".numPageMisses")
        .desc("Number of missing on GPU access to memory");
    numPageHits
        .name(name() + ".numPageHits")
        .desc("Number of hits on GPU access to memory");
    numEvictedPages
        .name(name() + ".numEvictedPages")
        .desc("Number of evited pages from GPU to CPU");
    numUniqPagesLoaded
        .name(name() + ".numUniqPagesLoaded")
        .desc("Number of unique pages loaded from CPU");

    sumPageLoadLatency
        .name(name() + ".sumPageLoadLatency")
        .desc("Sum of latency loading pages");
    avgPageLoadLatency
        .name(name() + ".avgPageLoadLatency")
        .desc("Aveage latency of loading a page")
        .precision(2);
    sumPageEvictLatency
        .name(name() + ".sumPageEvictLatency")
        .desc("Sum of latency evicting pages");
    avgPageEvictLatency
        .name(name() + ".avgPageEvictLatency")
        .desc("Average latency of evict a page")
        .precision(2);

    //avgPageLoadLatency = sumPageLoadLatency / numEvictedPages;
    //avgPageEvictLatency = sumPageEvictLatency / numPageMisses;

    avgMemAccessLatency
        .name(name() + ".avgMemAccessLatency")
        .desc("Average latency of gpu memory access")
        .precision(2);
}


void
GpuDataLoader::handleMissingPage(Addr cpuPageAddr)
{
    bool isFull = true;
    Addr targetGPUPageAddr;
    if (gpuAddrMap.find(cpuPageAddr) != gpuAddrMap.end()) {
        return;
    }
    for (int i = 0; i < secondChanceStatus.size(); i++) {
        if (secondChanceStatus[i] == SC_INVALID) {
            targetGPUPageAddr = baseAddr + pageSize*i;
            /* set corresponding GPU page addr */
            gpuAddrMap[cpuPageAddr] = targetGPUPageAddr;
            cpuAddrMap[targetGPUPageAddr] = cpuPageAddr;
            stateMap[cpuPageAddr] = Loading;
            /* set the second chance status */
            secondChanceVec[i] = cpuPageAddr;
            secondChanceStatus[i] = SC_TRUE;
            isFull = false;
            break;
        }
    }
    while (isFull) {
        if (secondChanceInd == secondChanceVec.size()) {
            secondChanceInd = 0;
        }
        auto currInd = secondChanceInd;
        secondChanceInd++;
        if (secondChanceStatus[currInd] == SC_TRUE) {
            secondChanceStatus[currInd] = SC_FALSE;
        }
        else if (secondChanceStatus[currInd] == SC_FALSE) {
            targetGPUPageAddr = secondChanceVec[currInd];
            /* should evict the page */
            Addr evictedPageAddr = getGpuPageAddr(targetGPUPageAddr);
            evictPage(evictedPageAddr, cpuPageAddr);

            /* set corresponding GPU page addr */
            gpuAddrMap[cpuPageAddr] = targetGPUPageAddr;
            cpuAddrMap[targetGPUPageAddr] = cpuPageAddr;
            stateMap[cpuPageAddr] = Loading;

            /* set second chance status to represent the current page
             * will be replaced with the new loaded page */
            secondChanceVec[currInd] = cpuPageAddr;
            secondChanceStatus[currInd] = SC_TRUE;

            /* return directly,  after evicting target page,
             * next it will call loadPageFromCPU to load the page
             * */
            return;
        }
    }

    loadPageFromCPU(cpuPageAddr);
}




