//#include <ctypes>
#include "scga_dma/scga_dma.hh"

#include <stdint.h>

#include "cpu/base.hh"
#include "debug/ScGaDMA.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

#define pageSize (4096)
#define blockSize (128)

static MM2S_DMACR_REG mm2sDmaCR;
static MM2S_DMASR_REG mm2sDmaSR;
static MM2S_CURDESC_REG mm2sCurrDesc;
static MM2S_CURDESC_MSB_REG mm2sCurrDescMSB;
static MM2S_TAILDESC_REG mm2sTailDesc;
static MM2S_TAILDESC_MSB_REG mm2sTailDescMSB;

static S2MM_DMACR_REG s2mmDmaCR;
static S2MM_DMASR_REG s2mmDmaSR;
static S2MM_CURDESC_REG s2mmCurrDesc;
static S2MM_CURDESC_MSB_REG s2mmCurrDescMSB;
static S2MM_TAILDESC_REG s2mmTailDesc;
static S2MM_TAILDESC_MSB_REG s2mmTailDescMSB;

ScGaDma* ScGaDma::scgaDmaInstance = NULL;

ScGaDma::ScGaDma(ScGaDmaParams *p) :
    DmaDevice(p),
    gpu_side_mem_port(this, sys),
    writeThrPort(this, sys),
    //scgaDmaReadEvent([this]{ scgaDmaReadDone(); }, name()),
    //scgaDmaWriteEvent([this]{ scgaDmaWriteDone(); }, name()),
    writeThrEvent([this]{ writeThrDone(); }, name()),
    cpu(p->cpu),
    pioAddr(p->pio_addr),
    pioSize(4096),
    pioDelay(p->pio_latency),
    tickEvent([this]{ exec(); }, "SCGA-DMA tick", false, Event::CPU_Tick_Pri)
{
    //tlbPort = new TLBPort(csprintf("%s-tlb-port", name()), this);
    dmaReadCPUData = new uint8_t[pageSize];
    dmaWriteGPUData = new uint8_t[64];
    DPRINTF(ScGaDMA, "%s instantiated.\n", name().c_str());
    //schedule(&tickEvent, curTick()+clockPeriod());

    dram_mem = NULL;
    dmaState = Idle;
    memset(&mm2sDmaCR, 0, sizeof(MM2S_DMACR_REG));
    mm2sDmaSR.Halted = 1;

    gatherDone = false;
    storeDone = false;

    isUnRemapStart = false;
    isBlocksBackStart = false;
    isUnRemapLoadDone = false;
    isUnRemapStoreDone = false;
}

ScGaDma*
ScGaDmaParams::create()
{
    ScGaDma* new_instance = new ScGaDma(this);
    ScGaDma::setInstance(new_instance);
    return new_instance;
}

//template <typename T>
void ScGaDma::setAddrTransRule(ContextID _id, SimpleThread* _thread)
{
    /* Now T is SimpleThread */
    if (addr_trans_map.find(_id) != addr_trans_map.end())
        return;
    addr_trans_map[_id] = _thread;
}

// For testing
void
ScGaDma::AddrTransTest(Addr addr, Request::Flags flags, MasterID mid)
{
    SimpleThread *thread = (addr_trans_map.begin())->second;
    BaseTLB::Mode mode = BaseTLB::Read;
    RequestPtr req = std::make_shared<Request>(
        0/*asid*/, addr, 8/*size*/, flags, mid, 0, thread->contextId());

    WholeTranslationState *state =
        new WholeTranslationState(req, NULL/*data*/, NULL/*res*/, mode);
    DataTranslation<TimingSimpleCPU*> *translation =
        new DataTranslation<TimingSimpleCPU*>(cpu, state);
    //thread->dtb->translateTiming(req, thread->getTC(), translation, mode);
    (void)(translation);

    Addr Paddr = getPhyAddr(addr, thread);
    (void)(Paddr);

    DPRINTF(ScGaDMA, "AddrTrans: Vaddr %x -> Paddr %x\n", addr, Paddr);
}

/* This translation function is only for x86 */
Addr
ScGaDma::getPhyAddr(Addr vaddr, SimpleThread* thread)
{
    ThreadContext *tc = thread->getTC();
    X86ISA::HandyM5Reg m5Reg = tc->readMiscRegNoEffect(X86ISA::MISCREG_M5_REG);
    X86ISA::TLB *tlb = dynamic_cast<X86ISA::TLB*>(thread->dtb);

    if (m5Reg.prot) {
        if (m5Reg.paging) {
            X86ISA::TlbEntry *entry = tlb->lookup(vaddr);
            if (!entry) {
                Process *p = tc->getProcessPtr();
                const EmulationPageTable::Entry *pte =
                    p->pTable->lookup(vaddr);
                if (!pte /*&& mode != BaseTLB::Execute*/) {
                    if (p->fixupStackFault(vaddr)) {
                        pte = p->pTable->lookup(vaddr);
                    }
                }
                if (pte) {
                    DPRINTF(ScGaDMA, "Entry found with paddr %#x, "
                            "doing protection checks.\n", pte->paddr);
                    return pte->paddr;
                }
                else {
                    DPRINTF(ScGaDMA, "Should not happen page fault? for %#x\n",
                                     vaddr);
                    assert(false);
                }
            }
            else {
                return (entry->paddr | (vaddr & mask(entry->logBytes)));
            }
        }
    }

    return 0;
}


void
ScGaDma::scgaDmaReadDone(Addr _addr)
{
    currLoadingBlocksSet.erase(_addr);
    DPRINTF(ScGaDMA, "scgaDmaReadDone: %#x\n", _addr);
    DDUMP(ScGaDMA, dmaReadCPUData, blockSize*3);
    if (currLoadingBlocksSet.empty()) {
        gatherDone = true;
    }
}

void ScGaDma::scgaDmaWriteDone(Addr _addr)
{
    DPRINTF(ScGaDMA, "scgaDmaWriteDone: %#x\n", _addr);
    storeDone = true;
}

void
ScGaDma::writeThrDone()
{
   DPRINTF(ScGaDMA, "scgaDma_writeThrDone\n");
}

void
ScGaDma::unRemapLoadDone(Addr _addr)
{
    unRemapLoadPagesSet.erase(_addr);
    DPRINTF(ScGaDMA, "%s: %#x\n", __func__, _addr);
    DDUMP(ScGaDMA, loadedPagesDataMap[_addr], blockSize*3);
    if (unRemapLoadPagesSet.empty()) {
        isUnRemapLoadDone = true;
        DPRINTF(ScGaDMA, "unRemap-Load all done!\n");
    }
}

void
ScGaDma::unRemapStoreDone(Addr _addr)
{
    DPRINTF(ScGaDMA, "%s: %#x\n", __func__, _addr);
    //DDUMP(ScGaDMA, dmaReadCPUData, blockSize*3);
    unRemapStoreBlocksSet.erase(_addr);
    if (unRemapStoreBlocksSet.empty()) {
        isUnRemapStoreDone = true;
        DPRINTF(ScGaDMA, "unRemap-Store all done!\n");
    }
}


void
ScGaDma::exec()
{
    /* Start reading from main CPU memory */
    SimpleThread *thread = (addr_trans_map.begin())->second;

    DPRINTF(ScGaDMA, "ScGaDma::exec start with state %u\n",
                     dmaState);

    //for (auto cpu_req : cpu_requests) {
    //    cpu_start_addr = cpu_req.first;
    //    cpu_transfer_size = cpu_req.second;
    //    this->dmaRead(cpu_start_addr, cpu_transfer_size,
    //                  &scgaDmaReadEvent, dmaReadCPUData, 0/*delay*/);
    //}
    //cpu_requests.clear();

    if (!mm2sDmaSR.Halted) {
        // start dma data moving
        //while (true) {
        switch (dmaState) {
            case Loading:
            {
                Addr currDescVAddr = (uint64_t)(mm2sCurrDesc.CurrDescriptor) |
                (uint64_t)((uint64_t)mm2sCurrDescMSB.CurrDescriptorMSB << 32);
                Addr tailDescVAddr = (uint64_t)(mm2sTailDesc.TailDescriptor) |
                (uint64_t)((uint64_t)mm2sTailDescMSB.TailDescriptorMSB << 32);
                (void)(tailDescVAddr);


                // Get the physical address
                Addr currDescPAddr = getPhyAddr(currDescVAddr, thread);

                // Send to DRAM to get the whole information of the struct
                // and copy the content to the SCGA_DESCRIPTOR struct
                SCGA_DESCRIPTOR currDesc;

                /* Get the information of descriptor */
                assert(dram_mem != NULL);
                getMemContent(currDescPAddr,
                              sizeof(SCGA_DESCRIPTOR),
                              (void*)&currDesc);

                Addr buffVAddr = (uint64_t)(currDesc.BUFFER_ADDR) |
                    (uint64_t)((uint64_t)currDesc.BUFFER_ADDR_MSB << 32);
                int buffSize = currDesc._ctrl.BUFFER_LENGTH;

                assert(buffSize == blockSize);

                DPRINTF(ScGaDMA, "%s dmaState: Loading, "
                                 "Desc %#x, VAddr: %#x\n",
                                 __func__, currDescVAddr, buffVAddr);

                // Transform to physical address
                Addr buffPAddr = getPhyAddr(buffVAddr, thread);
                getData(buffPAddr, buffSize,
                        &dmaReadCPUData[(blockCnt++)*blockSize]);

                // for remapPages afterwards
                currLoadingBlocks.push_back({buffPAddr,buffSize});

                // for handling response from loading
                currLoadingBlocksSet.insert(buffPAddr);

                if (currDesc._ctrl._EOF) {
                    //mm2sDmaCR.Halted = 1;
                    dmaState = Storing;
                    //dmaState = Idle;
                }
                else {
                    mm2sCurrDesc.CurrDescriptor = currDesc.NXTDESC;
                    mm2sCurrDescMSB.CurrDescriptorMSB = currDesc.NXTDESC_MSB;
                }
            }
                break;
            case Storing:
            {
                if (gatherDone) {
                    gatherDone = false;
                    Addr currDescVAddr
                     = (uint64_t)(s2mmCurrDesc.CurrDescriptor) |
                       (uint64_t)((uint64_t)s2mmCurrDescMSB.CurrDescriptorMSB
                           << 32);
                    // Get the physical address
                    Addr currDescPAddr = getPhyAddr(currDescVAddr, thread);

                    // Send to DRAM to get the whole information of the struct
                    // and copy the content to the SCGA_DESCRIPTOR struct
                    SCGA_DESCRIPTOR currDesc;

                    /* Get the information of descriptor */
                    assert(dram_mem != NULL);
                    getMemContent(currDescPAddr,
                                  sizeof(SCGA_DESCRIPTOR),
                                  (void*)&currDesc);

                    Addr buffVAddr = (uint64_t)(currDesc.BUFFER_ADDR) |
                    (uint64_t)((uint64_t)currDesc.BUFFER_ADDR_MSB << 32);
                    int buffSize = currDesc._ctrl.BUFFER_LENGTH;
                    // Translate to physical address
                    Addr buffPAddr = getPhyAddr(buffVAddr, thread);
                    sendData(buffPAddr, buffSize, dmaReadCPUData);

                    DPRINTF(ScGaDMA, "%s dmaState: Storing, "
                                     "Desc: %#x, VAddr: %#x\n",
                                     __func__, currDescVAddr, buffVAddr);

                    if (currDesc._ctrl._EOF) {
                        RemapPage rp(buffPAddr, currLoadingBlocks);
                        remapPages[buffPAddr] = rp;
                        //dmaState = Idle;
                    }
                    else {
                        s2mmCurrDesc.CurrDescriptor = currDesc.NXTDESC;
                        s2mmCurrDescMSB.CurrDescriptorMSB
                            = currDesc.NXTDESC_MSB;
                    }
                    dmaState = Completion;
                }
                else {
                    DPRINTF(ScGaDMA, "dmaState: Loading, Still gathering\n");
                }
            }
                break;
            case Completion:
                if (storeDone) {
                    storeDone = false;
                    DPRINTF(ScGaDMA, "Store one remapped page completed\n");
                    currLoadingBlocks.clear();
                    dmaState = Idle;
                }
                break;
            case Idle:
                mm2sDmaSR.Halted = 1;
                break;
            /* UnRemap */
            case UnRemap_Load:
                if (isUnRemapStart) {
                    rpIter++;
                    if (rpIter == remapPages.end()) {
                        dmaState = UnRemap_Store;
                        isUnRemapStart = false;
                        break;
                    }
                }
                else {
                    rpIter = remapPages.begin();
                    isUnRemapStart = true;
                }

                // load remapped pages from DRAM
                {
                    uint8_t* _data = new uint8_t[pageSize];
                    loadedPagesDataMap[rpIter->first] = _data;
                    getData(rpIter->first, pageSize, _data);
                }
                // Make a record
                unRemapLoadPagesSet.insert(rpIter->first);

                break;
            case UnRemap_Store:
                if (isUnRemapLoadDone) {
                    if (isBlocksBackStart) {
                        rpIter++;
                        if (rpIter == remapPages.end()) {
                            dmaState = UnRemap_Complt;
                            isUnRemapLoadDone = false;
                            isBlocksBackStart = false;
                            break;
                        }
                    }
                    else {
                        rpIter = remapPages.begin();
                        isBlocksBackStart = true;
                    }
                    // Iterate for each constitued block of the page
                    DPRINTF(ScGaDMA, "constituted blocks size %u\n",
                            (rpIter->second).constitutedBlocks.size());
                    int accu_bytes = 0;
                    for (auto cb_pair
                            : (rpIter->second).constitutedBlocks ) {
                        uint8_t* _data = loadedPagesDataMap[rpIter->first];
                        DPRINTF(ScGaDMA, "constituted blocks addr %x, "
                                         "size %u\n",
                                cb_pair.first, cb_pair.second);
                        DDUMP(ScGaDMA, &(_data[accu_bytes]), cb_pair.second);
                        sendData(cb_pair.first, cb_pair.second,
                                 &(_data[accu_bytes]));
                        unRemapStoreBlocksSet.insert(cb_pair.first);
                        accu_bytes += cb_pair.second;
                    }
                }
                break;
            case UnRemap_Complt:
                if (isUnRemapStoreDone) {
                    dmaState = Idle;
                    isUnRemapStoreDone = false;
                    unRemapClear();
                    remapPages.clear();
                }
                break;
            default:
                break;
        }
        if (!tickEvent.scheduled()) {
            schedule(&tickEvent, curTick()+clockPeriod());
        }
        DPRINTF(ScGaDMA, "Next State: %u, Schedule ScGaDma Regular Tick %u\n",
                         dmaState,
                         curTick()+clockPeriod());
    }
    else {
        // something wrong, shouldn't be here
        assert(true);
    }

}

BaseMasterPort&
ScGaDma::getMasterPort(const std::string& if_name,
                       PortID idx)
{
    if (if_name == "cpu_side_mem_port") {
        //return cpu_side_mem_port;
        return dmaPort;
    }
    else if (if_name == "gpu_side_mem_port") {
        return gpu_side_mem_port;
    }
    //else if (if_name == "translation_port") {
    //    return *tlbPort;
    //}
    else if (if_name == "writeThrPort") {
        return writeThrPort;
    }
    else {
        return DmaDevice::getMasterPort(if_name, idx);
    }
}

bool
ScGaDma::handleResponse(PacketPtr pkt)
{
    return true;
}

AddrRangeList
ScGaDma::getAddrRanges() const
{
    AddrRangeList ranges;

    DPRINTF(ScGaDMA, "ScGaDma registering addr range at %#x size %#x\n",
            pioAddr, pioSize);

    ranges.push_back(RangeSize(pioAddr, pioSize));

    return ranges;

}

Tick
ScGaDma::read(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr);
    assert(pkt->getAddr() < pioAddr + pioSize);

    int offset = pkt->getAddr() - pioAddr;
    //std::cout<<"need this later:::"<<offset<<std::endl;
    //pkt->allocate();

    DPRINTF(ScGaDMA, "SCGA-DMA read register %#x size=%d\n",
                     offset, pkt->getSize());

    switch (offset) {
        case DMA_NOOP:
        {
            uint64_t isOp = (dmaState == Idle) ? 1 : 0;
            DPRINTF(ScGaDMA, "Idle? %u\n", isOp);
            //pkt->setLE(isOp);
            pkt->setRaw(isOp);
            //uint8_t* _ptr = pkt->getPtr<uint8_t>();
            //memset((void*)_ptr, 0, sizeof(uint64_t));
            //memcpy((void*)_ptr, &isOp, pkt->getSize());
            break;
        }
        default:
            panic("%s unrecognized offset %u\n", __func__, offset);
            break;
    }

    /*
     * Read status of ScGaDma
     * */
    pkt->makeTimingResponse();

    return pioDelay;
}

Tick
ScGaDma::write(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr);
    assert(pkt->getAddr() < pioAddr + pioSize);

    int offset = pkt->getAddr() - pioAddr;

    DPRINTF(ScGaDMA, "ScGaDma write cmd: %s\n",
                          pkt->cmdString().c_str());
    DPRINTF(ScGaDMA, "ScGaDma write cmd: %#x\n",
                          pkt->getAddr());

    uint64_t data_val = 0;
    switch (pkt->getSize()) {
        case 1:
            data_val = pkt->getLE<uint8_t>();
            break;
        case 2:
            data_val = pkt->getLE<uint16_t>();
            break;
        case 4:
            data_val = pkt->getLE<uint32_t>();
            break;
        case 8:
            data_val = pkt->getLE<uint64_t>();
            break;
        default:
            DPRINTF(ScGaDMA, "bad size %d\n", pkt->getSize());
    }
    DPRINTF(ScGaDMA, "ScGaDMA write register %#x value %#x size=%d\n",
                             offset, data_val, pkt->getSize());

    /*
     * Store the data value to the registers
     */
    //if (offset) {
    //    assert(arg_trans_ready);
    //}
    switch(offset)
    {
        case MM2S_DMACR:
            memcpy(&mm2sDmaCR, &data_val, sizeof(mm2sDmaCR));
            if (mm2sDmaCR.RS) {
                // deassert Halted means actions starts
                mm2sDmaSR.Halted = 0;
                // change the state
                dmaState = Loading;
                // clear previous recorded blocks,
                // which should be copied to remapPages
                //currLoadingBlocks.clear();
                blockCnt = 0;
                schedule(&tickEvent, curTick()+pioDelay);
            }
            break;

        case MM2S_DMASR:
            memcpy(&mm2sDmaSR, &data_val, sizeof(mm2sDmaSR));
            break;

        case MM2S_CURDESC:
            memcpy(&mm2sCurrDesc, &data_val, sizeof(mm2sCurrDesc));
            break;

        case MM2S_CURDESC_MSB:
            memcpy(&mm2sCurrDescMSB, &data_val, sizeof(mm2sCurrDescMSB));
            //schedule(&tickEvent, curTick()+pioDelay);
            break;

        case MM2S_TAILDESC:
            memcpy(&mm2sTailDesc, &data_val, sizeof(mm2sTailDesc));
            break;

        case MM2S_TAILDESC_MSB:
            memcpy(&mm2sTailDesc, &data_val, sizeof(mm2sTailDescMSB));
            break;

        case SG_CTL:
            break;

        case S2MM_DMACR:
            memcpy(&s2mmDmaCR, &data_val, sizeof(s2mmDmaCR));
            break;

        case S2MM_DMASR:
            memcpy(&s2mmDmaSR, &data_val, sizeof(s2mmDmaSR));
            break;

        case S2MM_CURDESC:
            memcpy(&s2mmCurrDesc, &data_val, sizeof(s2mmCurrDesc));
            break;

        case S2MM_CURDESC_MSB:
            memcpy(&s2mmCurrDescMSB, &data_val, sizeof(s2mmCurrDescMSB));
            break;

        case S2MM_TAILDESC:
            memcpy(&s2mmTailDesc, &data_val, sizeof(s2mmTailDesc));
            break;

        case S2MM_TAILDESC_MSB:
            memcpy(&s2mmTailDescMSB, &data_val, sizeof(s2mmTailDescMSB));
            break;

        case UNREMAP:
            mm2sDmaSR.Halted = 0;
            // change the state
            dmaState = UnRemap_Load;
            schedule(&tickEvent, curTick()+pioDelay);
            break;

        default:
            panic("SCGA_DMA: Unrecognized offset!\n");
            break;
    }

    pkt->makeTimingResponse();

    return pioDelay;
}

bool
ScGaDma::getData(Addr addr, int size, uint8_t* loaded_data)
{
    EventFunctionWrapper *_event;
    if (dmaState == Loading) {
        _event = createGatherEvent(addr);
    }
    else {
        _event = createUnRemapLoadEvent(addr);
    }
    this->dmaRead(addr, size,
                  _event,
                  loaded_data, 0/*delay*/);
    return true;
}

bool
ScGaDma::sendData(Addr addr, int size, uint8_t* _data)
{
    EventFunctionWrapper *_event;
    if (dmaState == Storing) {
        _event = createCompltEvent(addr);
    }
    else {
        _event = createUnRemapStoreEvent(addr);
    }
    this->dmaWrite(addr, size,
                   _event,
                   _data, 0);
    return true;
}

void
ScGaDma::writeThrOp(Addr addr, int size,
                    uint8_t *data, CacheType cache_type)
{
    uint8_t* copy_data = new uint8_t[size];
    memcpy(copy_data, data, size);
    EventFunctionWrapper *event = createWriteThrEvent(addr, copy_data);
    writeThrPort.dmaAction(MemCmd::WriteReq, addr, size,
                           event/*&writeThrEvent*/, copy_data, 0/*delay*/);
}

EventFunctionWrapper*
ScGaDma::createWriteThrEvent(Addr addr, uint8_t* copy_data)
{
    return new EventFunctionWrapper(
        [this, addr, copy_data]{ processGpuWriteThrDone(addr, copy_data); },
        "Write Through Done Event", true
    );
}

void
ScGaDma::processGpuWriteThrDone(Addr addr, uint8_t* copy_data)
{
    DPRINTF(ScGaDMA, "%s for %#x\n", __func__, addr);
    delete copy_data;
}

uint8_t*
ScGaDma::getMemContent(Addr _addr, uint32_t _size, void* _dst)
{
    uint8_t* _src = dram_mem + _addr - dram_mem_range.start();
    memcpy(_dst, _src, _size);

    return _src;
}

EventFunctionWrapper*
ScGaDma::createGatherEvent(Addr _paddr)
{
    return new EventFunctionWrapper(
        [this, _paddr]{ scgaDmaReadDone(_paddr); },
        "Gathering completion for one block event",
        true
    );
}

EventFunctionWrapper*
ScGaDma::createCompltEvent(Addr _paddr)
{
    return new EventFunctionWrapper(
        [this, _paddr]{ scgaDmaWriteDone(_paddr); },
        "Storing completion for one remapped page",
        true
    );
}

EventFunctionWrapper*
ScGaDma::createUnRemapLoadEvent(Addr _paddr)
{
    return new EventFunctionWrapper(
        [this, _paddr]{ unRemapLoadDone(_paddr); },
        "Loading the remapped pages done",
        true
    );
}

EventFunctionWrapper*
ScGaDma::createUnRemapStoreEvent(Addr _paddr)
{
    return new EventFunctionWrapper(
        [this, _paddr]{ unRemapStoreDone(_paddr); },
        "Un-remap pages to all original blocks done",
        true
    );
}



void
ScGaDma::unRemapClear()
{
    for (auto it : loadedPagesDataMap) {
        free(it.second);
    }
    loadedPagesDataMap.clear();
    assert(unRemapLoadPagesSet.empty());
    assert(unRemapStoreBlocksSet.empty());
}
