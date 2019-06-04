//#include <ctypes>
#include "scga_dma/scga_dma.hh"

#include <stdint.h>

#include "cpu/base.hh"
#include "debug/ScGaDMA.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

ScGaDma* ScGaDma::scgaDmaInstance = NULL;

ScGaDma::ScGaDma(ScGaDmaParams *p) :
    DmaDevice(p),
    gpu_side_mem_port(this, sys),
    writeThrPort(this, sys),
    scgaDmaReadEvent([this]{ scgaDmaReadDone(); }, name()),
    scgaDmaWriteEvent([this]{ scgaDmaWriteDone(); }, name()),
    writeThrEvent([this]{ writeThrDone(); }, name()),
    cpu(p->cpu),
    pioAddr(p->pio_addr),
    pioSize(4096),
    pioDelay(p->pio_latency),
    tickEvent([this]{ exec(); }, "SCGA-DMA tick", false, Event::CPU_Tick_Pri)
{
    //tlbPort = new TLBPort(csprintf("%s-tlb-port", name()), this);
    dmaReadCPUData = new uint8_t[64];
    dmaWriteGPUData = new uint8_t[64];
    DPRINTF(ScGaDMA, "%s instantiated.\n", name().c_str());
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
                    DPRINTF(ScGaDMA, "Should not happen page fault?\n");
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
ScGaDma::scgaDmaReadDone()
{
    DPRINTF(ScGaDMA, "ScGaDma::exec, dmaRead: %x %x %x %x %x %x %x %x\n",
            dmaReadCPUData[0], dmaReadCPUData[1],
            dmaReadCPUData[2], dmaReadCPUData[3],
            dmaReadCPUData[4], dmaReadCPUData[5],
            dmaReadCPUData[6], dmaReadCPUData[7]);
    DPRINTF(ScGaDMA, "scgaDmaReadDone......\n");
}

void ScGaDma::scgaDmaWriteDone()
{
    DPRINTF(ScGaDMA, "scgaDmaWriteDone......\n");
}

void
ScGaDma::writeThrDone()
{
   DPRINTF(ScGaDMA, "scgaDma_writeThrDone\n");
}

void
ScGaDma::exec()
{
    /* Start reading from main CPU memory */

    DPRINTF(ScGaDMA, "ScGaDma::exec start\n");

    for (auto cpu_req : cpu_requests) {
        cpu_start_addr = cpu_req.first;
        cpu_transfer_size = cpu_req.second;
        this->dmaRead(cpu_start_addr, cpu_transfer_size,
                      &scgaDmaReadEvent, dmaReadCPUData, 0/*delay*/);
    }

    cpu_requests.clear();
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
    std::cout<<"need this later:::"<<offset<<std::endl;
    pkt->allocate();

    DPRINTF(ScGaDMA, "SCGA-DMA read register %#x size=%d\n",
                     offset, pkt->getSize());

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
     * Identify which actions should be taken
     * */
    if (offset) {
        assert(arg_trans_ready);
    }
    switch(offset)
    {
        case SCGA_DMA_OFFSET_CONFIG:
            arg_trans_ready = true;
            break;
        case SCGA_DMA_OFFSET_START_ADDR:
            cpu_start_addr = data_val;
            break;
        case SCGA_DMA_OFFSET_TRANS_SIZE:
            cpu_transfer_size = data_val;
            cpu_requests.push_back(std::make_pair(cpu_start_addr,
                                                  cpu_transfer_size));
            break;
        case SCGA_DMA_OFFSET_ACT:
            assert(!dma_start_act);
            if (data_val) {
                DPRINTF(ScGaDMA, "ScGaDma::write, activate\n");
                dma_start_act = true;
                schedule(&tickEvent, curTick()+pioDelay);
            }
            break;
        default:
            panic("SCGA_DMA: Unrecognized offset!\n");
            break;
    }

    pkt->makeTimingResponse();

    return pioDelay;
}

bool
ScGaDma::getDataFromCPU(Addr addr, int size)
{
    this->dmaRead(addr, size,
                  &scgaDmaReadEvent,
                  dmaReadCPUData, 0/*delay*/);
    return true;
}

bool ScGaDma::sendDataToGPU(Addr addr, int size)
{
    this->dmaWrite(addr, size,
                   &scgaDmaWriteEvent,
                   dmaWriteGPUData, 0);
    return true;
}

void ScGaDma::writeThrOp(Addr addr, int size,
                         uint8_t *data, CacheType cache_type)
{
    writeThrPort.dmaAction(MemCmd::WriteReq, addr, size,
                           &writeThrEvent, data, 0/*delay*/);
}


