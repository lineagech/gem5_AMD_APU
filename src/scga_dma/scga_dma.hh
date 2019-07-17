#ifndef __SCGA_DMA__
#define __SCGA_DMA__

#include <unordered_map>
#include <unordered_set>
#include <vector>

//For translation
#include "arch/x86/pagetable.hh"
#include "arch/x86/regs/misc.hh"
#include "arch/x86/tlb.hh"
#include "cpu/simple/timing.hh"
#include "cpu/simple_thread.hh"
#include "dev/dma_device.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "params/ScGaDma.hh"
#include "scga_dma/regs.hh"

#define SCGA_DMA_VIRTUAL_START           (0x20000000)
#define SCGA_DMA_PHYSICAL_START          (0x300000000)

#define SCGA_DMA_OFFSET_CONFIG           (0)
#define SCGA_DMA_OFFSET_START_ADDR       (8)
#define SCGA_DMA_OFFSET_TRANS_SIZE       (16)
#define SCGA_DMA_OFFSET_ACT              (64)
#define SCGA_DMA_OFFSET_ADDR_TRANS       (0xFF)

typedef enum {
    Idle,
    Loading,
    Storing,
    Completion,
    UnRemap_Load,
    UnRemap_Store,
    UnRemap_Complt
} DmaState;

typedef enum {
    L0_Cache = 0,
    L1_Cache,
    L2_Cache,
    L3_Cache
} CacheType;


class ScGaDma : public DmaDevice
{
    private:
        /**
         * Port on CPU or GPU memory that gets data from one of them,
         * and transfer the data to the other memory.
         *
        class CorePort : public MasterPort
        {
        private:
            ScGaDma *owner;

        public:
            CorePort(const std::string& name, ScGaDma *owner) :
                MasterPort(name, owner), owner(owner)
            { }

            void sendPacket(PacketPtr pkt);
        protected:
            bool recvTimingResp(PacketPtr pkt) override;
            void recvReqRetry() override;
            void recvRangeChange() override;
        };
        */
        //DmaPort cpu_side_mem_port;
        //cpu side memory port is being as dma_port
        DmaPort gpu_side_mem_port;
        DmaPort writeThrPort;

        // instruction sent by main CPU
        bool arg_trans_ready = false;
        bool dma_start_act = false;
        /* <cpu memory address, size > */
        std::vector< std::pair<uint64_t,uint64_t> > cpu_requests;
        /* <gpu memory address, size, data > */
        std::vector< std::pair<
                     std::pair<uint64_t,uint64_t>,uint8_t*> >
                     gpu_transfer_list;
        uint64_t cpu_start_addr;
        uint64_t cpu_transfer_size;

        //EventFunctionWrapper scgaDmaReadEvent;
        //EventFunctionWrapper scgaDmaWriteEvent;
        EventFunctionWrapper writeThrEvent;
        EventFunctionWrapper* createGatherEvent(Addr _paddr);
        EventFunctionWrapper* createCompltEvent(Addr _paddr);
        void scgaDmaReadDone(Addr _addr);
        void scgaDmaWriteDone(Addr _addr);
        void writeThrDone();
        //bool writeThrReqPending() { return writeThrPort.dmaPending(); }

        uint32_t blockCnt;
        uint8_t *dmaReadCPUData;
        uint8_t *dmaWriteGPUData;

        /* temp solution for getting dram content */
        uint8_t *dram_mem;
        uint8_t* getMemContent(Addr _addr, uint32_t _size, void* _dst);
        AddrRange dram_mem_range;

        /* DMA state */
        DmaState dmaState;

        /* Record which cache blocks constituing remapped pages */
        class RemapPage {
        public:
            RemapPage() {}
            RemapPage(Addr _addr,
                      std::vector< std::pair<Addr,uint32_t> >& _blks):
            remappedPageAddr(_addr), constitutedBlocks(_blks)
            {

            }
            RemapPage(const RemapPage& _rp)
            {
                this->remappedPageAddr = _rp.remappedPageAddr;
                this->constitutedBlocks = _rp.constitutedBlocks;
            }
            RemapPage& operator=(const RemapPage& _rp)
            {
                this->remappedPageAddr = _rp.remappedPageAddr;
                this->constitutedBlocks = _rp.constitutedBlocks;
                return *this;
            }

            /* Physical address of remapped page address */
            Addr remappedPageAddr;
            /* Which blocks constitute of remapped page
             * <physical address of the block, corresponding size>
             * */
            std::vector< std::pair<Addr,uint32_t> > constitutedBlocks;
        };

        /* <physical address of remapped page,
         *  corresponding RemaPage instance> */
        std::unordered_map<Addr,RemapPage>::iterator rpIter;
        std::unordered_map<Addr,RemapPage> remapPages;
        std::vector< std::pair<Addr,uint32_t> > currLoadingBlocks;
        std::unordered_set<Addr> currLoadingBlocksSet;

        /* Check loading status */
        bool gatherDone;
        bool storeDone;

        /* Un-Remap, storing remapped pages to original blocks */
        bool isUnRemapStart;
        bool isBlocksBackStart;
        bool isUnRemapLoadDone;
        bool isUnRemapStoreDone;
        std::unordered_set<Addr> unRemapLoadPagesSet;
        std::unordered_set<Addr> unRemapStoreBlocksSet;
        std::unordered_map<Addr,uint8_t*> loadedPagesDataMap;
        EventFunctionWrapper* createUnRemapLoadEvent(Addr _paddr);
        EventFunctionWrapper* createUnRemapStoreEvent(Addr _paddr);
        void unRemapLoadDone(Addr _addr); // _addr is the page address
        void unRemapStoreDone(Addr _addr); // _addr is the block address
        void unRemapClear();

    public:
        typedef ScGaDmaParams Params;

        ScGaDma(ScGaDmaParams *params);
        // declare the two pure virtual functions in the MemObject class,
        // getMasterPort and getSlavePort.
        // These two functions are used by gem during initialization
        // phase to connect memory objects
        // together via ports
        BaseMasterPort& getMasterPort( const std::string& if_name,
                                      PortID idx = InvalidPortID ) override;

        static ScGaDma* scgaDmaInstance;
        static ScGaDma* getInstance() { return scgaDmaInstance;  }
        static void setInstance(ScGaDma* instance) {
                scgaDmaInstance = instance;
        }

        void exec();

        bool handleResponse(PacketPtr pkt);

        AddrRangeList getAddrRanges() const;
        Tick read(PacketPtr pkt);
        Tick write(PacketPtr pkt);

        /* TLB stuff */
        TimingSimpleCPU *cpu;
        std::unordered_map<ContextID, SimpleThread*> addr_trans_map;
        //template <typename T>
        void setAddrTransRule(ContextID _id, SimpleThread* thread);
        Addr getPhyAddr(Addr vaddr, SimpleThread* thread);
        void AddrTransTest(Addr addr, Request::Flags flags, MasterID mid);

        bool getData(Addr addr, int size, uint8_t* loaded_data);
        bool sendData(Addr addr, int size, uint8_t* _data);

        /* When there are cache hits, current protocol would not
         * write through to main memory,
         * so just hacking that the ruby sequencer would call the function
         * to write the data block to main memory,
         * that is, by DMA
         * */
        void writeThrOp(Addr addr, int size,
                        uint8_t* data, CacheType cache_type);

        EventFunctionWrapper* createWriteThrEvent(Addr addr,
                                                  uint8_t* copy_data);
        void processGpuWriteThrDone(Addr addr, uint8_t* copy_data);
        bool writeThrReqPending() { return writeThrPort.dmaPending(); }

        void setDramMem(uint8_t* _mem) { dram_mem = _mem; }
        void setDramMemRange(AddrRange _addr_range)
        { dram_mem_range = _addr_range; }

    protected:

       Addr pioAddr;
       Addr pioSize;
       Tick pioDelay;
       EventFunctionWrapper tickEvent;

};

#endif
