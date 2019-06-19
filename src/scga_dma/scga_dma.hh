#ifndef __SCGA_DMA__
#define __SCGA_DMA__

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

#define SCGA_DMA_VIRTUAL_START           (0x20000000)
#define SCGA_DMA_PHYSICAL_START          (0x300000000)

#define SCGA_DMA_OFFSET_CONFIG           (0)
#define SCGA_DMA_OFFSET_START_ADDR       (8)
#define SCGA_DMA_OFFSET_TRANS_SIZE       (16)
#define SCGA_DMA_OFFSET_ACT              (64)
#define SCGA_DMA_OFFSET_ADDR_TRANS       (0xFF)


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

        EventFunctionWrapper scgaDmaReadEvent;
        EventFunctionWrapper scgaDmaWriteEvent;
        EventFunctionWrapper writeThrEvent;
        void scgaDmaReadDone();
        void scgaDmaWriteDone();
        void writeThrDone();
        //bool writeThrReqPending() { return writeThrPort.dmaPending(); }

        uint8_t *dmaReadCPUData;
        uint8_t *dmaWriteGPUData;

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

        bool getDataFromCPU(Addr addr, int size);
        bool sendDataToGPU(Addr addr, int size);

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

    protected:

       Addr pioAddr;
       Addr pioSize;
       Tick pioDelay;
       EventFunctionWrapper tickEvent;

};

#endif
