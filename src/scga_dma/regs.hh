/**
 * Define Scatter-Gather DMA registers
 * Based on XILINX AXI DMA v7.1 LogiCORE IP Product Guide
 */

#ifndef __REG_HH__
#define __REG_HH__

#define C_ADDRBASE            (0x300000000)

#define MM2S_DMACR          (0x0)
#define MM2S_DMASR          (0x4)
#define MM2S_CURDESC        (0x8)
#define MM2S_CURDESC_MSB    (0xC)
#define MM2S_TAILDESC       (0x10)
#define MM2S_TAILDESC_MSB   (0x14)

#define SG_CTL              (0x2c)
#define S2MM_DMACR          (0x30)
#define S2MM_DMASR          (0x34)
#define S2MM_CURDESC        (0x38)
#define S2MM_CURDESC_MSB    (0x3c)
#define S2MM_TAILDESC       (0x40)
#define S2MM_TAILDESC_MSB   (0x44)

#define UNREMAP             (0x100)
#define DMA_NOOP            (0x200)

typedef struct {
    uint32_t RS : 1;
    uint32_t RSVD : 1;
    uint32_t RSEST : 1;
    uint32_t Keyhole : 1;
    uint32_t Cyclic_BD_Enable : 1;
    uint32_t RSVD_2 : 7;
    uint32_t IOC_IrqEN : 1;
    uint32_t Dly_IrqEN : 1;
    uint32_t ERR_IrqEN : 1;
    uint32_t RSVD_3 : 1;
    uint32_t REM;
} MM2S_DMACR_REG;

typedef struct {
    uint32_t Halted : 1;
    uint32_t Idle : 1;
    uint32_t RSVD : 1;
    uint32_t SGIncId : 1;
    uint32_t DMAIntErr : 1;
    uint32_t DMASlvErr : 1;
    uint32_t DMADecErr : 1;
    uint32_t RSVD_2 : 1;
    uint32_t SGIntErr : 1;
    uint32_t SGSlvErr : 1;
    uint32_t SGDecErr : 1;
    uint32_t RSVD_3 : 1;
    uint32_t IOC_Irq : 1;
    uint32_t Dly_Irq : 1;
    uint32_t Err_Irq : 1;
    uint32_t RSVD_4 : 1;
    uint32_t IRQThresholdSts : 8;
    uint32_t IRQDelaySts : 8;
} MM2S_DMASR_REG;

typedef struct {
    //uint32_t RSVD : 6;
    uint32_t CurrDescriptor;
} MM2S_CURDESC_REG;

typedef struct {
    uint32_t CurrDescriptorMSB;
} MM2S_CURDESC_MSB_REG;

typedef struct {
    //uint32_t RSVD : 6;
    uint32_t TailDescriptor;
} MM2S_TAILDESC_REG;

typedef struct {
    uint32_t TailDescriptorMSB;
} MM2S_TAILDESC_MSB_REG;

/*
typedef struct {
    uint32_t SourceAddress;
} MM2S_SA_REG;

typedef struct {
    uint32_t SourceAddressMSB;
} MM2S_SA_MSB_REG;

typedef struct {
    uint32_t length;
} MM2S_LENGTH_REG;

typedef struct {
    uint32_t SG_CACHE : 4;
    uint32_t RSVD : 4;
    uint32_t SG_USER : 4;
} SG_CTL_REG;

typedef struct {
    uint32_t NextDescriptor;
} S2MM_NXTDESC_REG;

typedef struct {
    uint32_t NexDescriptor;
} S2MM_NXTDESC_MSB_REG;

typedef struct {
    uint32_t BufferAddress;
} S2MM_BUFFER_ADDRESS_REG;

typedef struct {
    uint32_t BufferLen : 26;
    uint32_t RXEOF : 1;
    uint32_t RXSOF : 1;
    uint32_t RSVD : 4;
} S2MM_CONTROL_REG;

typedef struct {
    uint32_t TransBytes : 26;
    uint32_t RXEOF : 1;
    uint32_t RXSOF : 1;
    uint32_t DMAIntErr : 1;
    uint32_t DMASlvErr : 1;
    uint32_t DMADecErr : 1;
    uint32_t Complt : 1;
} S2MM_STATUS_REG;
*/

typedef MM2S_DMACR_REG          S2MM_DMACR_REG;
typedef MM2S_DMASR_REG          S2MM_DMASR_REG;
typedef MM2S_CURDESC_REG        S2MM_CURDESC_REG;
typedef MM2S_CURDESC_MSB_REG    S2MM_CURDESC_MSB_REG;
typedef MM2S_TAILDESC_REG       S2MM_TAILDESC_REG;
typedef MM2S_TAILDESC_MSB_REG   S2MM_TAILDESC_MSB_REG;

typedef struct {
    uint32_t NXTDESC; // 00h
    uint32_t NXTDESC_MSB; // 04h
    uint32_t BUFFER_ADDR; // 08h
    uint32_t BUFFER_ADDR_MSB; // 0ch
    uint32_t RSVD; // 10h
    uint32_t RSVD_2; // 14h
    struct CONTROL {
        uint32_t BUFFER_LENGTH : 26;
        uint32_t _EOF : 1;
        uint32_t _SOF : 1;
        uint32_t RSVD;
    } _ctrl; // 18h
    struct STATUS {
        uint32_t TRANS_BYTES : 26;
        uint32_t RSVD : 2;
        uint32_t DMAIntErr : 1;
        uint32_t DMASlvErr : 1;
        uint32_t DMADecErr : 1;
        uint32_t Complt : 1;
    } _status; // 1ch
} SCGA_DESCRIPTOR;

#endif
