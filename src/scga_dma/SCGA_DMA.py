from m5.params import *
from m5.proxy import *
from MemObject import MemObject
from Device import DmaDevice
from m5.SimObject import SimObject

class ScGaDma(DmaDevice):
    type = 'ScGaDma'
    cxx_header = 'scga_dma/scga_dma.hh'
    
    pio_addr = Param.Addr(0x500000000, "Device Address")
    #pio_addr = Param.Addr(0x300000000, "Device Address")
    pio_latency = Param.Latency('1ns', "Programmed IO latency")

    translation_port = MasterPort('Port to TLB')

    cpu_side_mem_port = MasterPort('Port to CPU memory ctrl')
    gpu_side_mem_port = MasterPort('Port to GPU memory ctrl')

    writeThrPort = MasterPort("")

    cpu = Param.TimingSimpleCPU("")
    
