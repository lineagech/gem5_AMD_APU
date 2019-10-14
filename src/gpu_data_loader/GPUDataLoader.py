from m5.params import *
from m5.proxy import *
from MemObject import MemObject
from m5.SimObject import SimObject

class GpuDataLoader(MemObject):
    type = 'GpuDataLoader'
    cxx_header = 'gpu_data_loader/gpu_data_loader.hh'

    cpu_side_mem_port = MasterPort("")
    gpu_side_mem_port = MasterPort("")

    sys = Param.System(Parent.any, "")

    cpuMemRange = Param.AddrRange("")
    gpuMemRange = Param.AddrRange("")

    shader = Param.Shader("")
    PageSize = Param.Int(4096, "Page Size") 
    MemSize = Param.Int(4*1024*1024, "Memory Size")
