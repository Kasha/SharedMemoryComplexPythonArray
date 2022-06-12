import os
import sys
from ctypes import *
import numpy as np
import glob

from Modules.Config import Configuration as MainConfig
RadarConfig = MainConfig()

class BoolNan(Structure):
    _fields_ = [('reason', c_int),
                ('valid', c_int)]
MemoryManagerLib = None
if sys.platform=='win32':
    PROJECT_ROOT_REG = os.path.abspath(os.path.dirname(__file__))
    path = PROJECT_ROOT_REG + RadarConfig.WinMemoryManagerDLLBuildPath + RadarConfig.WinMemoryManagerDLLName
    MemoryManagerLib = windll.LoadLibrary(path)
else:
    ###lib = ctypes.CDLL("/ext_ssd/DetectionsSO/main.so")  # (r'C:\Users\Train\source\repos\Dll1\x64\Release\Dll1.dll')
    PROJECT_ROOT_REG = os.path.abspath(os.path.dirname(__file__))
    path_lib = PROJECT_ROOT_REG + '/build/*/MemoryManager*.so'
    libfile = glob.glob(path_lib)[0]
    # 1. open the shared library
    MemoryManagerLib = CDLL(libfile)

    print("Load Memory Manager Lib:",os.getpid())

c_uint_array = np.ctypeslib.ndpointer(dtype=np.uint, ndim=1, flags='C_CONTIGUOUS')

class cmplx_float(Structure):
    _fields_ = [('real', c_float),
                ('imag', c_float)]

MemoryManagerLib.AllocateSharedMemory.argtypes = [c_uint64, c_uint64, c_void_p]
MemoryManagerLib.AllocateSharedMemory.restype = c_int

MemoryManagerLib.AllocateSharedMemoryRead.argtypes = [c_void_p]
MemoryManagerLib.AllocateSharedMemoryRead.restype = c_int

MemoryManagerLib.DeallocateSharedMemory.argtypes = [c_void_p]
MemoryManagerLib.DeallocateSharedMemory.restype = c_int

MemoryManagerLib.DeallocateSharedMemoryRead.argtypes = [c_void_p]
MemoryManagerLib.DeallocateSharedMemoryRead.restype = c_int

c_float64_array = np.ctypeslib.ndpointer(dtype=np.float64, ndim=1, flags='C_CONTIGUOUS')

MemoryManagerLib.SetBuffers.argtypes = [c_int, c_uint_array, c_uint64, c_float64_array, c_uint64, c_uint64, c_void_p]
MemoryManagerLib.SetBuffers.restype = c_int

MemoryManagerLib.GetBuffers.argtypes = [c_int, c_void_p, c_uint64, c_void_p, c_uint64, c_uint64, c_void_p]
MemoryManagerLib.GetBuffers.restype = c_int

