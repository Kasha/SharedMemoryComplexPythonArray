import ctypes
import numpy as np
import glob
from ctypes import *

# find the shared library, the path depends on the platform and Python version
#lib = ctypes.CDLL("/ext_ssd/DetectionsSO/main.so")  # (r'C:\Users\Train\source\repos\Dll1\x64\Release\Dll1.dll')
libfile = glob.glob('build/*/DetectionProcess*.so')[0]

# 1. open the shared library
lib = ctypes.CDLL(libfile)

# 2. tell Python the argument and result types of functions in main
lib.add.argtypes = (ctypes.c_int, ctypes.c_int)
lib.add.restype = ctypes.c_int

c_float_array = np.ctypeslib.ndpointer(dtype=np.float32, ndim=1, flags='C_CONTIGUOUS')
lib.sum.argtypes = (c_float_array, ctypes.c_int)
lib.sum.restype = ctypes.c_float

lib.FindLocation.argtypes = (
    c_float_array, c_float_array, c_float_array, c_float_array, c_float_array, c_float_array, c_float_array,
    c_float_array, c_float_array, ctypes.c_int, ctypes.c_int, ctypes.c_float)
"""
c_uint_array = np.ctypeslib.ndpointer(dtype="uint64", ndim=1, flags='C_CONTIGUOUS')
c_int_array = np.ctypeslib.ndpointer(dtype="int32", ndim=1, flags='C_CONTIGUOUS')

c_uint_array = np.ctypeslib.ndpointer(dtype="uint64", ndim=1, flags='C_CONTIGUOUS')
lib.ParseIQ.argtypes = (c_uint_array,
                        c_uint32,
                        POINTER(c_int32),
                        POINTER(c_int32))

lib.CalculatedIQ.argtypes = [c_uint_array, c_int, c_void_p]

lib.CalculatedIQQueue.argtypes = [c_uint_array, c_int, c_int]
class BoolNan(Structure):
    _fields_ = [('val', c_int),
                ('valid', c_int)]

c_bool_nan_pointer = POINTER(BoolNan)

lib.GetIQBuffer.argtypes = [c_void_p, c_bool_nan_pointer]

c_float64_array = np.ctypeslib.ndpointer(dtype=np.float64, ndim=1, flags='C_CONTIGUOUS')


lib.GetGPSBuffer.argtypes = [c_void_p, c_void_p]

#np.ctypeslib.ndpointer(np.complex64, ndim=1, flags='C'), c_int
#c_complex64_array = np.ctypeslib.ndpointer(dtype=np.complex64, ndim=1, flags='C')
"""
dim = 5000

import numpy as np
from multiprocessing import shared_memory, Process, Lock
from multiprocessing import cpu_count, current_process
import time

lock = Lock()

def add_one(shr_name):

    existing_shm = shared_memory.SharedMemory(name=shr_name)
    np_array = np.ndarray((dim, dim,), dtype=np.int64, buffer=existing_shm.buf)
    lock.acquire()
    np_array[:] = np_array[0] + 1
    lock.release()
    time.sleep(10) # pause, to see the memory usage in top
    print('added one')
    existing_shm.close()

def create_shared_block():

    a = np.ones(shape=(dim, dim), dtype=np.int64)  # Start with an existing NumPy array

    shm = shared_memory.SharedMemory(create=True, size=a.nbytes)
    # # Now create a NumPy array backed by shared memory
    np_array = np.ndarray(a.shape, dtype=np.int64, buffer=shm.buf)
    np_array[:] = a[:]  # Copy the original data into shared memory
    return shm, np_array

if __name__ == '__main__':
    if current_process().name == "MainProcess":
        print("creating shared block")
        shr, np_array = create_shared_block()

        processes = []
        for i in range(cpu_count()):
            _process = Process(target=add_one, args=(shr.name,))
            processes.append(_process)
            _process.start()

        for _process in processes:
            _process.join()

        print("Final array")
        print(np_array[:10])
        print(np_array[10:])

        shr.close()
        shr.unlink()
    try:
        print("start")
        """Range_AllPlot = [0, 999]
        UtmX_AllPlot = [0, 999]
        UtmX_Plot = [0, 999]
        PowerDb_AllPlot = [0, 999]
        Range_Plot = [0, 999]
        DeltaRange = 7.9
        fScore_DeltaX_DeltaY = 7.5
            lib.FindLocation(
                    fScore_DeltaX_DeltaY,
                    np.asarray(Range_AllPlot, dtype=np.float32),
                    np.asarray(UtmX_AllPlot-UtmX_Plot[0], dtype=np.float32),
                    np.asarray(UtmY_AllPlot-UtmY_Plot[0], dtype=np.float32),
                    np.asarray(PowerDb_AllPlot, dtype=np.float32),
                    np.asarray(Range_Plot, dtype=np.float32),
                    np.asarray(UtmX_Plot-UtmX_Plot[0], dtype=np.float32),
                    np.asarray(UtmY_Plot-UtmY_Plot[0], dtype=np.float32),
                    np.asarray(PowerDb_Plot, dtype=np.float32),
                    Range_AllPlot.size,
                    Range_Plot.size,
                    float(DeltaRange))"""
        float_Element = np.array([5.1, 1.1, 3.1, 2.1, 4.1], dtype=np.float32)
        print("Sum = ", lib.sum(float_Element, float_Element.size))

        res = np.zeros(((32 - 1) * 8192), dtype=np.complex64)


        #resI = np.zeros(shape=(32 * 8192), dtype="int32", order='C')
        #resQ = np.zeros(shape=(32 * 8192), dtype="int32", order='C')
        #####resI = (c_int32 * (32*8192))(*range((32*8192)))
        ######resQ = (c_int32 * (32*8192))(*range((32*8192)))
        oFrames = np.array([757,4,3,2,1,3,4,5,6,7,8,9,8,7,6,54,3,2,1,2,3,4,5,6], dtype="uint64")
        """z = (c_void_p * (31 * 8192))(*range((31 * 8192)))
        z[0].real = 1.0
        z[0].img = 2.0

        z[1].real = 31.3
        z[1].img = 41.0

        z[2].real = 77.2
        z[2].img = 88.3"""
        z = np.array([1 + 2j, -3 + 4j, 5.0j], dtype=np.complex64)

        N = 4
        myresult = np.array([757,4,3,2,1,3,4,5,6,7,8,9,8,7,6,54,3,2,1,2,3,4,5,6], dtype=np.complex64)
        p = c_void_p(myresult.ctypes.data)
        ###lib.CalculatedIQ(oFrames, len(oFrames), p)
        #print("lib.CalculatedIQ OK:", myresult[1])

        arr = np.zeros((3,4), np.complex64)
        arr[0][0] = 1 + 2j
        arr[1][0] = -3 + 4j
        33, 554, 432
        #lib.AllocateSharedMemory(33554432,5)
        # make it two-dimensional

        ###lib.CalculatedIQQueue(oFrames, oFrames.size, 1)

        ####resIQarr = np.zeros(oFrames.size * 1, dtype=np.complex64)
        #####pFrames = c_void_p(resIQarr.ctypes.data)

        ####pResFrame = BoolNan()
        ####lib.GetIQBuffer(pFrames, byref(pResFrame))
        #####a=1
        #####del pResFrame
        """p = c_void_p(c_void_p(arr.ctypes.data))
        lib.CalculatedIQQueue1(p)
        print("lib.CalculatedIQ OK:", arr[1][0])"""

        #lib.CalculatedIQQueue1
        #####lib.ParseIQ(oFrames, oFrames.size, resI, resQ)
        #####print("lib.ParseIQ OK:", resI[0])

        #arr = np.zeros((3,4), dtype=np.complex64)
        #arr[0][0] = 1 + 2j
        #arr[1][0] = 4 + 2.8j
        """resIQarr1 = np.array([[1 + 2j, -3 + 4j, 5.0j],[47 + 4.5j, -3 + 4j, 5.3j]], dtype=np.complex64)
        p = resIQarr1.ctypes.data
        lib.CalculatedIQQueue1(oFrames, oFrames.size, p)
        print("lib.CalculatedIQQueue1 OK:", resIQarr1[0][1])"""




        """a1 = np.array([1,2], dtype=np.int)
        b1 = np.array([1,3], dtype=np.int)
        Herminitian_DoppH_Dopp = np.matmul(a1, b1)

        a = np.array([1 + 2j, -3 + 4j, 5+3j], dtype=np.complex64)
        a = np.ascontiguousarray(a, dtype=np.complex64)

        b = np.array([1 + 2j, -12 + 4j, 5.0j], dtype=np.complex64)
        b = np.ascontiguousarray(b, dtype=np.complex64)
        lib.FindAngelDeconv_Dopp(a, b, a.size)
        print("b[0]",b[0])"""
        print("gr8 ok")
    except KeyboardInterrupt:
       print("Error")



