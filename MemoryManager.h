#include <iostream>
#include <cstdint>
#include <math.h>
#include <cmath>
#include <numeric>
#include <signal.h>
#include <stdio.h>
#include <assert.h>
#include<memory>
#include<algorithm>
#if !defined(_WIN32)
#include <sys/wait.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <linux/memfd.h>
#include <string>
#include<sys/ipc.h>
#include<sys/shm.h>
#include<sys/types.h>

#include <stdlib.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <complex>
#include <vector>
#endif
// includes CUDA Runtime
//#include <cuda_runtime.h>

//#include <cuda_runtime_api.h>
/*#include "eigen/Eigen/Core"
#include "eigen/Eigen/Dense"
using namespace Eigen;*/

#if defined(_WIN32)
    //Compiling a Windows DLL - Windows dll export syntax
    #define DETECTION_EXP __declspec(dllexport)
#else
    //Compiling for Linux or any other OS
    #define DETECTION_EXP
#endif

extern "C"
{
#define SIDES_SIZE 5
#define POSITION_EQ 2
#define DEBUG_TEST 0


#define debug_print(fmt, ...) \
            do { if (DEBUG_TEST) if( nPrint > 0 ){fprintf(stderr, fmt, ## __VA_ARGS__); } } while (0)
    // A function adding two integers and returning the result


#if !defined(_WIN32)
    typedef std::complex<float> ComplexFloat;
    typedef std::vector <ComplexFloat> VectorComplexFloat;
    typedef std::shared_ptr<VectorComplexFloat> SharedVectorComplexFloat;

    typedef std::vector <float> VectorFloat;
    typedef std::shared_ptr<VectorFloat> SharedVectorFloat;
    DETECTION_EXP struct BoolNan
    {
        int reason = 0;
        int valid = 0;
    };
    cmplx_float *g_oPtrComplexWrite = NULL ;
    cmplx_float *g_oPtrComplexRead = NULL ;
    double *g_oPtrFloatWrite = NULL ;
    double *g_oPtrFloatRead = NULL ;
    int g_nFDIQRead = 0 ;
    int g_nFDIQWrite = 0 ;
    int g_nFDGPSRead = 0 ;
    int g_nFDGPSWrite = 0 ;

    ///proc/11775/fd/3
    #define SHM_KEY_IQ 0x1234
    #define SHM_KEY_GPS 0x1235
    #define SHM_KEY_PID 0x1236

    bool SetFID( key_t key, int nFD)
    {
       int segment_id  = shmget(key, sizeof(int), 0644|IPC_CREAT);
       if (segment_id == -1)
       {
          return false;
       }
       /* Attach the shared memory segment.  */
       int *shared_memory = (int*) shmat (segment_id, 0, 0);
       *shared_memory = nFD ;
       /* Detach the shared memory segment.  */
       shmdt((void *) shared_memory);

       return true ;
   }

   int GetFID( key_t key, bool bRelease = false)
   {
       int segment_id  = shmget(key, sizeof(int), IPC_EXCL);
       if (segment_id == -1)
       {
          return -1;
       }
       /* Attach the shared memory segment.  */
       int *shared_memory = (int*) shmat (segment_id, 0, 0);
       int nFD = *shared_memory;
       /* Detach the shared memory segment.  */
       shmdt((void *) shared_memory);
       if( bRelease == true)
       {
            /* Deallocate the shared memory segment.  */
            shmctl (segment_id, IPC_RMID, 0);
       }
       return nFD ;
   }
   ////////////////////////////////////////////////////////////////////////////////////////////
   uint64_t GetBufferSize(int fd)
   {
     if(fd < 0)
     {
        printf("GetBufferSize, Could not open fd=%d\n", fd);
        return -1 ;
     }

     struct stat statbuf;
     int err = fstat(fd, &statbuf);
     if(err < 0)
     {
        printf("GetBufferSize, Could not open fd=%d\n", fd);
        return -1 ;
     }

     return statbuf.st_size ;
   }
   int SHMGetPID()
   {
       int nPID = GetFID( SHM_KEY_PID) ;
       if (nPID == -1)
       {
           printf("SHMGetPID, Failed to get and release fd of PID SharedMemory data\n");
           return -1;
       }

       //printf("SHMGetPID PID=%d\n", nPID) ;
       return nPID ;
   }

   std::string SHMGetFDIQPath()
   {
      int nPID = SHMGetPID() ;
      if (nPID == -1)
      {
           printf("SHMGetFDIQHandler, Failed to get and release fd of PID SharedMemory data\n");
           return "";
      }

      int nFDIQ = GetFID(SHM_KEY_IQ) ;
      if (nFDIQ == -1)
      {
            printf("SHMGetFDIQHandler, Failed to get and release fd of IQ SharedMemory data\n");
            return "";
      }

       //printf("SHMGetFDIQHandler, PID=%d\n", nPID) ;
       std::string sPID = std::to_string(nPID) ;

       std::string sFDIQ = std::to_string(nFDIQ) ;
       //printf("SHMGetFDIQHandler, IQ FD=%d\n", nFDIQ) ;
       std::string sPathIQ = "/proc/" + sPID + "/fd/" + sFDIQ ;

       //printf("SHMGetFDIQHandler, sPathIQ=%s\n", sPathIQ.c_str()) ;

       return sPathIQ ;
   }

   int SHMGetFDIQHandler()
   {
        std::string sPathIQ = SHMGetFDIQPath() ;
        if( sPathIQ == "" )
        {
            printf("SHMGetFDIQHandler, Failed to get IQ Memory Map Path from Shared Memory\n");
            return -1;
        }

        int fd = open(sPathIQ.c_str(), O_RDWR);
        if(fd < 0)
        {
            printf("SHMGetFDIQHandler, Failed to open Memory Map File, %s\n", sPathIQ.c_str());
            return -1 ;
        }

        return fd ;
   }

   std::string SHMGetFDGPSPath()
   {
      int nPID = SHMGetPID() ;
      if (nPID == -1)
      {
           printf("SHMGetFDGPSHandler, Failed to get and release fd of PID SharedMemory data\n");
           return "";
      }

      int nFDGPS = GetFID(SHM_KEY_GPS) ;
      if (nFDGPS == -1)
      {
           printf("SHMGetFDGPSHandler, Failed to get and release fd of GPS SharedMemory data\n");
           return "";
      }

       //printf("SHMGetFDGPSHandler, PID=%d\n", nPID) ;
       std::string sPID = std::to_string(nPID) ;

       std::string sFDGPS = std::to_string(nFDGPS) ;
       //printf("SHMGetFDGPSHandler, GPS FD=%d\n", nFDGPS) ;
       std::string sPathGPS = "/proc/" + sPID + "/fd/" + sFDGPS ;

       //printf("SHMGetFDGPSHandler, sPathGPS=%s\n", sPathGPS.c_str()) ;

       return sPathGPS ;
   }

   int SHMGetFDGPSHandler()
   {
        std::string sPathGPS = SHMGetFDGPSPath() ;
        if( sPathGPS == "" )
        {
            printf("SHMGetFDIQHandler, Failed to get GPS Memory Map Path from Shared Memory\n");
            return -1;
        }

        int fd = open(sPathGPS.c_str(), O_RDWR);
        if(fd < 0)
        {
            printf("SHMGetFDIQHandler, Failed to open Memory Map File, %s\n", sPathGPS.c_str());
            return -1 ;
        }

        return fd ;
   }

   cmplx_float *GetGlobalFrameComplexPointer(int *nFDOut, uint64_t *nSizeOut)
   {
     int nFDIQ = SHMGetFDIQHandler() ;
     if(nFDIQ < 0)
     {
        printf("GetGlobalComplexPointer, Failed to get IQ fd from SHMGetFDIQHandler\n");
        return NULL ;
     }

     uint64_t nSize = GetBufferSize(nFDIQ) ;
     if( nSize < 0 )
     {
        return NULL ;
     }

     *nFDOut = nFDIQ ;
     *nSizeOut = nSize ;

     //printf("GetGlobalFrameComplexPointer, buffer size:%lu\n", nSize) ;
     return  (cmplx_float *)mmap(NULL, nSize, PROT_READ | PROT_WRITE, MAP_SHARED, nFDIQ, 0);
   }

   double *GetGlobalGPSDoublePointer(int *nFDOut, uint64_t *nSizeOut)
   {
     int nFDGPS = SHMGetFDGPSHandler() ;
     if(nFDGPS < 0)
     {
        printf("GetGlobalGPSDoublePointer, Failed to get IQ fd from SHMGetFDIQHandler\n");
        return NULL ;
     }

     uint64_t nSize = GetBufferSize(nFDGPS) ;
     if( nSize < 0 )
     {
        return NULL ;
     }

     *nFDOut = nFDGPS ;
     *nSizeOut = nSize ;

     //printf("GetGlobalGPSDoublePointer, buffer size:%lu\n", nSize) ;

     return  (double *)mmap(NULL, nSize, PROT_READ | PROT_WRITE, MAP_SHARED, nFDGPS, 0);
   }

   bool SHMUnmapFrameGM(cmplx_float *oPtrComplex, int nFDIQ, uint64_t nSize)
   {
       try
       {
           //Memory Map might be unmapped but it's file handler opened
           int err = munmap(oPtrComplex, nSize);
           if (err != 0)
           {
               printf("SHMUnmapFrameGM IQ Frame Memory Failed\n");
               //return false;
           }
       }
       catch(...)
       {
       }
       if( close(nFDIQ) < 0 )
       {
           printf("SHMUnmapFrameGM, Failed closing IQ Memory Map Handler\n");
           return false ;
       }
    return true ;
   }

   bool SHMUnmapGPSGM(double *oPtrFloat, int nFDGPS, uint64_t nSize)
   {
       try
       {
           //Memory Map might be unmapped but it's file handler opened
           int err = munmap(oPtrFloat, nSize);
           if (err != 0)
           {
               printf("SHMUnmapGPSGM GPS Frame Memory Failed\n");
               //return false;
           }
       }
       catch(...)
       {
       }

       if( close(nFDGPS) < 0 )
       {
           printf("SHMUnmapGPSGM, Failed closing GPS Memory Map Handler\n");
           return false ;
       }

       return true ;
   }
   /////////////////////////////////////////////////////////////////////////////////////////////////
   /**nFrameBufferSize = Number of Buffers * Buffer (Number of frasmes) * Framesize **/
   DETECTION_EXP int AllocateSharedMemory(uint64_t nFrameBufferSize, uint64_t nGPSBufferSize, void *oResOut)
   {
       BoolNan* oBoolResOut = ((BoolNan*)oResOut);
       try
       {
           if( g_oPtrComplexWrite != NULL)
           {
            return 1 ;
           }
           /**nFrameBufferTotalSize = Number of Buffers * Buffer (Number of frasmes) * Framesize * sizeof type (cmplx_float) **/
           uint64_t nFrameBufferTotalSize = nFrameBufferSize * sizeof(cmplx_float) ;
           //printf("AllocateSharedMemory nFrameBufferTotalSize=%lu\n", nFrameBufferTotalSize) ;

           int nFDIQ = memfd_create("FramesNiartGMM", MFD_ALLOW_SEALING);
           if (nFDIQ == -1)
           {
               oBoolResOut->reason = -206;/*GM_MAP_FAILED*/
               oBoolResOut->valid = 0;
               printf("AllocateSharedMemory, Failed with Creating IQ Global Map for Shared Memory - memfd_create\n");
               return 0;
           }

           if( SetFID(SHM_KEY_IQ, nFDIQ) == false )
           {
               oBoolResOut->reason = -214;/*GM_MAP_FAILED*/
               oBoolResOut->valid = 0;
               printf("AllocateSharedMemory, Failed with keeping IQ fd in Shared Memory\n");
               return 0;
           }

           if( SetFID(SHM_KEY_PID, getpid()) == false )
           {
               oBoolResOut->reason = -218;/*GM_MAP_FAILED*/
               oBoolResOut->valid = 0;
               printf("AllocateSharedMemory, Failed with keeping Process ID Shared Memory\n");
               return 0;
           }
           /*printf("PID: %ld; fd: %d; /proc/%ld/fd/%d\n",
                   (long) getpid(), fd, (long) getpid(), fd);*/
            int ret = ftruncate(nFDIQ, nFrameBufferTotalSize);
            if (ret == -1)
            {
               oBoolResOut->reason = -207;/*GM_MAP_FAILED*/
               oBoolResOut->valid = 0;
               printf("AllocateSharedMemory, Failed with Creating IQ Global Map for Shared Memory - ftruncate, failed to set memory size\n");
               return 0;
            }

            g_oPtrComplexWrite = (cmplx_float *)mmap(NULL, nFrameBufferTotalSize, PROT_READ | PROT_WRITE, MAP_SHARED, nFDIQ, 0);

           /*g_oPtrComplex = (cmplx_float*)mmap(NULL, nFrameBufferTotalSize, PROT_READ | PROT_WRITE,
               MAP_SHARED | MAP_ANONYMOUS,
               fd, 0);*/

           if (g_oPtrComplexWrite == MAP_FAILED)
           {
               oBoolResOut->reason = -200;/*GM_MAP_FAILED*/
               oBoolResOut->valid = 0;
               printf("AllocateSharedMemory, Failed with Creating IQ Global Map for Shared Memory\n");
               return 0;
           }
           ////////////////////////////////////////////////////////////////////////////////////
           ////////////////////////////////////////////////////////////////////////////////////
           uint64_t nGPSBufferTotalSize = nGPSBufferSize * sizeof(double) ;

           //printf("AllocateSharedMemory nGPSBufferTotalSize=%lu\n", nGPSBufferTotalSize) ;

           /* Create an anonymous file in tmpfs; allow seals to be
                  placed on the file */

           int nFDGPS = memfd_create("GPSNiartGMM", MFD_ALLOW_SEALING);
           if (nFDGPS == -1)
           {
               oBoolResOut->reason = -216;/*GM_MAP_FAILED*/
               oBoolResOut->valid = 0;
               printf("AllocateSharedMemory, Failed with Creating GPS Global Map for Shared Memory - memfd_create\n");
               return 0;
           }

           if( SetFID(SHM_KEY_GPS, nFDGPS) == false )
           {
               oBoolResOut->reason = -217;/*GM_MAP_FAILED*/
               oBoolResOut->valid = 0;
               printf("AllocateSharedMemory, Failed with keeping GPS fd in Shared Memory\n");
               return 0;
           }
           /*printf("PID: %ld; fd: %d; /proc/%ld/fd/%d\n",
                   (long) getpid(), fd, (long) getpid(), fd);*/
            ret = ftruncate(nFDGPS, nGPSBufferTotalSize);
            if (ret == -1)
            {
               oBoolResOut->reason = -207;/*GM_MAP_FAILED*/
               oBoolResOut->valid = 0;
               printf("AllocateSharedMemory, Failed with Creating IQ Global Map for Shared Memory - ftruncate, failed to set memory size\n");
               return 0;
            }

            g_oPtrFloatWrite = (double *)mmap(NULL, nGPSBufferTotalSize, PROT_READ | PROT_WRITE, MAP_SHARED, nFDGPS, 0);

           /*g_oPtrComplex = (cmplx_float*)mmap(NULL, nFrameBufferTotalSize, PROT_READ | PROT_WRITE,
               MAP_SHARED | MAP_ANONYMOUS,
               fd, 0);*/

           if (g_oPtrFloatWrite == MAP_FAILED)
           {
               oBoolResOut->reason = -201;/*GM_MAP_FAILED*/
               oBoolResOut->valid = 0;
               printf("AllocateSharedMemory, Failed with Creating GPS Global Map for Shared Memory\n");
               return 0;
           }

           int err1 = munmap(g_oPtrComplexWrite, nFrameBufferTotalSize);
           if (err1 != 0)
           {
               oBoolResOut->reason = -202;/*GM_UNMAP_FAILED*/
               oBoolResOut->valid = 0;

               printf("AllocateSharedMemory, UnMapping IQ buffers Memory Failed\n");
               return 0;
           }

           int err2 = munmap(g_oPtrFloatWrite, nGPSBufferTotalSize);
           if (err2 != 0)
           {
               oBoolResOut->reason = -203;/*GM_UNMAP_FAILED*/
               oBoolResOut->valid = 0;
               printf("AllocateSharedMemory, UnMapping GPS Memory Failed\n");
               return 0;
           }

           g_nFDIQWrite = nFDIQ;
           g_nFDGPSWrite = nFDGPS;
           ////////////////////////////////////////////////////////////////////////////////////
           ////////////////////////////////////////////////////////////////////////////////////
           //printf("AllocateSharedMemory GR8\n") ;
           printf("Allocated Global Memory - OK\n") ;
           ////////////////////////////////////////////////////////////////////////////////////////////////////////
           oBoolResOut->valid = 1;
           return 1 ;
       }
       catch(...)
       {
           oBoolResOut->valid = 0;
           printf("AllocateSharedMemory, General Failure\n") ;
           return 0 ;
       }
   }
   
   DETECTION_EXP int DeallocateSharedMemory(void* oResOut)
   {
       BoolNan* oBoolResOut = ((BoolNan*)oResOut);
       try
       {
           if (close(g_nFDIQWrite) < 0)
           {
               oBoolResOut->reason = -225;
               oBoolResOut->valid = 0;
               printf("DeallocateSharedMemory, Failed closing IQ Memory Map Handler\n");
           }

           if (close(g_nFDGPSWrite)  < 0)
           {
               oBoolResOut->reason = -226;
               oBoolResOut->valid = 0;
               printf("DeallocateSharedMemory, Failed closing GPS Memory Map, failed to retrieve fd handler\n");
           }

           /////////////////////////////////////////////////////////
           //Release Shared Memory, required for opening Memory Map
           GetFID(SHM_KEY_PID, true) ;
           GetFID(SHM_KEY_IQ, true) ;
           GetFID(SHM_KEY_GPS, true) ;
           /////////////////////////////////////////////////////////

           g_oPtrComplexWrite = NULL ;
           g_oPtrFloatWrite = NULL ;
           //printf("DeallocateSharedMemory GR8\n") ;
           //////////////////////////////////////////////////////////////////////////////////
           oBoolResOut->valid = 1;
           printf("Free Global Memory - OK\n");
           return 1;
       }
       catch(...)
       {
           g_oPtrComplexWrite = NULL ;
           g_oPtrFloatWrite = NULL ;
           oBoolResOut->valid = 0;
           printf("DeallocateSharedMemory, General Failure\n") ;
           return 0 ;
       }
   }

    DETECTION_EXP int SetBuffers(int nIndexBuffer, uint64_t* oFrameArr, uint64_t nFrameSize, double* oGPSArr, uint64_t nGPSItemSize, uint64_t nNumberOfItemsInBuffer, void* oResOut)
    {
       BoolNan* oBoolResOut = ((BoolNan*)oResOut);
       try
       {
           int nFDIQ = 0 ;
           uint64_t nBufferIQTotalSize = 0 ;
           //Retrieve IQ Frames Memory Map Pointer
           cmplx_float *oPtrComplex = GetGlobalFrameComplexPointer(&nFDIQ, &nBufferIQTotalSize) ;
           if (oPtrComplex == NULL)
           {
                   oBoolResOut->reason = -233;/*GM_GET_IQ_FD*/
                   oBoolResOut->valid = 0;

                   printf("Failed to get pointer to IQ SharedMemory data\n");
                   return 0;
           }
           //printf("SetBuffers GetGlobalFrameComplexPointer nFDIQ=%d, nBufferIQTotalSize=%d\n",nFDIQ, nBufferIQTotalSize ) ;
           int nFDGPS = 0 ;
           uint64_t nBufferGPSTotalSize = 0 ;
           //Retrieve GPS Memory Map Pointer
           double *oPtrFloat = GetGlobalGPSDoublePointer(&nFDGPS, &nBufferGPSTotalSize) ;
           if (oPtrFloat == NULL)
           {
                   oBoolResOut->reason = -234;/*GM_GET_IQ_FD*/
                   oBoolResOut->valid = 0;

                   printf("Failed to get pointer to GPS SharedMemory data\n");
                   return 0;
           }

           //printf("SetBuffers GetGlobalGPSDoublePointer nFDGPS=%d, nBufferGPSTotalSize=%d\n",nFDGPS, nBufferGPSTotalSize ) ;
           ///////////////////////////////////////////////////////////////////
           //BoolNan* oBoolResOut = ((BoolNan*)oResOut);

           //Calculates buffer starting and ending point - Number of frames * Frame/GPS size * Number of buffers
            uint64_t nStartIQ = nIndexBuffer * nNumberOfItemsInBuffer * nFrameSize;
            uint64_t nEndIQ = (nIndexBuffer + 1) * (nNumberOfItemsInBuffer * nFrameSize) - 1;

            uint64_t nStartGPS = nIndexBuffer * nNumberOfItemsInBuffer * nGPSItemSize;
            uint64_t nEndGPS = (nIndexBuffer + 1) * (nNumberOfItemsInBuffer * nGPSItemSize) - 1;

            for (uint64_t nIndex2 = 0; nIndex2 < nFrameSize*nNumberOfItemsInBuffer; nIndex2++)
            {
                int32_t nI, nQ;
                uint64_t oFrame = oFrameArr[nIndex2];
                //printf("oFrame[%d] =%ld\n", nIndex2, oFrame) ;
                /*if (nIndex2 < 35)
                {
                    printf("Lib CalculatedIQQueue index=%d, val=%ju\n", nIndex2, oFrame);
                }*/
                nI = (oFrame & 0x0000000000003ffc) >> 2 | (oFrame & 0x000000003ffc0000) >> 6;
                nQ = (oFrame & 0x00003ffc00000000) >> 34 | (oFrame & 0x3ffc000000000000) >> 38;
                nI = (nI << 8) >> 8;
                nQ = (nQ << 8) >> 8;

                oPtrComplex[nStartIQ + nIndex2].real = nI;
                oPtrComplex[nStartIQ + nIndex2].imag = nQ;
                //printf("nI=%d, nQ=%d\n", nI, nQ) ;
                //printf("GetBuffers oPtrComplex[%d].real=%f oPtrComplex[%d].imag=%f\n\n", nStartIQ + nIndex2, oPtrComplex[nStartIQ + nIndex2].real, nStartIQ + nIndex2, oPtrComplex[nStartIQ + nIndex2].imag) ;
            }

            //printf("OOOOOKKKKKK\n") ;
            uint64_t nSize = nNumberOfItemsInBuffer * nGPSItemSize;
            //Retrieves buffer type float for writing, increase "Queue" index, Frames and GPS buffers are filled with [X] number of frames
            //printf("Lib SetBuffer GPS End Loop=%d\n", nSize) ;
            for (uint64_t nIndex1 = 0; nIndex1 < nSize; nIndex1++)
            {
                oPtrFloat[nStartGPS+nIndex1] = oGPSArr[nIndex1];
                //printf( "Buffer Index=%d, GPS[%d]=%f, SM[%d]=%f\n", nIndexBuffer, nIndex1, oGPSArr[nIndex1], nStartGPS+nIndex1, oPtrFloat[nStartGPS+nIndex1]) ;
            }
           //Unmap MemoryMap and close MemoryMap Handler
           SHMUnmapFrameGM(oPtrComplex, nFDIQ, nBufferIQTotalSize) ;
           SHMUnmapGPSGM(oPtrFloat, nFDGPS, nBufferGPSTotalSize) ;
           oBoolResOut->valid = 1;
           //printf("SetBuffers, GR8 OK\n") ;
           return 1 ;
       }
       catch(...)
       {
           oBoolResOut->valid = 0;
           printf("SetBuffers, General Failure\n") ;
           return 0 ;
       }
    }

   DETECTION_EXP int GetBuffers(int nIndexBuffer, void* oComplexArrOut, uint64_t nFrameSize, void* oGPSArrOut, uint64_t nGPSItemSize, uint64_t nNumberOfItemsInBuffer, void* oResOut)
   {
       BoolNan* oBoolResOut = ((BoolNan*)oResOut);
       try
       {
           /*if( g_oPtrComplexRead == NULL)
           {
            AllocateSharedMemoryRead(oResOut) ;
            printf("GetBuffers, called to AllocateSharedMemoryRead\n") ;
           }*/
           int nFDIQ = 0 ;
           uint64_t nBufferIQTotalSize = 0 ;
           //Retrieve IQ Frames Memory Map Pointer
           cmplx_float *oPtrComplex = GetGlobalFrameComplexPointer(&nFDIQ, &nBufferIQTotalSize) ;
           if (oPtrComplex == NULL)
           {
                   oBoolResOut->reason = -233;/*GM_GET_IQ_FD*/
                   oBoolResOut->valid = 0;

                   printf("Failed to get pointer to IQ SharedMemory data\n");
                   return 0;
           }
           //printf("GetBuffers GetGlobalFrameComplexPointer nFDIQ=%d, nBufferIQTotalSize=%d\n",nFDIQ, nBufferIQTotalSize ) ;
           int nFDGPS = 0 ;
           uint64_t nBufferGPSTotalSize = 0 ;
           //Retrieve GPS Memory Map Pointer
           double *oPtrFloat = GetGlobalGPSDoublePointer(&nFDGPS, &nBufferGPSTotalSize) ;
           if (oPtrFloat == NULL)
           {
                   oBoolResOut->reason = -234;/*GM_GET_IQ_FD*/
                   oBoolResOut->valid = 0;

                   printf("Failed to get pointer to GPS SharedMemory data\n");
                   return 0;
           }
           //printf("GetBuffers GetGlobalFrameComplexPointer nFDGPS=%d, nBufferGPSTotalSize=%d\n",nFDGPS, nBufferGPSTotalSize ) ;
           ///////////////////////////////////////////////////////////////////

           //Calculates buffer starting and ending point - Number of frames * Frame/GPS size * Number of buffers
           uint64_t nStartIQ = nIndexBuffer * nNumberOfItemsInBuffer * nFrameSize;
           uint64_t nEndIQ = (nIndexBuffer + 1) * (nNumberOfItemsInBuffer * nFrameSize) - 1;

           uint64_t nStartGPS = nIndexBuffer * nNumberOfItemsInBuffer * nGPSItemSize;
           uint64_t nEndGPS = (nIndexBuffer + 1) * (nNumberOfItemsInBuffer*nGPSItemSize) - 1 ;


           //printf("GetIQBuffer 1\n");
           oBoolResOut->valid = 1;

           //Retrieves buffer type complex float for reading
           cmplx_float* oFloatComplexOutArr = ((cmplx_float*)oComplexArrOut);
           uint64_t nIndex = 0;

           //std::memcpy(oFloatComplexOutArr, oPtrComplex, nNumberOfItemsInBuffer * nFrameSize * sizeof(cmplx_float));
           //printf("GetBuffers nStartIQ=%lu nEndIQ=%lu\n", nStartIQ, nEndIQ) ;
          for (uint64_t i = nStartIQ; i <= nEndIQ; i++)
           {
               //printf("GetBuffers i=%d g_oPtrComplex[i].real=%f g_oPtrComplex[i].imag=%f\n", i, g_oPtrComplex[i].real, g_oPtrComplex[i].imag) ;
               //printf("GetIQBuffer 2\n");
               oFloatComplexOutArr[nIndex].real = oPtrComplex[i].real;
               oFloatComplexOutArr[nIndex].imag = oPtrComplex[i].imag;
               //printf("GetBuffers oPtrComplex[%d].real=%f oPtrComplex[%d].imag=%f\n", i, oPtrComplex[i].real, i, oPtrComplex[i].imag) ;
               //printf("GetBuffers oFloatComplexOutArr[%d].real=%f oFloatComplexOutArr[%d].imag=%f\n", nIndex, oFloatComplexOutArr[nIndex].real, nIndex, oFloatComplexOutArr[nIndex].imag) ;
               nIndex++ ;
               //printf("GetIQBuffer 3\n");
           }
           // memcpy(oFloatComplexOutArr, oPtrComplex, nEndIQ-nStartIQ);

           //printf("GetIQBuffer 1.1\n");

           //printf("GetGPSBuffer 2\n");
           oBoolResOut->valid = 1;

           //Retrieves buffer type complex float for reading
           double* oFloatOutArr = ((double*)oGPSArrOut);
           nIndex = 0;
           for (uint64_t i = nStartGPS; i <= nEndGPS; i++)
           {
               //printf("GetIQBuffer 2\n");
               oFloatOutArr[nIndex] = oPtrFloat[i];
               //printf("GetBuffers oPtrFloat[%d]=%f\n", i, oPtrFloat[i]) ;
               //printf("GetBuffers oFloatOutArr[%d]=%f\n", nIndex, oFloatOutArr[nIndex]) ;

               nIndex++ ;
               //printf("GetIQBuffer 3\n");
           }

           //printf("GetGPSBuffer 2.1\n");

          //Unmap MemoryMap and close MemoryMap Handler
           SHMUnmapFrameGM(oPtrComplex, nFDIQ, nBufferIQTotalSize) ;
           SHMUnmapGPSGM(oPtrFloat, nFDGPS, nBufferGPSTotalSize) ;
           oBoolResOut->valid = 1;
           //printf("GetBuffers, GR8 OK\n") ;
           return 1 ;
       }
       catch(...)
       {
           oBoolResOut->valid = 0;
           printf("GetBuffers, General Failure\n") ;
           return 0 ;
       }
   }
#endif
}