/* Prints HIP header values the probe pins in src/hipapi.rs. Build against a HIP include tree:
 *   cc -D__HIP_PLATFORM_AMD__ -I<rocm>/include -o dump2 dump2.c && ./dump2
 * (layout/fetch-headers.sh fetches the 6.2.0 and 7.0.0 trees used for layout/rocm-*.txt). */
#include <stdio.h>
#include <stddef.h>
#include <hip/hip_runtime_api.h>
#include <hip/hiprtc.h>
#define V(x) printf("val %s %lld\n", #x, (long long)(x));
#define S(t) printf("sizeof %s %zu\n", #t, sizeof(t));
#define O(t,f) printf("offsetof %s.%s %zu\n", #t, #f, offsetof(t,f));
int main(void){
V(hipHostRegisterPortable) V(hipHostRegisterMapped) V(hipHostRegisterReadOnly) V(hipExtHostRegisterCoarseGrained)
V(hipEventDisableTiming) V(hipEventInterprocess) V(hipIpcMemLazyEnablePeerAccess) V(hipStreamNonBlocking)
V(hipMemcpyHostToDevice) V(hipMemcpyDeviceToHost) V(hipHostMallocMapped) V(hipHostMallocDefault)
V(hipMemAllocationTypePinned) V(hipMemHandleTypePosixFileDescriptor) V(hipMemLocationTypeDevice)
V(hipMemAccessFlagsProtReadWrite) V(hipMemAllocationGranularityMinimum) V(hipMemAllocationGranularityRecommended)
V(hipExternalMemoryHandleTypeOpaqueFd) V(hipErrorNotSupported) V(hipSuccess) V(HIPRTC_SUCCESS)
S(hipMemAllocationProp) O(hipMemAllocationProp,type) O(hipMemAllocationProp,requestedHandleType) O(hipMemAllocationProp,location) O(hipMemAllocationProp,win32HandleMetaData) O(hipMemAllocationProp,allocFlags)
S(hipMemLocation) O(hipMemLocation,id)
S(hipMemAccessDesc) O(hipMemAccessDesc,location) O(hipMemAccessDesc,flags)
S(hipExternalMemoryHandleDesc) O(hipExternalMemoryHandleDesc,type) O(hipExternalMemoryHandleDesc,handle) O(hipExternalMemoryHandleDesc,size) O(hipExternalMemoryHandleDesc,flags)
S(hipExternalMemoryBufferDesc) O(hipExternalMemoryBufferDesc,offset) O(hipExternalMemoryBufferDesc,size) O(hipExternalMemoryBufferDesc,flags)
S(hipIpcMemHandle_t) S(hipIpcEventHandle_t) S(hipUUID) S(hipMemGenericAllocationHandle_t)
S(hipDeviceProp_tR0600) O(hipDeviceProp_tR0600,name) O(hipDeviceProp_tR0600,totalGlobalMem) O(hipDeviceProp_tR0600,hostRegisterSupported) O(hipDeviceProp_tR0600,hostRegisterReadOnlySupported) O(hipDeviceProp_tR0600,gcnArchName) O(hipDeviceProp_tR0600,canUseHostPointerForRegisteredMem) O(hipDeviceProp_tR0600,ipcEventSupported) O(hipDeviceProp_tR0600,multiProcessorCount) O(hipDeviceProp_tR0600,warpSize) O(hipDeviceProp_tR0600,pciBusID)
return 0;}
