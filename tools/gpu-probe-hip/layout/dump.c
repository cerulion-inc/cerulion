/* Prints HIP header values the probe pins in src/hipapi.rs. Build against a HIP include tree:
 *   cc -D__HIP_PLATFORM_AMD__ -I<rocm>/include -o dump dump.c && ./dump
 * (layout/fetch-headers.sh fetches the 6.2.0 and 7.0.0 trees used for layout/rocm-*.txt). */
#include <stdio.h>
#include <stddef.h>
#include <hip/hip_runtime_api.h>
#include <hip/hiprtc.h>
#define A(x) printf("attr %s %d\n", #x, (int)hipDeviceAttribute##x);
#define V(x) printf("val %s %lld\n", #x, (long long)(x));
#define S(t) printf("sizeof %s %zu\n", #t, sizeof(t));
#define O(t,f) printf("offsetof %s.%s %zu\n", #t, #f, offsetof(t,f));
int main(void){
A(CanMapHostMemory) A(CanUseHostPointerForRegisteredMem) A(Integrated) A(VirtualMemoryManagementSupported)
A(HostRegisterSupported) A(MultiprocessorCount) A(WarpSize) A(PageableMemoryAccess)
A(PageableMemoryAccessUsesHostPageTables) A(ConcurrentManagedAccess) A(UnifiedAddressing)
A(ClockRate) A(MemoryClockRate) A(MemoryBusWidth) A(PciBusId) A(PciDeviceId) A(PciDomainID)
A(ManagedMemory) A(DirectManagedMemAccessFromHost) A(HostNativeAtomicSupported) A(IsMultiGpuBoard)
A(MaxThreadsPerBlock) A(ComputeCapabilityMajor) A(ComputeCapabilityMinor) A(L2CacheSize)
A(CanUseStreamWaitValue) A(GlobalL1CacheSupported) A(AsicRevision) A(MemoryPoolsSupported)

return 0;}
