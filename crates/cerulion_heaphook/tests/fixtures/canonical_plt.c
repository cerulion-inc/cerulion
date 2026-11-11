// SPDX-License-Identifier: AGPL-3.0-only
#define _GNU_SOURCE
#include <dlfcn.h>
#include <elf.h>
#include <link.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>

// Taking malloc's address in a non-PIE executable creates a canonical PLT
// import. dladdr1 must identify SHN_UNDEF so this cannot pass as a PIE probe.
int main(void) {
    void *(*volatile allocate)(size_t) = malloc;
    void *resolved = dlsym(RTLD_DEFAULT, "malloc");
    Dl_info info;
    const ElfW(Sym) *symbol = NULL;
    int identified = dladdr1(resolved, &info, (void **)&symbol, RTLD_DL_SYMENT);
    printf("canonical_plt=%d\n", identified && symbol && symbol->st_shndx == SHN_UNDEF);

    uint32_t (*status)(void) = dlsym(RTLD_DEFAULT, "cerulion_heaphook_status");
    if (!status) {
        puts("present=0");
        puts("PROBE_DONE");
        return 0;
    }
    int (*arm)(void *, void *) = dlsym(RTLD_DEFAULT, "cerulion_heaphook_arm_window");
    int (*disarm)(void) = dlsym(RTLD_DEFAULT, "cerulion_heaphook_disarm_window");
    int (*range_test)(const void *, size_t) =
        dlsym(RTLD_DEFAULT, "cerulion_heaphook_window_range_test");
    int (*retire)(void *) = dlsym(RTLD_DEFAULT, "cerulion_heaphook_retire_slot");
    if (!arm || !disarm || !range_test || !retire) {
        return 2;
    }
    unsigned char *slot = mmap(NULL, 4096, PROT_READ | PROT_WRITE,
                               MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (slot == MAP_FAILED) {
        return 3;
    }
    uint32_t won = status() & 1;
    int arm_rc = arm(slot, slot + 4096);
    if (arm_rc != 0) {
        return 4;
    }
    // No printing or bookkeeping in the armed region: this call goes through
    // the executable's canonical PLT. A hook win must return the slot's first
    // byte; a foreign allocator must leave the slot untouched.
    void *allocation = allocate(64);
    int nonnull = allocation != NULL;
    int at_base = allocation == slot;
    if (nonnull) {
        for (size_t i = 0; i < 64; ++i) {
            ((unsigned char *)allocation)[i] = 0x5a;
        }
    }
    int slot_written = 1;
    int slot_untouched = 1;
    for (size_t i = 0; i < 64; ++i) {
        slot_written &= slot[i] == 0x5a;
        slot_untouched &= slot[i] == 0;
    }
    int adopted = range_test(allocation, 64);
    int disarm_rc = disarm();
    free(allocation);
    int retire_rc = retire(slot);
    // Keep the mapping alive until exit, matching the quarantine's lifetime.
    puts("present=1");
    printf("status_won=%u\n", won);
    printf("allocation_nonnull=%d\n", nonnull);
    printf("at_slot_base=%d\n", at_base);
    printf("slot_written=%d\n", slot_written);
    printf("slot_untouched=%d\n", slot_untouched);
    printf("range_test=%d\n", adopted);
    printf("disarm_rc=%d\n", disarm_rc);
    printf("retire_rc=%d\n", retire_rc);
    puts("PROBE_DONE");
    return 0;
}
