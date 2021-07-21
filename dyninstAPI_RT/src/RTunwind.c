#define _GNU_SOURCE
#include "libunwind.h"
#include <dlfcn.h>

#include "RTcommon.h"
#include "RTlinux.h"
#include "dyninstAPI_RT/h/dyninstAPI_RT.h"

#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// libunwind cursor is an opaque structure.
// Based on its source code, its fourth qword field
// is the current PC
#define IP_OFFSET_IN_CURSOR 3


#if defined(arch_x86_64)
const char * unw_step_name = "_ULx86_64_step";
#define UNW_FUNC_NAME _ULx86_64_step

#elif defined(arch_power)
const char * unw_step_name = "_ULppc64_step";
#define UNW_FUNC_NAME _ULppc64_step

#elif defined(arch_aarch64)
const char * unw_step_name = "_ULaarch64_step";
#define UNW_FUNC_NAME _ULaarch64_step
#endif

typedef int (*unw_step_fn_type) (unw_cursor_t *);
static unw_step_fn_type real_unw_step = NULL;

typedef struct {
    Address loadAddr;
    Address min;
    Address max;
    Address* entries;
} RAMappingTable;

RAMappingTable* ra_table = NULL;
DLLEXPORT int DYNINSTparsedMode = 0;
DLLEXPORT int DYNINSTRunningProcessMode = 0;

typedef struct {
    unsigned long total;
    Address min;
    Address max;
    Address entries[][2];
} RAMappingInBinary;

DLLEXPORT RAMappingInBinary *dyninstRTTable = NULL;

RAMappingTable* BuildProcessTable() {
  if(dyninstRTTable == NULL)
    return NULL;

  if(ra_table) {
    free(ra_table->entries);
    free(ra_table);
    ra_table = NULL;
  }
  RAMappingTable* table = (RAMappingTable*) malloc(sizeof(RAMappingTable));
  table->min = dyninstRTTable->min;
  table->max = dyninstRTTable->max;
  table->entries = (Address*) malloc( sizeof(Address) * (table->max - table->min + 1) );
  table->loadAddr = 0;
  memset(table->entries, 0, sizeof(Address) * (table->max - table->min + 1));
  for (unsigned long i = 0; i < dyninstRTTable->total; ++i) {
    table->entries[dyninstRTTable->entries[i][0] - table->min] = dyninstRTTable->entries[i][1];
  }
  return table;
}

RAMappingTable* ParseAModule(struct link_map *l) {
   ElfX_Dyn *dynamic_ptr;
   dynamic_ptr = (ElfX_Dyn *) l->l_ld;
   if (!dynamic_ptr)
      return NULL;

   assert(sizeof(dynamic_ptr->d_un.d_ptr) == sizeof(void *));
   for (; dynamic_ptr->d_tag != DT_NULL && dynamic_ptr->d_tag != DT_DYNINST_RAMAP; dynamic_ptr++);
   if (dynamic_ptr->d_tag == DT_NULL) {
      return NULL;
   }

   RAMappingInBinary* header = (RAMappingInBinary*) (dynamic_ptr->d_un.d_val + l->l_addr);

   RAMappingTable* table = (RAMappingTable*) malloc(sizeof(RAMappingTable));
   table->min = header->min;
   table->max = header->max;
   table->entries = (Address*) malloc( sizeof(Address) * (table->max - table->min + 1) );
   table->loadAddr = l->l_addr;
   memset(table->entries, 0, sizeof(Address) * (table->max - table->min + 1));
   for (unsigned long i = 0; i < header->total; ++i) {
        table->entries[header->entries[i][0] - table->min] = header->entries[i][1];
     }
   return table;
}

RAMappingTable* ExtractRAMapping() {
   rtdebug_printf("Enter ExtractRAMapping\n");
   RAMappingTable* table;
   struct link_map *l_current;
   l_current = _r_debug.r_map;
   while (l_current) {
      table = ParseAModule(l_current);
      if (table != NULL) return table;
      l_current = l_current->l_next;
   }
   return NULL;
}

DLLEXPORT Address DyninstRATranslation(Address ip) {
    if (DYNINSTparsedMode == 0) {
        DYNINSTparsedMode = 1;
        if(DYNINSTRunningProcessMode)
            ra_table = BuildProcessTable();
        else
            ra_table = ExtractRAMapping();
    }
    if (ra_table == NULL) return ip;
    Address a = ip;
    Address newip = 0;
    if (a < ra_table->loadAddr) return ip;
    a -= ra_table->loadAddr;
    if (ra_table->min <= a && a <= ra_table->max) {
        if (ra_table->entries[a - ra_table->min] != 0) {
            newip = (Address) ra_table->entries[a - ra_table->min];
        }
    }
    if (newip == 0) {        
        // Go runtime may substract the RA by 1 to lookup which 
        // function the RA belongs.
        if (ra_table->min <= a + 1 && a + 1 <= ra_table->max) {
            if (ra_table->entries[a + 1 - ra_table->min] != 0) {
                newip = (Address) ra_table->entries[a + 1 - ra_table->min];
                newip -= 1;
            }
        }
    }
    rtdebug_printf("input ip %lx, found ip %lx, loadd addr %lx , calculated ip %lx\n",
            ip, newip, ra_table->loadAddr, newip + ra_table->loadAddr);
    if (newip == 0) return ip;
    return newip + ra_table->loadAddr;
}

DLLEXPORT int UNW_FUNC_NAME(unw_cursor_t* cursor) {
    if (!real_unw_step) {
        void* handle = dlopen("libunwind.so", RTLD_LAZY);
        if (handle == NULL) {
            fprintf(stderr, "Cannot find libunwind handle\n");
        }
        real_unw_step = (unw_step_fn_type)dlsym(handle, unw_step_name);

        if (real_unw_step == NULL) {
            fprintf(stderr, "Cannot find %s\n", unw_step_name);
        }
    }

    int ret = real_unw_step(cursor);

    // The current PC may not be in a reg,
    // so we cannot just call unw_get_reg.
    unw_word_t* typed_cursor = (unw_word_t*) cursor;
    unw_word_t ip, new_ip;
    ip = typed_cursor[IP_OFFSET_IN_CURSOR];
    new_ip = DyninstRATranslation(ip);
    if (new_ip != ip) {
        typed_cursor[IP_OFFSET_IN_CURSOR] = new_ip;
    }
    return ret;
}


