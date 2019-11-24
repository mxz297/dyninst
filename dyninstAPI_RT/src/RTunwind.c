#include "libunwind.h"
#include <dlfcn.h>

#include "RTcommon.h"
#include "RTlinux.h"
#include "dyninstAPI_RT/h/dyninstAPI_RT.h"

#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

const char * unw_step_name = "_ULx86_64_step";

typedef int (*unw_step_fn_type) (unw_cursor_t *);
static unw_step_fn_type real_unw_step = NULL;

typedef struct {
    Address min;
    Address max;
    Address* entries;
} RAMappingTable;

typedef struct {
    unsigned long total;
    Address min;
    Address max;
    Address entries[][2]; 
} RAMappingInBinary;

RAMappingTable* ra_table = NULL;
int parsed = 0;

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
   memset(table->entries, 0, sizeof(Address) * (table->max - table->min + 1));
   for (unsigned long i = 0; i < header->total; ++i)
       table->entries[header->entries[i][0] - table->min] = header->entries[i][1];
   return table;
}

RAMappingTable* ExtractRAMapping() {
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

unw_word_t DyninstRATranslation(unw_word_t ip) {
    if (parsed == 0) {
        ra_table = ExtractRAMapping();
	parsed = 1;
	
    }
    if (ra_table == NULL) return ip;
    Address a = (Address)ip;
    if (ra_table->min <= a && a <= ra_table->max) {
        if (ra_table->entries[a - ra_table->min] != 0) {
            ip = (unw_word_t) ra_table->entries[a - ra_table->min];
        }
    }
    return ip;
}

DLLEXPORT int _ULx86_64_step(unw_cursor_t* cursor) {
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
    
    unw_word_t ip, new_ip;
    unw_get_reg(cursor, UNW_X86_64_RIP, &ip);
    new_ip = DyninstRATranslation(ip);
    if (new_ip != ip) {
        unw_set_reg(cursor, UNW_X86_64_RIP, new_ip);
    }
    return ret;
}


