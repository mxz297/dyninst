#ifndef _RTLINUX_H_
#define _RTLINUX_H_

#include <link.h>
#include "dyninstAPI_RT/h/dyninstAPI_RT.h"

#if defined(cap_binary_rewriter)

extern struct r_debug _r_debug;
DLLEXPORT struct r_debug _r_debug __attribute__ ((weak));

#if !defined(arch_x86_64) || defined(MUTATEE_32)
typedef Elf32_Dyn ElfX_Dyn;
#else
typedef Elf64_Dyn ElfX_Dyn;
#endif

#endif

#endif
