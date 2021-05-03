/*
 * See the dyninst/COPYRIGHT file for copyright information.
 * 
 * We provide the Paradyn Tools (below described as "Paradyn")
 * on an AS IS basis, and do not warrant its validity or performance.
 * We reserve the right to update, modify, or discontinue this
 * software at any time.  We shall have no obligation to supply such
 * updates or modifications or any other form of support to you.
 * 
 * By your use of Paradyn, you understand and agree that we (or any
 * other person or entity with proprietary rights in Paradyn) are
 * under no obligation to provide either maintenance services,
 * update services, notices of latent defects, or correction of
 * defects for Paradyn.
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/************************************************************************
 * RTsignal.c: C-language signal handling code
************************************************************************/
#define _GNU_SOURCE
#include <stdio.h>
#include <signal.h>
#include <dlfcn.h>

#include "RTcommon.h"
#include "dyninstAPI_RT/h/dyninstAPI_RT.h"


DLLEXPORT int dyn_sigaction(int signum, const struct sigaction *act, struct sigaction *oldact) {
    if (signum != SIGTRAP) {
        return sigaction(signum, act, oldact);
    }
    else {
       return 0;
    }
}

DLLEXPORT dynsighandler_t dyn_signal(int signum, dynsighandler_t handler) {
    if (signum != SIGTRAP) {
        return signal(signum, handler);
    }
    else {
        return SIG_DFL;
    }
}


#undef signal
typedef dynsighandler_t (*signal_type) (int signum, dynsighandler_t handler);
static signal_type real_signal = NULL;
dynsighandler_t user_trap_handler = NULL;
DLLEXPORT dynsighandler_t signal(int signum, dynsighandler_t handler) {
    if (!real_signal) {
        real_signal = (signal_type)dlsym(RTLD_NEXT, "signal");
        if (real_signal == NULL) {
            fprintf(stderr, "Cannot find signal\n");
        }
    }
    if (signum == SIGILL) {
        dynsighandler_t old_handler = user_trap_handler;
        user_trap_handler = handler;
        return old_handler;
    } else {
        return real_signal(signum, handler);
    }
}

#undef __sysv_signal
static signal_type real_sysv_signal = NULL;
DLLEXPORT dynsighandler_t __sysv_signal(int signum, dynsighandler_t handler) {
    if (!real_sysv_signal) {
        real_sysv_signal = (signal_type)dlsym(RTLD_NEXT, "__sysv_signal");
        if (real_sysv_signal == NULL) {
            fprintf(stderr, "Cannot find __sysv_signal\n");
        }
    }
    if (signum == SIGILL) {
        dynsighandler_t old_handler = user_trap_handler;
        user_trap_handler = handler;
        return old_handler;
    } else {
        return real_sysv_signal(signum, handler);
    }
}

#if defined(cap_mutatee_traps)
typedef int (*sigaction_type) (int, const struct sigaction*, struct sigaction*);
struct sigaction user_sigill_info;
static sigaction_type real_sigaction = NULL;

DLLEXPORT int sigaction(int signum, const struct sigaction* act, struct sigaction* oldact) {
    if (signum == SIGILL) {
        if (oldact != NULL) {
            *oldact = user_sigill_info;
        }
        if (act != NULL) {
            user_sigill_info = *act;
        }    
        return 0;
    } else {
        return real_sigaction(signum, act, oldact);
    }
}

extern void dyninstTrapHandler(int sig, siginfo_t *info, void *context);

int DYNINSTinitializeTrapHandler()
{
    real_sigaction = (sigaction_type)dlsym(RTLD_NEXT, "sigaction");
    if (real_sigaction == NULL) {
        fprintf(stderr, "Cannot find sigaction\n");
    }

   int result;
   struct sigaction new_handler;

   new_handler.sa_sigaction = dyninstTrapHandler;
   //new_handler.sa_restorer = NULL; obsolete
   sigemptyset(&new_handler.sa_mask);
   new_handler.sa_flags = SA_SIGINFO | SA_NODEFER;
   
   result = real_sigaction(SIGILL, &new_handler, NULL);
   return (result == 0) ? 1 /*Success*/ : 0 /*Fail*/ ;
}

#endif
