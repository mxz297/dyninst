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

/*
 * emit-aarch64.C - ARMv8 code generators (emitters)
 */

/*
#include <assert.h>
#include <stdio.h>
#include "common/src/Types.h"
#include "dyninstAPI/src/codegen.h"
#include "dyninstAPI/src/function.h"
#include "dyninstAPI/src/inst-x86.h"
#include "dyninstAPI/src/debug.h"
#include "dyninstAPI/src/ast.h"
#include "dyninstAPI/h/BPatch.h"
#include "dyninstAPI/h/BPatch_memoryAccess_NP.h"

#include "dyninstAPI/src/dynProcess.h"

#include "dyninstAPI/src/binaryEdit.h"
#include "dyninstAPI/src/image.h"
// get_index...
#include "dyninstAPI/src/dynThread.h"
#include "ABI.h"
#include "liveness.h"
#include "RegisterConversion.h"
*/


#include "dyninstAPI/src/emit-aarch64.h"
#include "dyninstAPI/src/registerSpace.h"


void EmitterAARCH64::emitLoadConst(Register dest, Address imm, codeGen &gen)
{
    insnCodeGen::loadImmIntoReg<Address>(gen, dest, imm);
}


void EmitterAARCH64::emitLoad(Register dest, Address addr, int size, codeGen &gen)
{
    Register scratch = gen.rs()->getScratchRegister(gen);

    insnCodeGen::loadImmIntoReg<Address>(gen, scratch, addr);
    insnCodeGen::generateMemAccess32or64(gen, insnCodeGen::Load, dest, scratch, 0, false); 

    gen.rs()->freeRegister(scratch);
    gen.markRegDefined(dest);
}


void EmitterAARCH64::emitStore(Address addr, Register src, int size, codeGen &gen)
{
    Register scratch = gen.rs()->getScratchRegister(gen);

    insnCodeGen::loadImmIntoReg<Address>(gen, scratch, addr);
    insnCodeGen::generateMemAccess32or64(gen, insnCodeGen::Store, src, scratch, 0, false); 

    gen.rs()->freeRegister(scratch);
    gen.markRegDefined(src);
}


void EmitterAARCH64::emitOp(
        unsigned opcode, Register dest, Register src1, Register src2, codeGen &gen)
{
    // dest = src1 + src2
    if( opcode == plusOp )
        insnCodeGen::generateAddSubShifted(gen, insnCodeGen::Add, 0, 0, src1, src2, dest, true);

    // dest = src1 - src2
    else if( opcode == minusOp )
        insnCodeGen::generateAddSubShifted(gen, insnCodeGen::Sub, 0, 0, src1, src2, dest, true);
    
    // dest = src1 / src2
    else if( opcode == divOp )
        insnCodeGen::generateDiv(gen, src1, src2, dest, true);

    // dest = src1 * src2
    else if( opcode == timesOp )
        insnCodeGen::generateMul(gen, src1, src2, dest, true);

    // dest = src1 & src2
    else if( opcode == andOp )
        insnCodeGen::generateBitwiseOpShifted(gen, insnCodeGen::And, 0, src1, 0, src2, dest, true);  

    // dest = src1 | src2
    else if( opcode == orOp )
        insnCodeGen::generateBitwiseOpShifted(gen, insnCodeGen::Or, 0, src1, 0, src2, dest, true);  
}


void EmitterAARCH64::emitRelOp(
        unsigned opcode, Register dest, Register src1, Register src2, codeGen &gen)
{
    // CMP is an alias to SUBS, dest here has src1-src2, which it's not important
    insnCodeGen::generateAddSubShifted(gen, insnCodeGen::Sub, 0, 0, src2, src1, dest, true);

    // make dest = 1, assuming it is true, before conditional jump 
    insnCodeGen::loadImmIntoReg<Address>(gen, dest, 0x1);

    // insert conditional jump to skip an eventual dest=0
    insnCodeGen::generateConditionalBranch(gen, 4, opcode);

    // make dest = 0, in case it fails the branch 
    insnCodeGen::loadImmIntoReg<Address>(gen, dest, 0x0);
}





