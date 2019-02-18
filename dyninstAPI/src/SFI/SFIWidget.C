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


#include "dyninstAPI/src/Relocation/Widgets/Widget.h"
#include "dyninstAPI/src/Relocation/CFG/RelocTarget.h"
#include "dyninstAPI/src/Relocation/Widgets/CFWidget.h" // CFPatch

// For our horribly horked memory effective address system
// Which I'm not fixing here. 
#include "dyninstAPI/h/BPatch_memoryAccess_NP.h"
#include "dyninstAPI/src/BPatch_memoryAccessAdapter.h"
#include "dyninstAPI/h/BPatch_addressSpace.h" // bpatch_address... you get the picture
#include "dyninstAPI/h/BPatch_point.h"
// Memory hackitude
#include "dyninstAPI/src/emit-x86.h"
#include "dyninstAPI/src/inst-x86.h"

#include "instructionAPI/h/Instruction.h"
#include "dyninstAPI/src/addressSpace.h"
#include "dyninstAPI/src/debug.h"
#include "dyninstAPI/src/registerSpace.h"
#include "dyninstAPI/src/mapped_object.h"
#include "dyninstAPI/src/Relocation/CodeBuffer.h"
#include "common/src/arch-x86.h"

#include "SFIWidget.h"

#include "dyninstAPI/src/RegisterConversion.h"

#include "boost/tuple/tuple.hpp"


using namespace Dyninst;
using namespace Relocation;
using namespace InstructionAPI;


SFIWidget::Ptr SFIWidget::create(Instruction insn,
				     Address addr,
				     int bits) {
  SFIWidget::Ptr ptr = SFIWidget::Ptr(new SFIWidget(insn, addr, bits));
  return ptr;
}


bool SFIWidget::generate(const codeGen &templ,
                           const RelocBlock *t,
                           CodeBuffer &buffer) 
{
    block = t->block();
    func = t->func();

    gen.allocate(128);
    gen.applyTemplate(templ);
    // We deal with 64-bit platforms
    BPatch_memoryAccessAdapter converter;
    BPatch_memoryAccess * mem = converter.convert(insn_, addr_, true);
    const BPatch_addrSpec_NP* spec = mem->getStartAddr();

    // We simply perform bit maksing to the registers 
    // used for addressing
    if (spec->getReg(0) >= 0) GenerateBitMask(spec->getReg(0), buffer);
    if (spec->getReg(1) >= 0) GenerateBitMask(spec->getReg(1), buffer);
    buffer.addPIC(gen, tracker());

    // We can reuse the origianal instruction
    buffer.addPIC(insn_.ptr(), insn_.size(), tracker()); 
    return true;
}

TrackerElement *SFIWidget::tracker() const {
    SFITracker *e = new SFITracker(addr_, block, func);
    return e;
}

void SFIWidget::GenerateBitMask(int reg, CodeBuffer &buffer) {
    unsigned mask = (1LL << bits_) - 1;
    unsigned opcode1 = 0x81;
    unsigned opcode2 = 0x4;
    gen.codeEmitter()->emitOpImm(opcode1, opcode2, reg, reg, mask, gen);
}
void SFIWidget::GenerateEffectiveAddress(int reg, CodeBuffer &buffer, const BPatch_addrSpec_NP* spec) {
    Emitter *e = gen.codeEmitter();
    Emitterx86 *x86_emitter = dynamic_cast<Emitterx86*>(e);
    x86_emitter->emitLEA(spec->getReg(0), spec->getReg(1), 
            spec->getScale(), spec->getImm(), reg, gen);
}
void SFIWidget::GenerateNewMemoryAccess(int reg, CodeBuffer &buffer, int size) {
    NS_x86::instruction ugly_insn(insn_.ptr(), true);
    bool ret;
    if (insn_.readsMemory()) {
        ret = insnCodeGen::generateMem(gen, ugly_insn, 0, 0, reg, Null_Register); 
    } else {
        ret = insnCodeGen::generateMem(gen, ugly_insn, 0, 0, Null_Register, reg); 
    }
    if (!ret) {
        fprintf(stderr, "Instruction re-generation failed : %s at %lx\n", insn_.format().c_str(), addr_);
    }
}
