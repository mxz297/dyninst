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



#include "SFITransformer.h"
#include "SFIWidget.h"
#include "dyninstAPI/src/debug.h"
#include "dyninstAPI/src/Relocation/Widgets/Widget.h"
#include "dyninstAPI/src/Relocation/Widgets/InsnWidget.h"
#include "dyninstAPI/src/instPoint.h" // Memory insn modelling requirement.
#include "dyninstAPI/src/mapped_object.h"
#include <iomanip>
#include "dyninstAPI/src/Relocation/CFG/RelocBlock.h"

using namespace std;
using namespace Dyninst;
using namespace Relocation;
using namespace InstructionAPI;


// Add a bit mask to all speificed memory access type
bool SFITransformer::process(RelocBlock *rblock, RelocGraph *rgraph)
{
  if (!(rblock->block())) return true;
  
  // AssignmentConverter is written in terms of parse_func,
  // so translate
  func_instance *func = rblock->func();

  WidgetList &elements = rblock->elements();
  
  for (WidgetList::iterator e_iter = elements.begin();
       e_iter != elements.end(); ++e_iter) {
    // If we're not an instruction then skip...
     InsnWidget::Ptr reloc = boost::dynamic_pointer_cast<Relocation::InsnWidget>(*e_iter);
     if (!reloc) continue;

    relocation_cerr << "Memory emulation considering addr " << hex << reloc->addr() << dec << endl;

    if (!isSensitive(reloc)) {
        relocation_cerr << "\t Not sensitive, skipping" << endl;
        continue;
    }

    Widget::Ptr replacement = SFIWidget::create(reloc->insn(), reloc->addr(), maskBits);
    if (!replacement) return false;
    
    (*e_iter).swap(replacement);
  }
  return true;
}

bool SFITransformer::isSensitive(InsnWidget::Ptr reloc) {
    InstructionAPI::Instruction i = reloc->insn(); 
    entryID id = i.getOperation().getID();
    if (id == e_push) return false;
    if (i.readsMemory() && (mode == READ || mode == BOTH)) return true;
    if (i.writesMemory() && (mode == WRITE || mode == BOTH)) return true;
    return false;
}


