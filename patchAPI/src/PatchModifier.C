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

#include "PatchModifier.h"
#include "PatchCFG.h"
#include "PatchMgr.h"
#include "CFGModifier.h"
#include "debug_patch.h"

#include <queue>
#include <set>
#include <vector>

#include "asmjit/asmjit.h"

using namespace std;
using namespace Dyninst;
using namespace PatchAPI;

/* If target is NULL, user is requesting a redirect to the sink block */
bool PatchModifier::redirect(PatchEdge *edge, PatchBlock *target) {
   // Do we want edges to only be in the same object? I don't think so.
   // However, same address space is probably a good idea ;)
   if (target && edge->src()->obj()->addrSpace() != target->obj()->addrSpace()) return false;

   // Also, changing catch edges would be awesome! ... but we can't.
   if (edge->type() == ParseAPI::CATCH ||
       edge->type() == ParseAPI::RET) return false;

   edge->trg()->removeSourceEdge(edge);
   edge->trg_ = target;
   target->addSourceEdge(edge);
   return true;
}



PatchBlock *PatchModifier::split(PatchBlock *block, Address addr, bool trust, Address newlast) {
   if (!trust) {
      if (!block->getInsn(addr).isValid()) return NULL;
      newlast = -1; // If we don't trust the address, don't
      // trust the new last instruction addr.
   }

   if (newlast == (Address) -1) {
      // Determine via disassembly
      PatchBlock::Insns insns;
      block->getInsns(insns);
      PatchBlock::Insns::iterator iter = insns.find(addr);
      if (iter == insns.begin()) return NULL;
      --iter;
      newlast = iter->first;
   }

   Address offset = addr - block->obj()->codeBase();

   ParseAPI::Block *split_int = ParseAPI::CFGModifier::split(block->block(), offset, true, newlast);
   if (!split_int) return NULL;

   // We want to return the new block so that folks have a handle;
   // look it up.
   PatchBlock *split = block->obj()->getBlock(split_int);
   block->markModified();
   split->markModified();

   return split;
}

InsertedCode::Ptr PatchModifier::insert(PatchObject *obj, void *start, unsigned size, Address base) {
   ParseAPI::CodeObject *co = obj->co();

   ParseAPI::InsertedRegion *newRegion = ParseAPI::CFGModifier::insert(co, base, start, size);

   if (!newRegion) return InsertedCode::Ptr();

   ParseAPI::Block *_e = co->findBlockByEntry(newRegion, base);
   if (!_e) return InsertedCode::Ptr();

   // Let's get handles to the various bits'n'pieces
   PatchBlock *entry = obj->getBlock(_e);
   if (!entry) return InsertedCode::Ptr();

   InsertedCode::Ptr ret = InsertedCode::Ptr(new InsertedCode());
   ret->entry_ = entry;

   std::queue<PatchBlock *> worklist; worklist.push(entry);
   while (!worklist.empty()) {
      PatchBlock *cur = worklist.front(); worklist.pop();
      assert(cur);

      if (ret->blocks_.find(cur) != ret->blocks_.end()) continue;
      ret->blocks_.insert(cur);

      for (PatchBlock::edgelist::const_iterator iter = cur->targets().begin();
           iter != cur->targets().end(); ++iter) {
         PatchEdge *e = (*iter);
         if (e->sinkEdge()) {

            ret->exits_.push_back(e);
            continue;
         }
         ParseAPI::Block *t_ = e->edge()->trg(); assert(t_);
         if (t_->region() != newRegion) {
            ret->exits_.push_back(e);
            continue;
         }
         PatchBlock *t = obj->getBlock(t_);
         worklist.push(t);
      }
   }

   return ret;
}

InsertedCode::Ptr PatchModifier::insert(PatchObject *obj, void *start, unsigned size) {
   ParseAPI::CodeObject *co = obj->co();
   Address base = co->getFreeAddr();
   return insert(obj, start, size, base);
}


InsertedCode::Ptr PatchModifier::insert(PatchObject *obj, SnippetPtr snip, Point *p) {
   if (!snip) return InsertedCode::Ptr();

   ParseAPI::CodeObject *co = obj->co();
   Address base = co->getFreeAddr();

   Buffer buf(base, 1024);
   if (!snip->generate(p, buf)) return InsertedCode::Ptr();

   return insert(obj, buf.start_ptr(), buf.size(), base);
}

bool PatchModifier::remove(vector<PatchBlock *> &blocks, bool force)
{
    //vector<PatchFunction*> funcs;
    //block->getFunctions(std::back_inserter(funcs));
   vector<ParseAPI::Block*> parseBs;
   for (unsigned bidx=0; bidx < blocks.size(); bidx++) {
      parseBs.push_back(blocks[bidx]->block());
   }
   bool success = ParseAPI::CFGModifier::remove(parseBs, force);

    //// DEBUG  // this check came too early, we haven't updated other block points
                // to match changes to the underlying bytes after an overwrite
    //if (success) {
    //    for (unsigned int fidx = 0; fidx < funcs.size(); fidx++) {
    //        assert(funcs[fidx]->consistency());
    //    }
    //}
    return success;
}

bool PatchModifier::remove(PatchFunction *func)
{
  //    PatchObject *obj = func->obj();
  //bool success =
       return ParseAPI::CFGModifier::remove(func->function());

    // DEBUG
    //if (success) {
    //    assert(obj);
    //}
    //return success;
}

bool PatchModifier::addBlockToFunction(PatchFunction *f, PatchBlock* b) {
    // First trigger block filling
    f->blocks();
    // Then add the block
    //f->addBlock(b);
    f->all_blocks_.insert(b);
    return true;
}

static PatchEdge* getFTEdge(PatchBlock* b) {
   for (auto e : b->targets()) {
      if (e->type() == ParseAPI::FALLTHROUGH) return e;
   }
   return nullptr;
}

static bool RedirectEdgeList(
   std::vector<PatchEdge*> &redges,
   PatchBlock* tar
) {
   for (auto e : redges) {
      if (!PatchModifier::redirect(e, tar)) {
         patch_printf("\t\t\tfail to redirect edge from [%lx, %lx] to the new target, edge type %d\n",
            e->src()->start(), e->src()->end(), e->type());
         return false;
      }
   }
   return true;
}

static bool SplitAndGetEdges(
   PatchBlock* b,
   std::vector<PatchEdge*> &redges
) {
   PatchBlock::Insns insns;
   b->getInsns(insns);
   if (insns.size() > 1) {
      patch_printf("\t\tblock has more than one instruction\n");
      // We first remove the return instruction
      if (PatchModifier::split(b, b->last(), true, b->last()) == nullptr) {
         patch_printf("\t\t\tfail to split the block at [%lx, %lx)\n", b->start(), b->end());
         return false;
      }
      // We then redirect the fallthrough edge from
      // then-return block to the call-ft block
      PatchEdge * ftedge = getFTEdge(b);
      redges.emplace_back(ftedge);
      if (ftedge == nullptr) {
         patch_printf("\t\t\tno fallthrough edge after spliting block [%lx, %lx)\n", b->start(), b->end());
         return false;
      }
   } else {
      patch_printf("\t\tblock has only one instruction\n");
      for (auto e : b->sources()) {
         if (e->interproc()) continue;
         redges.emplace_back(e);
      }
   }
   return true;
}

static PatchBlock* GetCallFTBlock(PatchBlock* b) {
   PatchBlock* call_ft_block = nullptr;
   for (auto e : b->targets()) {
      if (e->type() == ParseAPI::CALL_FT) {
         call_ft_block = e->trg();
         break;
      }
   }
   return call_ft_block;
}

static std::set<PatchFunction*> functionSplit;

bool SplitFunctionRetBlocks(PatchFunction* f) {
   if (functionSplit.find(f) != functionSplit.end()) {
      return true;
   }
   functionSplit.insert(f);
   std::vector<PatchBlock*> retBlocks;
   for (auto b : f->blocks()) {
      bool isRetBlock = false;
      for (auto e : b->targets()) {
         if (e->type() == ParseAPI::RET) {
            isRetBlock = true;
            break;
         }
      }
      if (isRetBlock) retBlocks.emplace_back(b);
   }
   for (auto b: retBlocks) {
      PatchBlock::Insns insns;
      b->getInsns(insns);
      if (insns.size() == 1) continue;
      if (PatchModifier::split(b, b->last(), true, b->last()) == nullptr) {
         patch_printf("\t\t\tfail to split return block at [%lx, %lx)\n", b->start(), b->end());
         return false;
      }
   }
   return true;
}

class AssemblerHolder {
   public:

   AssemblerHolder();
   ~AssemblerHolder();
   asmjit::x86::Assembler* GetAssembler() { return assembler_; }
   asmjit::StringLogger* GetStringLogger() { return logger_; }
   asmjit::CodeHolder* GetCode() { return code_; }
   bool CopyToDyninstBuffer(Dyninst::Buffer &buf);

   private:

   asmjit::JitRuntime* rt_;
   asmjit::CodeHolder* code_;
   asmjit::x86::Assembler* assembler_;
   asmjit::StringLogger* logger_;
};

// Error handler that just prints the error and lets AsmJit ignore it.
class PrintErrorHandler : public asmjit::ErrorHandler {
   public:
   // Return `true` to set last error to `err`, return `false` to do nothing.
   void handleError(asmjit::Error err, const char* message, asmjit::BaseEmitter* origin) override {
      std::cerr << "ERROR : " << message;
   }
};

AssemblerHolder::AssemblerHolder() {
   rt_ = new asmjit::JitRuntime();

   code_ = new asmjit::CodeHolder;
   code_->init(rt_->codeInfo());
   code_->setErrorHandler(new PrintErrorHandler());

   logger_ = new asmjit::StringLogger();
   code_->setLogger(logger_);

   assembler_ = new asmjit::x86::Assembler(code_);
}

AssemblerHolder::~AssemblerHolder() {
   delete assembler_;
   delete code_;
   delete rt_;
}

bool AssemblerHolder::CopyToDyninstBuffer(Dyninst::Buffer &buf) {
   size_t size = GetCode()->codeSize();
   char* temp_buf = (char*)malloc(size);

   GetCode()->relocateToBase((uint64_t)temp_buf);

   size = GetCode()->codeSize();
   GetCode()->copyFlattenedData(temp_buf, size, asmjit::CodeHolder::kCopyWithPadding);

   buf.copy(temp_buf, size);
   free(temp_buf);
   return true;
}

static int version;

#include "Snippet.h"
class AdjustSPSnippet : public Dyninst::PatchAPI::Snippet {
   public:
   explicit AdjustSPSnippet(int adj): adjustment(adj) {}
   bool generate(Dyninst::PatchAPI::Point* pt, Dyninst::Buffer& buf) override {
      AssemblerHolder ah;
      asmjit::x86::Assembler* a = ah.GetAssembler();
      a->lea(asmjit::x86::rsp, asmjit::x86::ptr(asmjit::x86::rsp, adjustment, 8));
      return ah.CopyToDyninstBuffer(buf);
   }
   private:
   int adjustment;
};

#include "PatchLabel.h"

class EmulatedReturnAddressSnippet : public Dyninst::PatchAPI::Snippet {
   public:
   explicit EmulatedReturnAddressSnippet(PatchFunction* func, PatchBlock* targetBlock):
               f(func), b(targetBlock) {}

   bool generate(Dyninst::PatchAPI::Point* pt, Dyninst::Buffer& buf) override {
      AssemblerHolder ah;
      asmjit::x86::Assembler* a = ah.GetAssembler();
      a->mov(asmjit::x86::ptr(asmjit::x86::rsp, 0, 8), 0);
      ah.CopyToDyninstBuffer(buf);
      Dyninst::PatchAPI::PatchLabel::generateALabel(f, b, buf.curAddr() - 4);
      return true;
   }

   private:
   PatchFunction *f;
   PatchBlock* b;
};

static bool InlineImpl(
   CFGMaker * cfgMaker,
   PatchMgr::Ptr patcher,
   PatchFunction * caller,
   std::vector<PatchEdge*>& inEdges,
   Address calleeAddr,
   PatchBlock* returnTarget
) {
   version += 1;
   // 1. Find the callee function
   PatchObject* po = caller->obj();
   PatchFunction* callee = po->getFunc(po->co()->findFuncByEntry(caller->entry()->block()->region(), calleeAddr));
   // callee is NULL if this is not a call block or an indirect call
   if (callee == nullptr) {
      patch_printf("\tCannot find callee at %lx\n", calleeAddr);
      return false;
   }
   patch_printf("\tget callee %s at %lx\n", callee->name().c_str(), callee->addr());

   if (!SplitFunctionRetBlocks(callee)) {
      patch_printf("\tfail to spilt return blocks in %s at %lx\n", callee->name().c_str(), callee->addr());
      return false;
   }

   // 2. Clone all blocks in the callee and reconnect edges
   std::map<PatchBlock*, PatchBlock*> cloneBlockMap;
   std::vector<PatchBlock*> newBlocks;
   for (auto b : callee->blocks()) {
      PatchBlock* cloneB = cfgMaker->cloneBlock(b, b->object());
      cloneB->setCloneVersion(version);
      cloneBlockMap[b] = cloneB;
      newBlocks.emplace_back(cloneB);
      PatchModifier::addBlockToFunction(caller, cloneB);
   }

   // 3. Copy the jump table data from calee to caller
   // Jump table data copy should be done before redirecting edges
   for (auto &it : cloneBlockMap) {
      PatchBlock* origB = it.first;
      PatchBlock* newB = it.second;
      const auto jit = callee->getJumpTableMap().find(origB);
      if (jit == callee->getJumpTableMap().end()) continue;
      patch_printf("\tcopy jump table for block [%lx, %lx)\n", origB->start(), origB->end());
      PatchFunction::PatchJumpTableInstance pjti = jit->second;

      // Build a map from target address to PatchEdge
      // We can use this map to match cloned edge with original edge
      dyn_hash_map<Address, PatchEdge*> newEdges;
      for (auto e : newB->targets()) {
         if (e->sinkEdge() || e->type() != ParseAPI::INDIRECT) continue;
         newEdges[e->trg()->start()] = e;
      }

      // Update edges in jump table data to cloned edges
      for (auto& e : pjti.tableEntryEdges) {
         Address trg = e->trg()->start();
         auto edgeIter = newEdges.find(trg);
         assert(edgeIter != newEdges.end());
         e = edgeIter->second;
      }
      patch_printf("\t\n");

      caller->addJumpTableInstance(newB, pjti);
   }

   // 4. The edges of the cloned blocks are still from/to original blocks
   // Now we redirect all edges
   std::vector<PatchBlock*> newRetBlocks;
   for (auto b : newBlocks) {
      bool isRetBlock = false;
      for (auto e : b->targets()) {
         if (e->type() == ParseAPI::RET) isRetBlock = true;
         if (e->sinkEdge() || e->interproc()) continue;
         if (e->type() == ParseAPI::CATCH) continue;
         auto it = cloneBlockMap.find(e->trg());
         if (it == cloneBlockMap.end()) continue;
         PatchBlock* newTarget = it->second;
         if (!PatchModifier::redirect(e, newTarget)) {
            patch_printf("\tfail to redirect edge [%lx, %lx)->[%lx, %lx), edge type %d\n",
               b->start(), b->end(), e->trg()->start(), e->trg()->end(), e->type());
            return false;
         }
      }
      if (isRetBlock) newRetBlocks.emplace_back(b);
   }

   // 5. Remove return instruction from return blocks
   for (auto b: newRetBlocks) {
      patch_printf("\tsplit return block and redirect edge for [%lx, %lx)\n", b->start(), b->end());
      std::vector<PatchEdge*> redges;
      if (!SplitAndGetEdges(b, redges)) {
         patch_printf("\t\tfailed\n");
         return false;
      }
      if (!RedirectEdgeList(redges, returnTarget)) {
         patch_printf("\t\tfaile to redirect edges");
         return false;
      }
      // Move up SP to mimic the effect of a return instruction to stack
      for (auto e: redges) {
         if (e->interproc()) continue;
         patch_printf("\t\tgenerate sp move up for source edge from [%lx, %lx), type %d\n", e->src()->start(), e->src()->end(), e->type());
         auto p2 = patcher->findPoint(Location::EdgeInstance(caller, e), Point::EdgeDuring, true);
         Snippet::Ptr moveSPUp = AdjustSPSnippet::create(new AdjustSPSnippet(8));
         p2->pushBack(moveSPUp);
      }
   }
   
   PatchBlock* newEntry = cloneBlockMap[callee->entry()];
   newEntry->setAlignHint(true);
   bool newEntryHasIntraEdge = false;
   for (auto e: newEntry->sources()) {
      newEntryHasIntraEdge = true;
   }

   // 6. Redirect edges to cloned function entry
   if (!RedirectEdgeList(inEdges, newEntry)) {
      patch_printf("\t\tFailed to redirect edges to cloned entry\n");
      return false;
   }

   // 7. Move down SP to mimic the effect of a call instruction to stack
   if (newEntryHasIntraEdge) {
      for (auto e : inEdges) {
         if (e->interproc()) continue;
         auto p1 = patcher->findPoint(Location::EdgeInstance(caller, e), Point::EdgeDuring, true);
         Snippet::Ptr moveSPDown = AdjustSPSnippet::create(new AdjustSPSnippet(-8));
         p1->pushBack(moveSPDown);
      }
   } else {
      auto p1 = patcher->findPoint(Location::BlockInstance(caller, cloneBlockMap[callee->entry()]), Point::BlockEntry, true);
      Snippet::Ptr moveSPDown = AdjustSPSnippet::create(new AdjustSPSnippet(-8));
      p1->pushBack(moveSPDown);
   }

   // 8. Insert snippets to push orignial return address for tail calls
   // Cannot handle conditional tail calls
   for (auto b : newBlocks) {
      bool hasTailCall = false;
      for (auto e : b->targets()) {
         if (!e->interproc()) continue;
         if (e->type() == ParseAPI::DIRECT ||
             e->type() == ParseAPI::INDIRECT) {
                hasTailCall = true;
                break;
         }
      }
      if (!hasTailCall) continue;

      auto p = patcher->findPoint(Location::InstructionInstance(caller, b, b->last()), Point::PreInsn, true);
      Snippet::Ptr emulatedRA = EmulatedReturnAddressSnippet::create(new EmulatedReturnAddressSnippet(caller, returnTarget));
      p->pushBack(emulatedRA);
   }
   return true;
}

bool PatchModifier::inlineDirectCall(
   CFGMaker* cfgMaker,
   PatchMgr::Ptr patcher,
   PatchFunction* caller,
   PatchBlock* cb,
   Address callee
) {
   patch_printf("Enter PatchModifier::inlineDirectCall: caller %s at %lx, call block [%lx, %lx), callee %lx\n",
     caller->name().c_str(), caller->addr(), cb->start(), cb->end(), callee);

   PatchBlock* call_ft_block = GetCallFTBlock(cb);
   if (call_ft_block == nullptr) {
      patch_printf("\tcannot find call fallthrough block\n");
      return false;
   }

   std::vector<PatchEdge*> redges;
   patch_printf("\tsplit call block and redirect edge for [%lx, %lx)\n", cb->start(), cb->end());
   if (!SplitAndGetEdges(cb, redges)) {
      patch_printf("\t\tfailed\n");
      return false;
   }
   return InlineImpl(cfgMaker, patcher, caller, redges, callee, call_ft_block);
}

static std::map<Dyninst::MachRegister, asmjit::x86::Gp> dyninstToasmjitRegMap = {
    {x86_64::rax, asmjit::x86::rax}, {x86_64::rbx, asmjit::x86::rbx}, {x86_64::rcx, asmjit::x86::rcx},
    {x86_64::rdx, asmjit::x86::rdx}, {x86_64::rsp, asmjit::x86::rsp}, {x86_64::rbp, asmjit::x86::rbp},
    {x86_64::rsi, asmjit::x86::rsi}, {x86_64::rdi, asmjit::x86::rdi}, {x86_64::r8, asmjit::x86::r8},
    {x86_64::r9, asmjit::x86::r9},   {x86_64::r10, asmjit::x86::r10}, {x86_64::r11, asmjit::x86::r11},
    {x86_64::r12, asmjit::x86::r12}, {x86_64::r13, asmjit::x86::r13}, {x86_64::r14, asmjit::x86::r14},
    {x86_64::r15, asmjit::x86::r15}
};

#include "Visitor.h"
#include "Expression.h"

class IndirectCallInlineSnippet : public Dyninst::PatchAPI::Snippet {
   public:
   explicit IndirectCallInlineSnippet(const InstructionAPI::Instruction &insn, std::vector<Address> &addresses):
      i(insn), addrs(addresses) {}
   bool generate(Dyninst::PatchAPI::Point* pt, Dyninst::Buffer& buf) override {
      AssemblerHolder ah;
      asmjit::x86::Assembler* a = ah.GetAssembler();
      asmjit::x86::Gp reg;

      if (i.readsMemory()) {
         reg = asmjit::x86::rax;
         a->mov(asmjit::x86::rax, GetCallTargetMemory());
      } else {
         reg = GetCallTargetRegister();
      }
      asmjit::Label indCall = a->newLabel();
      if (addrs.size() > 5) {
         sort(addrs.begin(), addrs.end());
         asmjit::Label start = a->newLabel();
         EmitBinarySearch(a, reg, 0, addrs.size() - 1, start, indCall);
      } else {
         for (auto addr : addrs) {
            a->cmp(reg, addr);
            // Provide a large displacment to ensure the
            // target is outside the code region
            a->je(2048);
         }
      }
      a->bind(indCall);
      a->call(reg);
      return ah.CopyToDyninstBuffer(buf);
   }
   private:

   const InstructionAPI::Instruction& i;
   std::vector<Address>& addrs;
   std::vector<asmjit::Label> labels;

   class MemoryEffectiveAddressVisitor : public Dyninst::InstructionAPI::Visitor {
      public:
      virtual void visit(Dyninst::InstructionAPI::BinaryFunction* b) {
         if(b->isAdd()) {
            if (!immStack.empty()) {
               a.setOffset(getLastImm());
            }
            if (!regStack.empty()) {
               a.setBase(getLastReg());
            }
            if (!regStack.empty()) {
               a.setIndex(getLastReg(), 0);
            }
         } else if(b->isMultiply()) {
            assert(!immStack.empty());
            assert(!regStack.empty());
            long val = getLastImm();
            int scale = 0;
            switch (val) {
               case 8: scale = 3; break;
               case 4: scale = 2; break;
               case 2: scale = 1; break;
               case 1: scale = 0; break;
               default: assert(0);
            }
            a.setIndex(getLastReg(), scale);
         } else {
            assert(0);
         }
      }
      virtual void visit(Dyninst::InstructionAPI::Dereference*) {}
      virtual void visit(Dyninst::InstructionAPI::Immediate* i) { immStack.emplace_back(i); }
      virtual void visit(Dyninst::InstructionAPI::RegisterAST* r) {
         assert(!r->getID().isPC());
         regStack.emplace_back(r);
      }
      MemoryEffectiveAddressVisitor(asmjit::x86::Mem& access) : a(access) {}
      void finalize() {
         if (!regStack.empty()) {
            a.setBase(getLastReg());
         }
      }
      private:
      asmjit::x86::Mem &a;

      std::vector<Dyninst::InstructionAPI::Immediate*> immStack;
      std::vector<Dyninst::InstructionAPI::RegisterAST*> regStack;

      asmjit::x86::Gp getLastReg() {
         Dyninst::InstructionAPI::RegisterAST* r = regStack.back();
         regStack.pop_back();
         auto rit = dyninstToasmjitRegMap.find(r->getID().getBaseRegister());
         assert(rit != dyninstToasmjitRegMap.end());
         return rit->second;
      }

      long getLastImm() {
         Dyninst::InstructionAPI::Immediate* imm = immStack.back();
         immStack.pop_back();
         return imm->eval().convert<long>();
      }
   };

   asmjit::x86::Gp GetCallTargetRegister() {
      std::vector<Dyninst::InstructionAPI::Operand> ops;
      i.getOperands(ops);
      auto o = ops[0];

      std::set<InstructionAPI::RegisterAST::Ptr> regsRead;
      o.getReadSet(regsRead);
      assert(regsRead.size() == 1);
      for (auto r : regsRead) {
         MachRegister reg = r->getID().getBaseRegister();
         auto it = dyninstToasmjitRegMap.find(reg);
         assert(it != dyninstToasmjitRegMap.end());
         return it->second;
      }
      return asmjit::x86::Gp();
   }

   asmjit::x86::Mem GetCallTargetMemory() {
      std::set<Dyninst::InstructionAPI::Expression::Ptr> memAccess;
      i.getMemoryReadOperands(memAccess);
      assert(memAccess.size() == 1);
      for (auto m : memAccess) {
         asmjit::x86::Mem a;
         MemoryEffectiveAddressVisitor meav(a);
         m->apply(&meav);
         meav.finalize();
         return a;
      }
      return asmjit::x86::Mem();
   }

   void EmitBinarySearch(
      asmjit::x86::Assembler* a,
      asmjit::x86::Gp & reg,
      size_t l,
      size_t r,
      asmjit::Label &cur,
      asmjit::Label &indCall
   ) {
      a->bind(cur);
      if (r - l < 3) {
         for (size_t i = l; i <= r; i++) {
            a->cmp(reg, addrs[i]);
            a->je(2048);
         }
         a->jmp(indCall);
         return;
      }
      size_t m = (l + r) / 2;
      a->cmp(reg, addrs[m]);
      // Provide a large displacment to ensure the
      // target is outside the code region
      a->je(2048);
      asmjit::Label above = a->newLabel();
      a->ja(above);
      asmjit::Label below = a->newLabel();
      EmitBinarySearch(a, reg, l, m - 1, below, indCall);
      EmitBinarySearch(a, reg, m + 1, r, above, indCall);
   }
};

bool PatchModifier::inlineIndirectCall(
   CFGMaker* cfgMaker,
   PatchMgr::Ptr patcher,
   PatchFunction* caller,
   PatchBlock* cb,
   std::vector<Address> &calleeAddrs
)
{
   patch_printf("Enter PatchModifier::inlineDirectCall\n");
   PatchBlock* call_ft_block = GetCallFTBlock(cb);
   if (call_ft_block == nullptr) {
      patch_printf("\tcannot find call fallthrough block\n");
      return false;
   }
   // 1. Create indirect call inline dispatch code
   // and its corresponding CFG representation
   PatchBlock::Insns insns;
   cb->getInsns(insns);
   Snippet::Ptr indSnippet = IndirectCallInlineSnippet::create(new IndirectCallInlineSnippet(insns[cb->last()], calleeAddrs));
   auto point = patcher->findPoint(Location::InstructionInstance(caller, cb, cb->last()), Point::PreInsn, true);

   patch_printf("\tbefore insert dispatch code, call block range [%lx, %lx)\n", cb->start(), cb->end());
   for (auto e: cb->sources()) {
      patch_printf("\t\tincoming edge %p\n", e);
   }
   InsertedCode::Ptr indCallDispatch = PatchModifier::insert(caller->obj(), indSnippet, point);

   // 2. Wire CFG
   for (auto b : indCallDispatch->blocks()) {
      PatchModifier::addBlockToFunction(caller, b);
   }

   // 2.1 redirect control flow to dispatch code entry
   PatchBlock* dispatchEntry = indCallDispatch->entry();
   dispatchEntry->setAlignHint(true);
   std::vector<PatchEdge*> redges;
   if (!SplitAndGetEdges(cb, redges)) {
      patch_printf("\t\tfailed\n");
      return false;
   }
   if (!RedirectEdgeList(redges, dispatchEntry)) {
      patch_printf("\t\tfailed to redirect edges\n");
      return false;
   }


   for (auto e : indCallDispatch->exits()) {
      if (e->type() == ParseAPI::CALL_FT) continue;
      PatchBlock* b = e->src();
      PatchBlock::Insns insns;
      b->getInsns(insns);
      auto rit = insns.rbegin();
      rit++;
      const InstructionAPI::Instruction& cmpi = rit->second;
      if (cmpi.getOperation().getID() != e_cmp) continue;
      const unsigned char* addrStart = (const unsigned char*)cmpi.ptr();
      addrStart += cmpi.size() - 4;
      Address addr = *((const uint32_t*)addrStart);

      std::vector<PatchEdge*> entryEdges;
      entryEdges.emplace_back(e);
      e->setTailCallOverride(false);
      assert(!entryEdges.empty());
      if (!InlineImpl(cfgMaker, patcher, caller, entryEdges, addr, call_ft_block)) {
         patch_printf("\t\tfailed to inline callee %lx\n", addr);
         return false;
      }
   }

   for (auto e : indCallDispatch->exits()) {
      if (e->type() != ParseAPI::CALL_FT) continue;
      if (!PatchModifier::redirect(e, call_ft_block)) {
         patch_printf("\t\tfail to redirect edge to call ft block\n");
         return false;
      }
   }


   return true;
}
