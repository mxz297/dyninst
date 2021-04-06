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

static bool SplitAndRedirectEdges(PatchBlock* b, PatchBlock* tar, bool &split) {
   PatchBlock::Insns insns;
   b->getInsns(insns);
   if (insns.size() > 1) {
      split = true;
      patch_printf("\t\tblock has more than one instruction\n");
      // We first remove the return instruction
      if (PatchModifier::split(b, b->last(), true, b->last()) == nullptr) {
         patch_printf("\t\t\tfail to split the block at [%lx, %lx)\n", b->start(), b->end());
         return false;
      }
      // We then redirect the fallthrough edge from
      // then-return block to the call-ft block
      PatchEdge * ftedge = getFTEdge(b);
      if (ftedge == nullptr) {
         patch_printf("\t\t\tno fallthrough edge after spliting block [%lx, %lx)\n", b->start(), b->end());
         return false;
      }
      if (!PatchModifier::redirect(ftedge, tar)) {
         patch_printf("\t\t\tfail to redirect splitted block [%lx, %lx) to new target\n", b->start(), b->end());
         return false;
      }
   } else {
      split = false;
      patch_printf("\t\tblock has only one instruction\n");
      std::vector<PatchEdge*> sedges;
      for (auto e : b->sources()) {
         sedges.emplace_back(e);
      }
      for (auto e: sedges) {
         if (!PatchModifier::redirect(e, tar)) {
            patch_printf("\t\t\tfail to redirect edge from [%lx, %lx] to the new target, edge type %d\n",
               e->src()->start(), e->src()->end(), e->type());
            return false;
         }
         patch_printf("\t\tredirect source from [%lx, %lx), type %d\n", e->src()->start(), e->src()->end(), e->type());
         for (auto te: e->src()->targets()) {
            patch_printf("\t\t\ttarget to [%lx, %lx), type %d\n", te->trg()->start(), te->trg()->end(), te->type());
         }
      }
   }
   return true;
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

static int version;

#include "Snippet.h"
class AdjustSPSnippet : public Dyninst::PatchAPI::Snippet {
 public:
  explicit AdjustSPSnippet(int adj): adjustment(adj) {}
  bool generate(Dyninst::PatchAPI::Point* pt, Dyninst::Buffer& buf) override {
    // instruction template 48 8d a4 24 58 ff ff ff    lea    -0xa8(%rsp),%rsp
    const int LENGTH = 8;
    unsigned char insn[LENGTH];
    insn[0] = 0x48;
    insn[1] = 0x8d;
    insn[2] = 0xa4;
    insn[3] = 0x24;
    int32_t disp = adjustment;
    *((int32_t*)(insn+4)) = disp;
    buf.copy(insn, LENGTH);
    return true;
  }
 private:
  int adjustment;
};

bool PatchModifier::inlineFunction(CFGMaker* cfgMaker, PatchMgr::Ptr patcher, PatchFunction* caller, PatchBlock* cb) {
   version += 1;
   patch_printf("Enter PatchModifier::inlineFunction: caller %s at %lx, call block [%lx, %lx)\n",
     caller->name().c_str(), caller->addr(), cb->start(), cb->end());
   // 1. Find the callee function
   PatchFunction* callee = cb->getCallee();
   // callee is NULL if this is not a call block or an indirect call
   if (callee == nullptr) {
      patch_printf("\tCannot find callee\n");
      return false;
   }
   patch_printf("\tget callee %s at %lx\n", callee->name().c_str(), callee->addr());

   if (!SplitFunctionRetBlocks(callee)) {
      patch_printf("\tfail to spilt return blocks in %s at %lx\n", callee->name().c_str(), callee->addr());
      return false;
   }

   PatchBlock* call_ft_block = nullptr;
   for (auto e : cb->targets()) {
      if (e->type() == ParseAPI::CALL_FT) {
         call_ft_block = e->trg();
         break;
      }
   }
   if (call_ft_block == nullptr) {
      patch_printf("\tcannot find call fallthrough block\n");
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
      bool split;
      if (!SplitAndRedirectEdges(b, call_ft_block, split)) {
         patch_printf("\t\tfailed\n");
         return false;
      }
   }

   // 6. Remove call instruction and redirect edge to inlined entry
   bool split;
   patch_printf("\tsplit call block and redirect edge for [%lx, %lx)\n", cb->start(), cb->end());
   if (!SplitAndRedirectEdges(cb, cloneBlockMap[callee->entry()], split)) {
      patch_printf("\t\tfailed\n");
      return false;
   }

   // 7. Move down SP to mimic the effect of a call instruction to stack
   if (!split) {
      for (auto e : cb->sources()) {
         auto p1 = patcher->findPoint(Location::EdgeInstance(caller, e), Point::EdgeDuring, true);
         Snippet::Ptr moveSPDown = AdjustSPSnippet::create(new AdjustSPSnippet(-8));
         p1->pushBack(moveSPDown);
      }
   } else {
      auto p1 = patcher->findPoint(Location::BlockInstance(caller, cb, true), Point::BlockExit, true);
      Snippet::Ptr moveSPDown = AdjustSPSnippet::create(new AdjustSPSnippet(-8));
      p1->pushBack(moveSPDown);
   }

   // 8. Move up SP to mimic the effect of a return instruction to stack
   auto p2 = patcher->findPoint(Location::BlockInstance(caller, call_ft_block, true), Point::BlockEntry, true);
   Snippet::Ptr moveSPUp = AdjustSPSnippet::create(new AdjustSPSnippet(8));
   p2->pushBack(moveSPUp);

   return true;
}
