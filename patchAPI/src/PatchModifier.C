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

   // Current limitation: cannot retarget indirect edges (well, we can,
   // but don't expect it to work)
   // Also, changing catch edges would be awesome! ... but we can't.
   if (edge->type() == ParseAPI::CATCH ||
       edge->type() == ParseAPI::RET) return false;

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
    f->addBlock(b);
    return true;
}

static PatchEdge* getFTEdge(PatchBlock* b) {
   for (auto e : b->targets()) {
      if (e->type() == ParseAPI::FALLTHROUGH) return e;
   }
   return nullptr;
}

static bool SplitAndRedirectEdges(PatchBlock* b, PatchBlock* tar) {
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
      if (ftedge == nullptr) {
         patch_printf("\t\t\tno fallthrough edge after spliting block [%lx, %lx)\n", b->start(), b->end());
         return false;
      }
      if (!PatchModifier::redirect(ftedge, tar)) {
         patch_printf("\t\t\tfail to redirect splitted block [%lx, %lx) to new target\n", b->start(), b->end());
         return false;
      }
   } else {
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
      }
   }
   return true;
}

static bool CheckBlock(PatchBlock* b){
   // If the block contains only one instruction,
   // we need to redirect its source edges.
   // TODO: how to redirect INDIRECT source edges
   PatchBlock::Insns insns;
   b->getInsns(insns);
   if (insns.size() > 1) return true;
   for (auto e : b->sources()) {
      if (e->type() == ParseAPI::INDIRECT) {
         patch_printf("\t\tblock [%lx, %lx) has INDIRECT source edge \n", b->start(), b->end());
         return false;
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

bool PatchModifier::inlineFunction(CFGMaker* cfgMaker, PatchFunction* caller, PatchBlock* cb) {
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

   patch_printf("\tCheck whether we can handle call block\n");
   if (!CheckBlock(cb)) {
      patch_printf("\t\tCannot handle call block\n");
      return false;
   }

   patch_printf("\tCheck whether we can handle return blocks\n");
   for (auto b: callee->blocks()) {
      bool isRet = false;
      for (auto e: b->targets()) {
         if (e->type() == ParseAPI::RET) isRet = true;
      }
      if (!isRet) continue;
      if (!CheckBlock(b)) {
         patch_printf("\t\tCannot handle return block [%lx, %lx)\n", b->start(), b->end());
         return false;
      }
   }

   // Only inline function without jump tables
   for (auto b: callee->blocks()) {
      for (auto e: b->targets()) {
         if (e->intraproc() && e->type() == ParseAPI::INDIRECT && !e->sinkEdge()) {
            patch_printf("\tcallee has jump table at [%lx, %lx)\n", b->start(), b->end());
            return false;
         }
      }
   }

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
      cloneBlockMap[b] = cloneB;
      newBlocks.emplace_back(cloneB);
      PatchModifier::addBlockToFunction(caller, cloneB);
   }

   // The edges of the cloned blocks are still from/to original blocks
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

   // 3. Remove return instruction from return blocks
   for (auto b: newRetBlocks) {
      patch_printf("\tsplit return block and redirect edge for [%lx, %lx)\n", b->start(), b->end());
      if (!SplitAndRedirectEdges(b, call_ft_block)) {
         patch_printf("\t\tfailed\n");
         return false;
      }
   }

   // 4. Remove call instruction and redirect edge to inlined entry
   patch_printf("\tsplit call block and redirect edge for [%lx, %lx)\n", cb->start(), cb->end());
   if (!SplitAndRedirectEdges(cb, cloneBlockMap[callee->entry()])) {
      patch_printf("\t\tfailed\n");
      return false;
   }
   return true;
}
