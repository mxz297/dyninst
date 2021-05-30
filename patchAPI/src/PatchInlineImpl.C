
#include "PatchModifier.h"
#include "PatchCFG.h"
#include "PatchMgr.h"
#include "CFGModifier.h"

#include "debug_patch.h"
#include "PatchInlineSnippet.h"

#include <queue>
#include <set>
#include <vector>

using namespace std;
using namespace Dyninst;
using namespace PatchAPI;

static std::set<PatchFunction*> functionSplit;

static std::map<Address, int> versionNumberMap;

using FunctionCallSitesInfo = std::map<PatchBlock*, std::vector<Address> >;
static std::map<PatchFunction*, FunctionCallSitesInfo> inlineMap;
static std::map<Address, PatchFunction*> functionEntryMap;
static int ind_limit = 5;

void PatchModifier::setIndirectCallInlineLimit(int i) {
   ind_limit = i;
}

bool PatchModifier::beginInlineSet(PatchObject * obj) {
   patch_printf("Enter PatchModifier::beginInlineSet\n");
   functionSplit.clear();   
   inlineMap.clear();
   functionEntryMap.clear();
   for (auto & pair : obj->getFuncMap()) {
      PatchFunction* f = pair.second;
      functionEntryMap[f->addr()] = f;
   }
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

static PatchEdge* LookupNewEdge(PatchEdge* e, std::map<PatchBlock*, PatchBlock*>& blockMap) {
   PatchBlock* newSrc = blockMap[e->src()];
   assert(newSrc != nullptr);
   PatchBlock* newTrg = blockMap[e->trg()];
   assert(newTrg != nullptr);
   for (auto e1 : newSrc->targets()) {
      if (e1->trg() == newTrg) {
         return e1;
      }
   }
   return nullptr;
}

static bool InlineImpl(
   PatchFunction * caller,
   std::vector<PatchEdge*>& inEdges,
   Address calleeAddr,
   PatchBlock* returnTarget
) {
   // 1. Find the callee function
   PatchObject* po = caller->obj();
   CFGMaker * cfgMaker = po->getCFGMaker();
   PatchMgr::Ptr patcher = po->mgr();

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
      int version = versionNumberMap[b->start()] + 1;
      cloneB->setCloneVersion(version);
      versionNumberMap[b->start()] = version;
      cloneBlockMap[b] = cloneB;
      newBlocks.emplace_back(cloneB);
      PatchModifier::addBlockToFunction(caller, cloneB);

      // The callee may also contain inlined function,
      // so we need to clone instrumentation
      cloneB->cloneInstrumentation(caller, callee, b);
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
      auto p1 = patcher->findPoint(Location::BlockInstance(caller, newEntry), Point::BlockEntry, true);
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

static bool inlineDirectCall(
   PatchFunction* caller,
   PatchBlock* cb,
   Address callee
) {
   patch_printf("Enter inlineDirectCall: caller %s at %lx, call block [%lx, %lx), callee %lx\n",
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
   return InlineImpl(caller, redges, callee, call_ft_block);
}

static bool inlineIndirectCall(
   PatchFunction* caller,
   PatchBlock* cb,
   std::vector<Address> &calleeAddrs
)
{
   PatchMgr::Ptr patcher = caller->obj()->mgr();
   patch_printf("Enter inlineDirectCall\n");
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
      if (!InlineImpl(caller, entryEdges, addr, call_ft_block)) {
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

static bool CalleeHasConditionalTailCall(PatchFunction* callee) {
   for (auto b : callee->blocks()) {
      for (auto e : b->targets()) {
      if (!e->interproc()) continue;
      if (e->type() == ParseAPI::COND_TAKEN)
         return true;
      }
   }
   return false;
}

static bool FormInliningCycleImpl(
   PatchFunction *cur,
   PatchFunction *target,
   std::set<PatchFunction*>& visited
) {
   if (cur == target) return true;
   if (visited.find(cur) != visited.end()) return false;
   visited.insert(cur);
   auto it = inlineMap.find(cur);
   if (it == inlineMap.end()) return false;
   for (auto& callsite : it->second) {
      for (auto& addr : callsite.second) {
         PatchFunction* next = functionEntryMap[addr];
         if (FormInliningCycleImpl(next, target, visited)) return true;
      }
   }
   return false;
}

static bool FormInliningCycle(PatchFunction* caller, PatchFunction* callee) {
   std::set<PatchFunction*> visited;
   return FormInliningCycleImpl(callee, caller, visited);
}

bool PatchModifier::inlineCall(PatchFunction* f, PatchBlock*b, Address callee) {
   patch_printf("Attempt to inline %lx at call site ending at %lx", callee, b->end());

   bool isIndirect = false;
   PatchEdge* callEdge = nullptr;
   Address actualCallee = 0;
   for (auto e : b->targets()) {
      if (e->type() == ParseAPI::CALL && e->sinkEdge()) {
         isIndirect = true;

         // If IP-relative indirect call, skip it
         InstructionAPI::Instruction i = b->getInsn(b->last());
         if (i.readsMemory()) {
            InstructionAPI::RegisterAST thePC = InstructionAPI::RegisterAST::makePC(Dyninst::Arch_x86_64);
            std::set<Dyninst::InstructionAPI::Expression::Ptr> memAccess;
            i.getMemoryReadOperands(memAccess);
            for (auto m : memAccess) {
               if (m->bind(&thePC, InstructionAPI::Result(InstructionAPI::u32, 0))) {
                  patch_printf("\tdo not inline due to indirect pc-relative call\n");
                  return false;
               }
            }
         }
         continue;
      }
      if (e->type() == ParseAPI::CALL && !e->sinkEdge()) {
         actualCallee = e->trg()->start();
         callEdge = e;
      }
   }

   // Tail calls can cause wrong call sites
   if (!isIndirect && actualCallee != callee) {
      if (callEdge == nullptr) {
         patch_printf("\t cannot inline, do not find call edge for direct call\n");
         return false;
      }
      PatchBlock* trg = callEdge->trg();
      bool onlyBranch = false;
      int edgeCount = 0;
      for (auto e : trg->targets()) {
         edgeCount += 1;
         if (e->type() == ParseAPI::DIRECT) onlyBranch = true;
      }
      PatchBlock::Insns insns;
      trg->getInsns(insns);
      if (insns.size() == 1 && onlyBranch && edgeCount == 1) {

      } else {
         patch_printf("\tCannot inline unmatched direct call sites, can be caused by tail calls\n");
         return false;
      }
   }

   // Cannot inline recursive call
   if (f->addr() == callee) {
      patch_printf("\tdo not inline direct recursive call\n");
      return false;
   }

   // currently cannot inline callee if it has conditional tail calls
   PatchFunction* calleeFunc = b->obj()->getFunc(b->obj()->co()->findFuncByEntry(b->block()->region(), callee));
   if (calleeFunc == nullptr) {
      patch_printf("Cannot find function for callee %lx\n", callee);
      return false;
   }
   if (CalleeHasConditionalTailCall(calleeFunc)) {
      patch_printf("\tdo not inline call where callee has conditional tail call\n");
      return false;
   }

   if (isIndirect && inlineMap[f][b].size() == ind_limit) {
      patch_printf("\t do not inline, indirect call whose inline target has reached limit\n");
      return false;
   }

   if (FormInliningCycle(f, calleeFunc)) {
      patch_printf("\t do not inline, would cause inlining cycle\n");
      return false;
   }

   patch_printf("\t inlined, is indirect %d\n", isIndirect);
   inlineMap[f][b].emplace_back(callee);
   return true;
}

bool PatchModifier::endInlineSet() {
   patch_printf("Enter PatchModifier::endInlineSet\n");
   bool done = false;
   while (!done) {
      done = true;
      PatchFunction* caller;
      PatchBlock* cb;
      std::vector<Address> callees;
      for (auto &fmapIt : inlineMap) {
         for (auto& bIt : fmapIt.second) {
            bool ready = true;
            for (auto &ait : bIt.second) {
               PatchFunction* callee = functionEntryMap[ait];
               if (inlineMap.find(callee) != inlineMap.end()) {
                  ready = false;
                  break;
               }
            }
            if (ready) {
               done = false;
               caller = fmapIt.first;
               cb = bIt.first;
               callees = bIt.second;
               break;
            }
         }
         if (!done) break;
      }

      if (!done) {
         bool isIndirect = false;
         for (auto e : cb->targets()) {
            if (e->type() == ParseAPI::CALL && e->sinkEdge()) {
               isIndirect = true;
               break;
            }
         }
         patch_printf("Perform inlining: call block [%lx, %lx) in function at %lx, callee", cb->start(), cb->end(), caller->addr());
         for (auto a : callees) {
            patch_printf(" %lx ", a);
         }
         patch_printf("\n");
         if (isIndirect) {
            assert(inlineIndirectCall(caller, cb, callees));
         } else {
            assert(inlineDirectCall(caller, cb, callees[0]));
         }
         inlineMap[caller].erase(cb);
         if (inlineMap[caller].empty()) {
            inlineMap.erase(caller);
         }
      }
   }
   assert(inlineMap.empty());
   return true;
}