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

#ifndef _LOOPANALYZER_H_
#define _LOOPANALYZER_H_

#include <string>
#include <set>
#include "Annotatable.h"
#include "CFG.h"

using namespace std;

/** class which finds loops in a function 
  *
  */
namespace Dyninst {
namespace ParseAPI {

//  Implement WMZC algorithm to detect both natural loops 
//  and irreducible loops
//  Reference: "A New Algorithm for Identifying Loops in Decompilation"
//  by Tao Wei, Jian Mao, Wei Zou and Yu Chen

template<
  typename TF, // Function type: ParseAPI::Function, PatchAPI::PatchFunction
  typename TB, // Block type: ParseAPI::Block, PatchAPI::PatchBlock
  typename TE, // Edge type: ParseAPI::Edge, PatchAPI::PatchEdge
  typename TL, // Loop type: ParseAPI::Loop, PatchAPI::PatchLoop
  typename TLT // LoopTreeNode: ParseAPI::LoopTreeNode, PatchAPI::PatchLoopTreeNode
>
class LoopAnalyzer {
  
  TF *func;
  std::map<TB*, set<TB*> > loop_tree;
  std::map<TB*, TL*> loops;

  std::map<TB*, TB*> header;  
  std::map<TB*, int> DFSP_pos;
  std::set<TB*> visited;

  struct edge_sort {
    bool operator() (TE* l, TE* r) const {
        return l->trg()->start() < r->trg()->start();
    }
  };


  TB* WMZC_DFS(TB* b0, int pos) {
    visited.insert(b0);
    DFSP_pos[b0] = pos;
    // The final loop nesting structure depends on
    // the order of DFS. To guarantee that we get the 
    // same loop nesting structure for an individual binary 
    // in all executions, we sort the target blocks using
    // the start adress.
    vector<TE*> visitOrder;
    edge_sort es;
    visitOrder.insert(visitOrder.end(), b0->targets().begin(), b0->targets().end());
    sort(visitOrder.begin(), visitOrder.end(), es);
    
    for (auto e : visitOrder) {
      if (e->interproc() || e->sinkEdge() || e->type() == ParseAPI::CATCH) continue;
      TB* b = e->trg();	
      if (visited.find(b) == visited.end()) {
        // case A, new
        TB* nh = WMZC_DFS(b, pos + 1);
        WMZC_TagHead(b0, nh);
      } else {
        if (DFSP_pos[b] > 0) {
          // case B
          if (loops[b] == NULL) {
            loops[b] = new TL(func);
          }            
          WMZC_TagHead(b0, b);
          loops[b]->entries.insert(b);
          loops[b]->backEdges.insert(e);
        }
        else if (header[b] == NULL) {
          // case C, do nothing
        } else {
          TB* h = header[b];
          if (DFSP_pos[h] > 0) {
            // case D
            WMZC_TagHead(b0, h);
          } else {
            // case E
            // Mark b and (b0,b) as re-entry
            assert(loops[h]);
            loops[h]->entries.insert(b);
            while (header[h] != NULL) {
              h = header[h];
              if (DFSP_pos[h] > 0) {
                WMZC_TagHead(b0, h);
                break;
              }	
              assert(loops[h]);
              loops[h]->entries.insert(b);
            }
          }
        }
      }
    }
    DFSP_pos[b0] = 0;
    return header[b0];    
  }

  void WMZC_TagHead(TB* b, TB* h) {
    if (b == h || h == NULL) return;
    TB* cur1, *cur2;
    cur1 = b; cur2 = h;
    while (header[cur1] != NULL) {
      TB* ih = header[cur1];
      if (ih == cur2) return;
      if (DFSP_pos[ih] < DFSP_pos[cur2]) {
        // Can we guarantee both are not 0?
        header[cur1] = cur2;
        cur1 = cur2;
        cur2 = ih;
      } else cur1 = ih;
    }
    header[cur1] = cur2;
  }

  void FillMoreBackEdges(TL*loop)  {
    // All back edges to the header of the loop have been identified.
    // Now find all back edges to the other entries of the loop.
    for (auto b : loop->exclusiveBlocks) {        
      for (auto e : b->targets()) {
        if (e->interproc() || e->sinkEdge()) continue;
        if (loop->entries.find(e->trg()) != loop->entries.end())
          loop->backEdges.insert(e);
      }
    }
    for (auto b : loop->childBlocks) {        
      for (auto e : b->targets()) {            
        if (e->interproc() || e->sinkEdge()) continue;
        if (loop->entries.find(e->trg()) != loop->entries.end())
          loop->backEdges.insert(e);
      }
    }
  }

  void dfsCreateLoopHierarchy(
    TLT* parent,
    vector<TL*> &loops,
    std::string level) {
    
    for (unsigned int i = 0; i < loops.size(); i++) {
      // loop name is hierarchical level
      std::string clevel = (level != "")
        ? level + "." + utos(i+1)
        : utos(i+1);

      // add new tree nodes to parent
      TLT* child =
        new TLT(loops[i], (std::string("loop_"+clevel)).c_str());

      parent->children.push_back(child);

      // recurse with this child's outer loops
      vector<TL*> outerLoops;
      loops[i]->getOuterLoops(outerLoops);
      loop_sort l;
      std::sort(outerLoops.begin(), outerLoops.end(), l);
      dfsCreateLoopHierarchy(child, outerLoops, clevel);
    }
  }            

struct loop_sort {
  Address getLoopSmallestEntry(TL*l) const {
    vector<TB*> entries;
    l->getLoopEntries(entries);
    Address min = 0;
    for (auto b : entries) {
      if (min == 0 || b->start() < min) min = b->start();
    }
    return min;          
  }
  bool operator()(TL*l, TL*r) const {
    Address lentry = getLoopSmallestEntry(l);
    Address rentry = getLoopSmallestEntry(r);
    return lentry < rentry; 
  }
};            

public:
  bool analyzeLoops() {
    WMZC_DFS(func->entry(), 1);

    for (auto b : func->blocks()) {
      if (header[b] == NULL) continue;
      loop_tree[header[b]].insert(b);
    }

    for (auto b : func->blocks()) {        
      if (header[b] == NULL) {
        // if header[b] == NULL, b is either the header of a outermost loop, or not in any loop
        createLoops(b);
      }
    }

    // The WMZC algorithm only identifies back edges 
    // to the loop head, which is the first node of the loop 
    // visited in the DFS.
    // Add other back edges that targets other entry blocks
    for (auto b : func->blocks()) {      
      if (loops[b] != NULL) FillMoreBackEdges(loops[b]);
    }
    
    // Finish constructing all loops in the function.
    // Now populuate the loop data structure of the function.
    for (auto b : func->blocks()) {        
      if (loops[b] != NULL)
        func->_loops.insert(loops[b]); 
    }    
    return true;
  }

  /** create the tree of loops/callees for this flow graph */
  void createLoopHierarchy() {
    TLT* loopRoot = new TLT(NULL, NULL);
    func->_loop_root = loopRoot;

    vector<TL*> outerLoops;
    func->getOuterLoops(outerLoops);

    loop_sort l;
    std::sort(outerLoops.begin(), outerLoops.end(), l);
    // Recursively build the loop nesting tree
    dfsCreateLoopHierarchy(loopRoot, outerLoops, string(""));

    // Enumerate every basic blocks in the functions to find all create 
    // call information for each loop
    for (auto b : func->blocks()) {
      for (auto e : b->targets()) {
        // Can tail call happen here?
        if (e->type() == ParseAPI::CALL) {
          TB *target = e->trg();
          TF* callee = lookupFunctionByAddr(target->start());
          if (callee) insertCalleeIntoLoopHierarchy(callee, b->last());
          
          if (getArch() == Arch_ppc64) {
            // Since Power 8, the new ABI will typically create two entries for a function
            callee = lookupFunctionByAddr(target->start() - 8);
            if (callee == NULL) continue;
            // Make sure that the function entry block indeed aligns with the call target
            if (callee->entry()->end() != target->start()) continue;
            // Make sure that the function entry block will fall through to the call target
            bool findFT = false;
            for (auto eit = callee->entry()->targets().begin(); eit != callee->entry()->targets().end(); ++eit) {
              if ((*eit)->type() == FALLTHROUGH) {
                findFT = true;
                break;
              }
            }

            if (findFT)
                insertCalleeIntoLoopHierarchy(callee, b->last());
          }
        }
      }
    }
  }  
 
  LoopAnalyzer (const TF *f): func((TF*)f) {
    for (auto b : func->blocks()) {      
      DFSP_pos[b] = 0;
      header[b] = NULL;
    }
  }

  // The implementation of these two functions
  // varies for PatchAPI and ParseAPI.
  // So, leave them to template specialization.
  Dyninst::Architecture getArch();
  TF* lookupFunctionByAddr(Dyninst::Address);

 private:     


  bool dfsInsertCalleeIntoLoopHierarchy(
    TLT* node,
    TF* callee,
    unsigned long addr) {

    // if this node contains func then insert it
    if ((node->loop != NULL) && node->loop->containsAddress(addr)) {
      node->callees.push_back(callee);
      return true;
    }

    // otherwise recur with each of node's children
    bool success = false;

    for (auto n : node->children) {
      bool ret = dfsInsertCalleeIntoLoopHierarchy(n, callee, addr);
      success |= ret;
    }

    return success;
  }                                        

  void insertCalleeIntoLoopHierarchy(TF * callee, unsigned long addr) {
    // try to insert func into the loop hierarchy
    bool success = dfsInsertCalleeIntoLoopHierarchy(func->_loop_root, callee, addr);

    // if its not in a loop make it a child of the root
    if (!success) {
      func->_loop_root->callees.push_back(callee);
    }
  }


  void createLoops(TB* cur) {
    auto curLoop = loops[cur];
    if(curLoop == NULL) return;
    curLoop->insertBlock(cur);

    for (auto child : loop_tree[cur]) {        
      createLoops(child);
      auto childLoop = loops[child];
      if (childLoop != NULL) {
        curLoop->insertLoop(childLoop);
      }
      curLoop->insertBlock(child);
    }
  }

};

}
}

#endif /* _LOOPANALYZER_H_ */
