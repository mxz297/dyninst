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


#include <stdio.h>
#include <string.h>
#include <iostream>
#include "common/src/std_namesp.h"
#include <iterator>
#include <algorithm>

#include "PatchCFG.h"

using namespace std;
using namespace Dyninst;
using namespace Dyninst::PatchAPI;
//constructors
//internal use only


PatchLoop::PatchLoop(PatchFunction *f)
    : func(f), parent(NULL) {
    }

PatchLoop::PatchLoop(PatchEdge *be, PatchFunction *f)
    : func(f), parent(NULL) 
{
    backEdges.insert(be);
}

bool PatchLoop::containsAddress(Address addr)
{
    for(auto b : exclusiveBlocks) {
        if (addr >= b->start() && addr < b->end())
            return true;
    }
    return false;
}

bool PatchLoop::containsAddressInclusive(Address addr)
{
    vector<PatchBlock*> blks;
    getLoopBasicBlocks(blks);

    for(unsigned i = 0; i < blks.size(); i++) {
	if (addr >= blks[i]->start() &&
	    addr < blks[i]->end() ) 
	    return true;
    }

    return false;
}

int PatchLoop::getBackEdges(vector<PatchEdge*> &edges)
{
   edges.insert(edges.end(), backEdges.begin(), backEdges.end());
   return edges.size();

}

bool 
PatchLoop::hasAncestor(PatchLoop* l) {
   return (l->containedLoops.find(this) != l->containedLoops.end());
}


bool
PatchLoop::getLoops(vector<PatchLoop*>& nls, bool outerMostOnly) const
{
   for (auto iter = containedLoops.begin(); iter != containedLoops.end(); ++iter) {
      // only return a contained PatchLoop if this PatchLoop is its parent
      if (outerMostOnly && (this != (*iter)->parent)) continue;
      nls.push_back(*iter);
   }
   
   return true;
}

//method that returns the nested PatchLoops inside the PatchLoop. It returns a set
//of PatchLoop that are contained. It might be useful to add nest 
//as a field of this class but it seems it is not necessary at this point
bool
PatchLoop::getContainedLoops(vector<PatchLoop*>& nls)
{
  return getLoops(nls, false);
}

// get the outermost PatchLoops nested under this PatchLoop
bool 
PatchLoop::getOuterLoops(vector<PatchLoop*>& nls)
{
  return getLoops(nls, true);
}

//returns the basic blocks in the PatchLoop
bool PatchLoop::getLoopBasicBlocks(vector<PatchBlock*>& bbs) {
   bbs.insert(bbs.end(), exclusiveBlocks.begin(), exclusiveBlocks.end());
    bbs.insert(bbs.end(), childBlocks.begin(), childBlocks.end());
  return true;
}

void PatchLoop::insertBlock(PatchBlock* b)
{
    if(childBlocks.find(b) == childBlocks.end()) exclusiveBlocks.insert(b);
}
void PatchLoop::insertChildBlock(PatchBlock* b)
{
    exclusiveBlocks.erase(b);
    childBlocks.insert(b);
}


// returns the basic blocks in this PatchLoop, not those of its inner PatchLoops
bool PatchLoop::getLoopBasicBlocksExclusive(vector<PatchBlock*>& bbs) {
    std::copy(exclusiveBlocks.begin(), exclusiveBlocks.end(), std::back_inserter(bbs));
    return true;
}



bool PatchLoop::hasBlock(PatchBlock* block) 
{
    vector<PatchBlock*> blks;
    getLoopBasicBlocks(blks);

    for(unsigned i = 0; i < blks.size(); i++)
        if (blks[i]->start() == block->start())
            return true;
    return false;
}


bool PatchLoop::hasBlockExclusive(PatchBlock*block) 
{
    vector<PatchBlock*> blks;
    getLoopBasicBlocksExclusive(blks);

    for(unsigned i = 0; i < blks.size(); i++)
        if (blks[i]->start() == block->start())
            return true;
    return false;
}




int PatchLoop::getLoopEntries(vector<PatchBlock*> &e) {
    e.insert(e.end(), entries.begin(), entries.end());
    return e.size();
}


PatchFunction* PatchLoop::getFunction()
{
    return func;
}


void PatchLoop::insertLoop(PatchLoop *childLoop) {
    containedLoops.insert(childLoop);
    childLoop->parent = this;
    for(auto L = childLoop->containedLoops.begin();
            L != childLoop->containedLoops.end();
            ++L)
    {
        containedLoops.insert(*L);
    }
    for (auto b = childLoop->exclusiveBlocks.begin();
         b != childLoop->exclusiveBlocks.end();
         ++b) {
        insertChildBlock(*b);
    }
    for (auto b = childLoop->childBlocks.begin();
         b != childLoop->childBlocks.end();
         ++b) {
        insertChildBlock(*b);
    }
}

std::string PatchLoop::format() const {
   std::stringstream ret;
   
   ret << hex << "(PatchLoop " << this << ": ";
   for (std::set<PatchBlock *>::iterator iter = exclusiveBlocks.begin();
        iter != exclusiveBlocks.end(); ++iter) {
      ret << (*iter)->start() << ", ";
   }
    for (std::set<PatchBlock *>::iterator iter = childBlocks.begin();
         iter != childBlocks.end(); ++iter) {
        ret << (*iter)->start() << ", ";
    }
   ret << ")" << dec << endl;

   return ret.str();
}
