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


#include "PatchCFG.h"

using namespace Dyninst;
using namespace Dyninst::PatchAPI;

PatchLoopTreeNode::PatchLoopTreeNode(PatchLoop *l, const char *n)
{
    loop = l;
    hierarchicalName = NULL;
    if (n != NULL) {
        hierarchicalName = strdup(n);
    }    
}
 

const char * 
PatchLoopTreeNode::getCalleeName(unsigned int i) 
{
    assert(i < callees.size());
    assert(callees[i] != NULL);
    return callees[i]->name().c_str();
}

const char * 
PatchLoopTreeNode::name()
{
    assert(loop != NULL);
    return hierarchicalName; 
}

unsigned int
PatchLoopTreeNode::numCallees() { 
    return callees.size(); 
}


PatchLoopTreeNode::~PatchLoopTreeNode() {
    // Loops are deleted by Function...
    for (unsigned i = 0; i < children.size(); i++)
	delete children[i];
    if (hierarchicalName)
        free(hierarchicalName);
    // don't delete callees!
}


PatchLoop *
PatchLoopTreeNode::findLoop(const char *name) 
{ 
    if (loop) {
        if (0==strcmp(name,hierarchicalName)) 
            return loop;
    }
    for (unsigned i = 0; i < children.size(); i++) {
        PatchLoop *lp = children[i]->findLoop(name);
        if (lp) return lp;
    }
    return NULL;
}

bool PatchLoopTreeNode::getCallees(vector<PatchFunction *> &v)
{
   for (unsigned i=0; i<callees.size(); i++) {
      v.push_back(callees[i]);
   }
   return true;
}
