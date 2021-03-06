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
/* Public Interface */

#include "PatchCFG.h"
#include "PatchCallback.h"
#include "PatchMgr.h"

using namespace std;
using namespace Dyninst;
using namespace PatchAPI;

PatchEdge*
PatchEdge::create(ParseAPI::Edge *ie, PatchBlock *src, PatchBlock *trg) {
  PatchObject *obj = (src ? src->object() : trg->object());
  return obj->getEdge(ie, src, trg);
}

PatchEdge::PatchEdge(ParseAPI::Edge *internalEdge,
                     PatchBlock *source,
                     PatchBlock *target)
  : edge_(internalEdge), original_edge_(NULL), src_(source), trg_(target),
  tailCallOverride_(false), tailCallOverrideVal_(false) {
}

PatchEdge::PatchEdge(PatchEdge *parent, PatchBlock *src, PatchBlock *trg)
  : edge_(NULL), original_edge_(parent), src_(src), trg_(trg),
  tailCallOverride_(false), tailCallOverrideVal_(false) {
}

PatchBlock*
PatchEdge::src() {
  if (src_) return src_;
  // Interprocedural sources _must_ be pre-created since we don't
  // have enough information to create them here.
  assert(!interproc());
  assert(trg_);
  ParseAPI::Block *isrc = edge_->src();
  if (!isrc) return NULL;
  src_ = trg_->object()->getBlock(isrc);
  return src_;
}

PatchBlock*
PatchEdge::trg() {
  if (trg_) return trg_;
  assert(!interproc());
  assert(src_);
  ParseAPI::Block *itrg = edge_->trg();
  if (!itrg) return NULL;
  trg_ = src_->object()->getBlock(itrg);
  return trg_;
}

PatchEdge::~PatchEdge() {
}

ParseAPI::Edge*
PatchEdge::edge() const {
  return edge_;
}

PatchEdge* PatchEdge::original_edge() const { return original_edge_; }

ParseAPI::EdgeTypeEnum
PatchEdge::type() const {
    if (edge_ != NULL) return edge_->type();
    return original_edge_->type();
}

bool
PatchEdge::sinkEdge() {
   return trg()->start() == std::numeric_limits<Address>::max();
}

bool
PatchEdge::interproc() const {
    if (tailCallOverride_) {
       return tailCallOverrideVal_;
    }
    if (edge_ != NULL) {
  return edge_->interproc() ||
         (edge_->type() == ParseAPI::CALL) ||
         (edge_->type() == ParseAPI::RET);
    }
    return original_edge_->interproc();
}

void PatchEdge::remove(Point *p) {
   assert(p->edge() == this);
   points_.during = NULL;
}

PatchCallback *PatchEdge::cb() const {
   return src_->object()->cb();
}

bool PatchEdge::consistency() const { 
   fprintf(stderr, "PatchEdge::consistency() implementation is currently wrong\n");
   if (src_) {
      if (src_->block() != edge_->src()) CONSIST_FAIL;
   }
   if (trg_) {
      if (trg_->block() != edge_->trg()) CONSIST_FAIL;
   }

   if (!points_.consistency(this, NULL)) CONSIST_FAIL;
   return true;
}

bool EdgePoints::consistency(const PatchEdge *edge, const PatchFunction *func) const {
   if (during) {
      if (!during->consistency()) CONSIST_FAIL;
      if (during->type() != Point::EdgeDuring) CONSIST_FAIL;
      if (during->edge() != edge) CONSIST_FAIL;
      if (during->func() != func) CONSIST_FAIL;
   }
   return true;
}

std::string PatchEdge::format() const {
   stringstream ret;
   ret << "{"
       << src_->format()
       << ","
       << trg_->format()
       << ","
       << ParseAPI::format(type())
       << "}";
   return ret.str();
}

void PatchEdge::setTailCallOverride(bool val) {
   tailCallOverride_ = true;
   tailCallOverrideVal_ = val;
}

void PatchEdge::cloneInstrumentation(
   PatchFunction* newF, // newF is the function that contains this edge
   PatchFunction* f, // f is the function that contains edge e
   PatchEdge *e
) {
   PatchMgr::Ptr patcher = f->obj()->mgr();
   auto p1 = patcher->findPoint(Location::EdgeInstance(f, e), Point::EdgeDuring, false);
   if (p1 != nullptr) {
      auto p2 = patcher->findPoint(Location::EdgeInstance(newF, this), Point::EdgeDuring, true);
      for (auto const& instInstance : p1->getInstanceList()) {
         p2->pushBack(instInstance->snippet());
      }
   }   
}