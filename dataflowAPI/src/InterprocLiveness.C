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

#include "debug_dataflow.h"
#include "CFG.h"
#include "Location.h"
#include "InstructionDecoder.h"
#include "Register.h"
#include "Instruction.h"

#include "liveness.h"
#include "ABI.h"
#include "bitArray.h"
#include "RegisterMap.h"

using namespace std;
using namespace Dyninst;
using namespace Dyninst::ParseAPI;
using namespace Dyninst::InstructionAPI;

void InterproceduralLivenessAnalyzer::analyze(CodeObject* codeObject) {
    co = codeObject;
    SymtabCodeSource* scs = static_cast<SymtabCodeSource*>(co->cs());
    abi = ABI::getABI(scs->getAddressWidth());
    setPassParamRegisterArray();
    
    calculateBlocksSummary();
    identifyIndirectCallFTBlocks();
    calculateFixedPoint();
}

static void printArray(bitArray &a) {
    vector< pair<int, string> > liveRegs;
    for (auto &it : DataflowAPI::machRegIndex_x86_64()) {
        if (a[it.second] == 1) {
            liveRegs.emplace_back(make_pair(it.second, it.first.name()));
        }
    }
    sort(liveRegs.begin(), liveRegs.end());
    printf("\t\t");
    for (auto &it: liveRegs) {
        printf("%s,", it.second.c_str());
    }
    printf("\n");
}

void InterproceduralLivenessAnalyzer::print(CodeObject* codeObject) {
    analyze(codeObject);
    for (auto f : codeObject->funcs()) {
        printf("Function %s at %lx\n", f->name().c_str(), f->addr());
        for (auto b : f->blocks()) {
            ConcurrentLivenessBlockDataMap::accessor a;
            assert(blockLiveMap.find(a, b));           
            printf("\tLive at block entry [%lx, %lx)\n", b->start(), b->end());
            printArray(a->second.in);
            printf("\tLive at block exit [%lx, %lx)\n", b->start(), b->end());
            printArray(a->second.out);
            printf("\tUse in block [%lx, %lx)\n", b->start(), b->end());
            printArray(a->second.use);
            printf("\tDef in block [%lx, %lx)\n", b->start(), b->end());
            printArray(a->second.def);

        }
    }
}

void InterproceduralLivenessAnalyzer::setPassParamRegisterArray() {
    passParamRegs = abi->getBitArray();
    passParamRegs.set(abi->getIndex(x86_64::rax));
    passParamRegs.set(abi->getIndex(x86_64::rcx));
    passParamRegs.set(abi->getIndex(x86_64::rdx));
    passParamRegs.set(abi->getIndex(x86_64::rsi));
    passParamRegs.set(abi->getIndex(x86_64::rdi));
    passParamRegs.set(abi->getIndex(x86_64::r8));
    passParamRegs.set(abi->getIndex(x86_64::r9));

    passParamRegs.set(abi->getIndex(x86_64::xmm0));
    passParamRegs.set(abi->getIndex(x86_64::xmm1));
    passParamRegs.set(abi->getIndex(x86_64::xmm2));
    passParamRegs.set(abi->getIndex(x86_64::xmm3));
    passParamRegs.set(abi->getIndex(x86_64::xmm4));
    passParamRegs.set(abi->getIndex(x86_64::xmm5));
    passParamRegs.set(abi->getIndex(x86_64::xmm6));
    passParamRegs.set(abi->getIndex(x86_64::xmm7));
}

void InterproceduralLivenessAnalyzer::calculateBlocksSummary() {
    std::set<Block*> blocks;
    for (auto f : co->funcs()) {
        for (auto b: f->blocks()) {
            blocks.insert(b);
        }
    }

    std::vector<Block*> blockVec;
    for (auto b : blocks) {
        blockVec.emplace_back(b);
    }

    for (size_t i = 0; i < blockVec.size(); ++i) {
        Block* b = blockVec[i];
        calculateABlockSummary(b);
    }
}

static bool convertRegisterToBitArray(ABI* abi, std::set<RegisterAST::Ptr>& regs, bitArray& arr) {
    for (auto& r : regs) {
        MachRegister cur = r->getID();
        MachRegister base = cur.getBaseRegister();
        int index = abi->getIndex(base);
        if (index < 0) {
            fprintf(stderr, "cannot get register index for %s, base %s\n", cur.name().c_str(), base.name().c_str());
            return false;
        }
        arr[index] = true;
    }
    return true;
}
static bool handleSpecialInstruction(
    ABI* abi, 
    InstructionAPI::Instruction &i, 
    bitArray &read, 
    std::set<RegisterAST::Ptr> &readRegs, 
    bitArray &written, 
    std::set<RegisterAST::Ptr> writtenRegs
) {
    // xor reg reg
    // clears reg to zero.
    if (i.getOperation().getID() == e_xor) {
        if (readRegs.size() != 1) return false;
        if (writtenRegs.size() != 1) return false;
        MachRegister r1 = (*readRegs.begin())->getID();
        MachRegister r2 = (*writtenRegs.begin())->getID();
        if (r1 != r2) return false;
        convertRegisterToBitArray(abi, writtenRegs, written);
        return true;
    }
    return false;
}

void InterproceduralLivenessAnalyzer::calculateABlockSummary(Block* b) {
    livenessData data;
    data.use = data.def = data.in = data.out = abi->getBitArray();

    Block::Insns insns;
    b->getInsns(insns);

    for (auto &iit: insns) {
        InstructionAPI::Instruction &i = iit.second;

        bitArray read = abi->getBitArray();        
        std::set<RegisterAST::Ptr> readRegs;
        i.getReadSet(readRegs);

        bitArray written = abi->getBitArray();
        std::set<RegisterAST::Ptr> writtenRegs;
        i.getWriteSet(writtenRegs);        
        
        if (!handleSpecialInstruction(abi, i, read, readRegs, written, writtenRegs)) {
            // In typical cases, read registers are used
            // and written registers are defined
            if (!convertRegisterToBitArray(abi, readRegs, read)) {
                fprintf(stderr, "fail to convert read registers to bit array\n");
                fprintf(stderr, "\t%lx: %s\n", iit.first, i.format().c_str());            
            }        
            
            if (!convertRegisterToBitArray(abi, writtenRegs, written)) {
                fprintf(stderr, "fail to convert written registers to bit array\n");
                fprintf(stderr, "\t%lx: %s\n", iit.first, i.format().c_str());            
            }
        } 
        
        data.use |= (read & ~data.def);
        data.def |= written;
    }
    ConcurrentLivenessBlockDataMap::accessor a;
    blockLiveMap.insert(a, make_pair(b, data));
}

void InterproceduralLivenessAnalyzer::identifyIndirectCallFTBlocks() {
    indCallFTBlocks.clear();
    for (auto f: co->funcs()) {
        for (auto b : f->blocks()) {
            bool hasIndirectCall = false;
            Block* ft_block = nullptr;
            for (auto e : b->targets()) {
                if (e->type() == CALL && e->sinkEdge()) {
                    hasIndirectCall = true;
                }
                if (e->type() == CALL_FT) {
                    ft_block = e->trg();
                }
            }
            if (hasIndirectCall && ft_block != nullptr) {
                indCallFTBlocks.emplace_back(ft_block);
            }
        }
    }
}

void InterproceduralLivenessAnalyzer::calculateFixedPoint() {
    bool done = false;
    joinOfFuncEntryBlocks = abi->getBitArray();
    joinOfIndirCallFTBlocks = abi->getBitArray();
    while (!done) {
        done = true;
        joinOfFuncEntryBlocks.reset();
        for (auto f: co->funcs()) {
            ConcurrentLivenessBlockDataMap::accessor a;
            assert(blockLiveMap.find(a, f->entry()));
            joinOfFuncEntryBlocks |= a->second.in;
        }
        joinOfIndirCallFTBlocks.reset();
        for (auto b: indCallFTBlocks) {
            ConcurrentLivenessBlockDataMap::accessor a;
            assert(blockLiveMap.find(a, b));
            joinOfIndirCallFTBlocks |= a->second.in;
        }

        for (auto f : co->funcs()) {
            for (auto b: f->blocks()) {
                if (updateBlock(b)) done = false;
            }
        }
    }
}

bool InterproceduralLivenessAnalyzer::updateBlock(Block *b) {
    ConcurrentLivenessBlockDataMap::accessor a;
    assert(blockLiveMap.find(a, b));
    livenessData &data = a->second;
    bitArray input = abi->getBitArray();

    bool isRetBlock = false;
    bool isIndirCall = false;
    bool isPLTCall = false;
    Block* callFTBlock = nullptr;

    for (auto e : b->targets()) {
        if (e->type() == ParseAPI::RET) isRetBlock = true;
        if (e->type() == ParseAPI::CALL && e->sinkEdge()) isIndirCall = true;
        if ((e->type() == ParseAPI::DIRECT || e->type() == ParseAPI::COND_TAKEN) && e->interproc() && e->sinkEdge()) isIndirCall = true;
        if (e->type() == ParseAPI::CALL && !e->sinkEdge()) {
            if (co->cs()->linkage().find(e->trg()->start()) != co->cs()->linkage().end()) isPLTCall = true;
        }
        if (e->type() == ParseAPI::CALL_FT) callFTBlock = e->trg();
    }
    if (isRetBlock) {
        input = joinOfIndirCallFTBlocks;
    } else if (isIndirCall) {
        input = joinOfFuncEntryBlocks;
    } else if (isPLTCall) {
        // Assume the ABI where all registers that can be used for
        // passing parameters will be used in the callee
        input = passParamRegs;
        if (callFTBlock != nullptr) {
            ConcurrentLivenessBlockDataMap::accessor a2;
            assert(blockLiveMap.find(a2, callFTBlock));
            input = input | a2->second.in;  
        }
    } else {
        for (auto e : b->targets()) {
            if (e->sinkEdge()) continue;
            // How exception impacts interprocedural liveness?
            if (e->type() == ParseAPI::CATCH) continue;
            if (e->type() == ParseAPI::CALL_FT) continue;
            if (e->trg() == b) {
                input |= data.in;
            } else {
                ConcurrentLivenessBlockDataMap::accessor a2;
                assert(blockLiveMap.find(a2, e->trg()));
                input |= a2->second.in;
            }
        }
    }

    data.out = input;
    bitArray output = data.use | (data.out - data.def);
    if (output == data.in) {
        return false;
    } else {
        data.in = output;
        return true;
    }
}