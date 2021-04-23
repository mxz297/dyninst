#include "JumpTableMover.h"
#include "dyninstAPI/src/addressSpace.h"
#include "dyninstAPI/src/function.h"
#include "dyninstAPI/src/debug.h"
#include "dyninstAPI/src/mapped_object.h"

#include "SymEval.h"
#include "slicing.h"


using namespace std;
using namespace Dyninst;
using namespace Dyninst::DataflowAPI;
using namespace Relocation;

JumpTableMover::Ptr JumpTableMover::create(FuncSetOrderdByLayout::const_iterator begin,
        FuncSetOrderdByLayout::const_iterator end,
        AddressSpace *s) {
    Ptr ret = Ptr(new JumpTableMover(s));
    ret->setupRelocationEntries();

    for (; begin != end; ++begin) {
        func_instance *func = *begin;
        if (!func->isInstrumentable()) {
            relocation_cerr << " Jump table relocation skips uninstrumentable function "
                << func->name() << " at " << hex << func->addr() << dec << endl;
            continue;
        }
        ret->moveJumpTableInFunction(func);
        if (func->getNoPowerPreambleFunc() != nullptr)
            ret->moveJumpTableInFunction(func->getNoPowerPreambleFunc());
        if (func->getPowerPreambleFunc() != nullptr)
            ret->moveJumpTableInFunction(func->getPowerPreambleFunc());
    }

    return ret;
}

void JumpTableMover::setupRelocationEntries() {
    vector<SymtabAPI::relocationEntry> &r = as->getAOut()->parse_img()->getObject()->getDynRelocations();
    for (size_t i = 0; i < r.size(); ++i) {
        relocs[r[i].rel_addr()] = &r[i];
    }
}

void JumpTableMover::moveJumpTableInFunction(func_instance *func) {
    if (processed_funcs.find(func) != processed_funcs.end()) return;
    processed_funcs.insert(func);
    for (const auto& jit : func->getJumpTableMap()) {
        moveOneJumpTable(func, jit.first, jit.second);
    }

}

void JumpTableMover::moveOneJumpTable(
    func_instance* func,
    PatchAPI::PatchBlock* b,
    const PatchAPI::PatchFunction::PatchJumpTableInstance& jt
) {
    Address jumpAddr = b->last();
    // First, detetermine whether the relocation will cause table entry overflow.
    bool overflow = computeNewTableEntries(func, b, jt);
    Address newJumpTargetBase = 0;
    bool needLargerTable = overflow && checkFitInAndAdjust(newJumpTargetBase, jt);
    if (overflow && !needLargerTable) {
        if (!modifyJumpTargetBaseInstructions(func, b->getCloneVersion(), newJumpTargetBase, jt)) {
            fprintf(stderr, "need to modify jump target base to %lx, but failed\n", newJumpTargetBase);
            assert(0);
        }
    }
    // We can rewrite the original jump table inplace only when
    // we are relocating the original code.
    // For all other cases, we need to allocate a new table
    Address tableStart = jt.tableStart;
    if (needLargerTable || b->getCloneVersion() > 0) {
        tableStart = as->inferiorMalloc(jt.tableEnd - jt.tableStart);
        if (!modifyJumpTableBaseInstructions(func, b->getCloneVersion(), tableStart, jt)) {
            fprintf(stderr, "need to modify jump table base to %lx, but failed\n", tableStart);
            assert(0);
        }
    }
    /* TODO
    if (needLargerTable) {
        if (!modifyJumpTableAccess()) {
            fprintf(stderr, "need to increase jump table memory access width, but failed\n");
            assert(0);
        }
    }
    */
    // Emit the new table
    codeGen gen;
    gen.invalidate();
    gen.allocate(jt.tableEnd - jt.tableStart);
    gen.setAddrSpace(as);
    gen.setAddr(tableStart);
    fillNewTableEntries(gen, jumpAddr, jt.indexStride);
    codeGens.emplace_back(gen);
}

bool JumpTableMover::computeNewTableEntries(
    func_instance* func,
    PatchAPI::PatchBlock* b,
    const PatchAPI::PatchFunction::PatchJumpTableInstance& jt
) {
    bool overflow = false;
    // Old table entry address -> pair of relocated target address and new table entry value
    newTable.clear();

    relocation_cerr << "Relocation jump table at " << hex << b->last()
                << " for function " << func->name() << " at " << func->addr() << dec
                << " table stride " << jt.indexStride
                << " block clone version " << b->getCloneVersion() << endl;
    int index = 0;
    for (Address addr = jt.tableStart; addr < jt.tableEnd; addr += jt.indexStride) {
        // 1. Lookup the relocated target address
        // the target edge can be redirected
        PatchAPI::PatchEdge * targetEdge = jt.tableEntryEdges[index++];
        Address reloc = findRelocatedBlockStart(func, targetEdge->trg());

        // For ppc64le, we check relocated addresses using both
        // global entry function and local entry function.
        if (reloc == 0 && func->getNoPowerPreambleFunc() != nullptr) {
            reloc = findRelocatedBlockStart(func->getNoPowerPreambleFunc(), targetEdge->trg());
        }
        if (reloc == 0 && func->getPowerPreambleFunc() != nullptr) {
            reloc = findRelocatedBlockStart(func->getPowerPreambleFunc(), targetEdge->trg());
        }

        if (reloc == 0) {
            fprintf(stderr, "Cannot find relocated address for %lx for jump table at %lx for function %s at %lx\n",
              targetEdge->trg()->start(), b->last(), func->name().c_str(), func->addr());
        }
        assert(reloc);

        // 2. Calculate the new table entry
        NewTableEntryVisitor ntev(reloc);
        jt.jumpTargetExpr->accept(&ntev);
        int64_t newEntry = ntev.newValue;
        relocation_cerr << "\t table address " << hex << addr
            << " original target address " << targetEdge->trg()->start()
            << " new target address " << reloc
            << " new entry value " << dec << newEntry << endl;

        // 3. Record new target and new entry value
        newTable.emplace(addr, make_pair(reloc, newEntry));

        // 5. Check if new entry fits in the old table
        if (jt.indexStride == 2) {
            if ((newEntry >= (1 << 15)) || (newEntry < -(1 << 15))) overflow = true;
        } else if (jt.indexStride == 1) {
            if ((newEntry >= (1 << 7)) || (newEntry < -(1 << 7))) overflow = true;
        }
    }
    return overflow;
}

void JumpTableMover::fillNewTableEntries(codeGen& gen, Address jumpAddr, int indexStride) {
    for (auto& it : newTable) {
        const Address& addr = it.first;
        const int64_t& newEntry = it.second.second;
        relocation_cerr << "\tnew entry at " << hex << it.first << " with value " << dec << newEntry << endl;
        auto reloc_it = relocs.find(gen.currAddr());
        if (reloc_it != relocs.end()) {
            reloc_it->second->setAddend(newEntry);
            relocation_cerr << "\t\t modify relocation entry" << endl;
        }
        switch (indexStride) {
            case 8: {
                gen.copy(&newEntry, sizeof(int64_t));
                break;
            }
            case 4: {
                int32_t entry = (int32_t) newEntry;
                gen.copy(&entry, sizeof(int32_t));
                break;
            }
            case 2: {
                int16_t entry = (int16_t) newEntry;
                gen.copy(&entry, sizeof(int16_t));
                break;
            }
            case 1: {
                int8_t entry = (int8_t) newEntry;
                gen.copy(&entry, sizeof(int8_t));
                break;
            }
            default: {
                fprintf(stderr, "Unhandled jump table stride %d for indirect jump %lx\n", indexStride, jumpAddr);
                assert(0);
            }
        }
    }

}

Address JumpTableMover::findRelocatedVersionedAddress(func_instance* func, int version, Address orig) {
    block_instance* block = func->getBlock(orig, version);
    if (block == NULL) return 0;
    // We try to find the exact indirect jump instruction,
    // so we pass false as the last parameter to skip potential
    // instrumentation.
    return as->getRelocPreAddr(orig, block, func, false);
}

Address JumpTableMover::findRelocatedBlockStart(func_instance* func, PatchAPI::PatchBlock* b) {
    block_instance* block = static_cast<block_instance*>(b);
    if (block == NULL) return 0;
    // When finding the start of a relocated block, we need to include
    // potential instrumentation.
    return as->getRelocPreAddr(block->start(), block, func, true);
}

static bool insnReadsPC(const InstructionAPI::Instruction& i) {
    static MachRegister pc = MachRegister::getPC(i.getArch());
    std::set<InstructionAPI::RegisterAST::Ptr> regs;
    i.getReadSet(regs);
    for (auto r : regs) {
        if (r->getID() == pc) return true;
    }
    return false;
}

bool JumpTableMover::modifyJumpTargetBaseInstructions(
    func_instance* func,
    int version,
    Address newBase,
    const PatchAPI::PatchFunction::PatchJumpTableInstance& jt
) {
    // This function assumes aarch64
    Graph::Ptr g = jt.formatSlice;
    NodeIterator nbegin, nend;
    g->exitNodes(nbegin, nend);
    SliceNode::Ptr indJumpNode;
    for (; nbegin != nend; ++nbegin) {
        indJumpNode = boost::static_pointer_cast<SliceNode>(*nbegin);
    }
    queue<SliceNode::Ptr> queue;
    queue.push(indJumpNode);

    bool findJumpBase = false;
    while (!findJumpBase && !queue.empty()) {
        SliceNode::Ptr cur = queue.front();
        queue.pop();
        cur->ins(nbegin, nend);
        for (; nbegin != nend; ++nbegin) {
            SliceNode::Ptr s = boost::static_pointer_cast<SliceNode>(*nbegin);
            if (insnReadsPC(s->assign()->insn())) {
                findJumpBase = true;
                Address relocated_insn_addr = findRelocatedVersionedAddress(func, version, s->assign()->addr());
                relocation_cerr << "\t modify instruction " << s->assign()->insn().format() << " at " << hex
                    << s->assign()->addr() << ", relocated to " << relocated_insn_addr << ", to reference " << newBase << dec
                    << " version " << version << endl;

                codeGen gen;
                gen.invalidate();
                gen.allocate(s->assign()->insn().size() * 2);
                gen.setAddrSpace(as);
                gen.setAddr(relocated_insn_addr);

                instruction ugly_insn(s->assign()->insn().ptr(), (gen.width() == 8));
                insnCodeGen::modifyData(newBase, ugly_insn, gen);

                // The next relocated instruction may be emitted to compensate
                // the PC adjustment. If it maps back to the same original instruction,
                // we need to void this instruction.
                Address origNextAddr = 0;
                std::vector<func_instance*> origFuncs;
                baseTramp* bt;
                as->getAddrInfo(relocated_insn_addr + 4, origNextAddr, origFuncs, bt);
                if (origNextAddr == s->assign()->addr()) {
                    relocation_cerr << "\t next relocated instruction has the same original address, change it to nop" << endl;
                    insnCodeGen::generateNOOP(gen, 4);
                }
                codeGens.emplace_back(gen);
                break;
            }
            queue.push(s);
        }
    }
    return findJumpBase;
}

bool JumpTableMover::modifyJumpTableBaseInstructions(
    func_instance* func,
    int version,
    Address newBase,
    const PatchAPI::PatchFunction::PatchJumpTableInstance& jt
) {
    // Assume x86-64
    Address origBase = jt.tableStart;
    Graph::Ptr g = jt.formatSlice;
    NodeIterator nbegin, nend;
    g->exitNodes(nbegin, nend);
    SliceNode::Ptr indJumpNode;
    for (; nbegin != nend; ++nbegin) {
        indJumpNode = boost::static_pointer_cast<SliceNode>(*nbegin);
    }
    queue<SliceNode::Ptr> queue;
    queue.push(indJumpNode);

    bool findMemoryRead = false;
    bool findJumpTableBase = false;
    while (!findJumpTableBase && !queue.empty()) {
        SliceNode::Ptr cur = queue.front();
        queue.pop();
        const InstructionAPI::Instruction &i = cur->assign()->insn();
        if (i.readsMemory()) findMemoryRead = true;
        if (!findMemoryRead) continue;
        uint32_t newVal;
        if (insnReadsPC(i)) {
            findJumpTableBase = true;
            uint32_t origVal = *((const uint32_t*)((const char*)(i.ptr()) + i.size() - 4));
            newVal = newBase - origBase + origVal;
        } else if (i.size() > 4) {
            uint32_t origVal = *((const uint32_t*)((const char*)(i.ptr()) + i.size() - 4));
            if (origVal == origBase) {
                findJumpTableBase = true;
                newVal = newBase;
            }
        }
        if (findJumpTableBase) {
            Address relocated_insn_addr = findRelocatedVersionedAddress(func, version, cur->assign()->addr());
            relocation_cerr << "\t modify instruction " << cur->assign()->insn().format() << " at " << hex
                << cur->assign()->addr() << ", relocated to " << relocated_insn_addr << ", to reference new table at " << newBase << dec
                << " version " << version << endl;

            codeGen gen;
            gen.invalidate();
            gen.allocate(4);
            gen.setAddrSpace(as);
            gen.setAddr(relocated_insn_addr + i.size() - 4);
            gen.copy(&newVal, 4);
            codeGens.emplace_back(gen);
            break;
        }
        cur->ins(nbegin, nend);
        for (; nbegin != nend; ++nbegin) {
            queue.push(boost::static_pointer_cast<SliceNode>(*nbegin));
        }
    }
    return findJumpTableBase;
}

bool JumpTableMover::checkFitInAndAdjust(
    Address& newJumpTargetBase,
    const PatchAPI::PatchFunction::PatchJumpTableInstance& jt
) {
    bool first = true;
    Address minAddr = 0;
    Address maxAddr = 0;
    int64_t minVal, maxVal;
    minVal = maxVal = 0;
    for (const auto& new_entry : newTable) {
        const int64_t& val = new_entry.second.second;
        const Address& addr = new_entry.second.first;
        if (first) {
            first = false;
            minAddr = maxAddr = addr;
            minVal = maxVal = val;
        }
        if (addr < minAddr) {
            minAddr = addr;
            minVal = val;
        }
        if (addr > maxAddr) {
            maxAddr = addr;
            maxVal = val;
        }
    }
    int64_t range = maxVal - minVal;
    if (jt.indexStride == 2) {
        if (range >= (1 << 16)) return true;
    } else if (jt.indexStride == 1) {
        if (range >= (1 << 8)) return true;
    }
    int64_t newTableValueBase = 0;
    if (jt.isZeroExtend) {
        // If the jump table is unsigned, a table entry is in the range
        // of [0, MAX]. To have the maximal jump range, we set the jump
        // base to the minimal target address
        newJumpTargetBase = minAddr;
        newTableValueBase = minVal;
    } else {
        // If the jump table is signed, a table entry is in the range
        // of [-MAX-1, MAX]. To have the maximal jump range, we set the jump
        // base to the mid point of the range
        newTableValueBase = (minVal + maxVal + 1) / 2;
        // Currently only on aarch64, we have table overflow,
        // and on aarch64, jump table values are left shifted two bits adding to the jump base
        newJumpTargetBase = minAddr - (minVal - newTableValueBase) * 4;
    }
    // We adjust the new table entry based on the new table value base
    for (auto& it : newTable) {
        it.second.second -= newTableValueBase;
    }
    return false;
}

AST::Ptr NewTableEntryVisitor::visit(DataflowAPI::RoseAST *ast) {
    unsigned totalChildren = ast->numChildren();

    bool isImmValue[2];
    isImmValue[0] = isImmValue[1] = false;
    int64_t immValue[2];
    for (unsigned i = 0; i < totalChildren; ++i) {
        AST::Ptr child = ast->child(i);
        if (child->getID() == AST::V_ConstantAST) {
            isImmValue[i] = true;
            child->accept(this);
            immValue[i] = curImmValue;
        }
    }

    switch (ast->val().op) {
        case ROSEOperation::addOp:
            if (isImmValue[0]) newValue -= immValue[0]; else newValue -= immValue[1];
            break;
        case ROSEOperation::invertOp:
            newValue = ~newValue;
            break;
        case ROSEOperation::sMultOp:
        case ROSEOperation::uMultOp:
            if (isImmValue[0]) newValue /= immValue[0]; else newValue /= immValue[1];
            break;
        case ROSEOperation::shiftLOp:
        case ROSEOperation::rotateLOp:
            assert(isImmValue[1]);
            newValue >>= immValue[1];
            break;
        case ROSEOperation::shiftROp:
        case ROSEOperation::rotateROp:
            assert(isImmValue[1]);
            newValue <<= immValue[1];
            break;
        case ROSEOperation::derefOp:
            return AST::Ptr();
        case ROSEOperation::andOp:
            // Here the andOp is outside the memory reference.
            // Assume it is on ppc64le or aarch64 where the andOp is used to mask the
            // lower 2 bit. So, it does not really impact the calculation.
            break;
        default:
            assert(0);
            break;
    }

    for (unsigned i = 0 ; i < totalChildren; ++i) {
        ast->child(i)->accept(this);
    }
    return AST::Ptr();
}

AST::Ptr NewTableEntryVisitor::visit(DataflowAPI::ConstantAST *ast) {
    const Constant &v = ast->val();
    int64_t value = v.val;
    if (v.size != 1 && v.size != 64 && (value & (1ULL << (v.size - 1)))) {
        // Compute the two complements in bits of v.size
	// and change it to a negative number
        value = -(((~value) & ((1ULL << v.size) - 1)) + 1);
    }
    curImmValue = value;
    return AST::Ptr();
}


