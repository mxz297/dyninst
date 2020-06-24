#include "JumpTableMover.h"
#include "dyninstAPI/src/addressSpace.h" 
#include "dyninstAPI/src/function.h"
#include "dyninstAPI/src/debug.h"

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

    for (; begin != end; ++begin) {
        func_instance *func = *begin;
        if (!func->isInstrumentable()) {
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

void JumpTableMover::moveJumpTableInFunction(func_instance *func) {
    if (processed_funcs.find(func) != processed_funcs.end()) return;
    processed_funcs.insert(func);
    parse_func * f = func->ifunc();
    map<Address,ParseAPI::Function::JumpTableInstance>& jt_list = f->getJumpTables();

    for (auto& jit : jt_list) {
        moveOneJumpTable(func, jit.first, jit.second);
    }

}

void JumpTableMover::moveOneJumpTable(func_instance* func, Address jumpAddr, ParseAPI::Function::JumpTableInstance& jt) {
    
    // First, detetermine whether the relocation will cause table entry overflow.
    bool overflow = computeNewTableEntries(func, jumpAddr, jt);
    if (overflow) {
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
        bool needNewTable = false;
        if (jt.indexStride == 2) {
            if (range >= (1 << 16)) needNewTable = true;
        } else if (jt.indexStride == 1) {
            if (range >= (1 << 8)) needNewTable = true;
        }
        assert(!needNewTable);
        Address newJumpTargetBase = 0;
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
        if (!modifyJumpTargetBase(func, newJumpTargetBase, jt)) {
            fprintf(stderr, "need to modify jump target base to %lx, but failed\n", newJumpTargetBase);
            assert(0);
        }
    }
    
    // Emit the new table
    codeGen gen;
    gen.invalidate();
    gen.allocate(jt.tableEnd - jt.tableStart);
    gen.setAddrSpace(as);
    gen.setAddr(jt.tableStart);
    fillNewTableEntries(gen, jumpAddr, jt.indexStride);
    codeGens.emplace_back(gen);
}

bool JumpTableMover::computeNewTableEntries(func_instance* func, Address jumpAddr, ParseAPI::Function::JumpTableInstance& jt) {
    bool overflow = false;
    // Old table entry address -> pair of relocated target address and new table entry value
    newTable.clear();
    
    relocation_cerr << "Relocation jump table at " << hex << jumpAddr
                << " for function " << func->name() << " at " << func->addr() << dec
                << " table stride " << jt.indexStride
                << " memory read size " << jt.memoryReadSize
                << " memory zero extend " << jt.isZeroExtend << endl;

    for (Address addr = jt.tableStart; addr < jt.tableEnd; addr += jt.indexStride) {
        // 1. Calculate the original target
        JumpTableEntryVisitor jtev(as, addr, jt.isZeroExtend, jt.memoryReadSize);
        jt.jumpTargetExpr->accept(&jtev);
        Address orig = jtev.targetAddress;
            
        // 2. Lookup the relocated address
        Address reloc = findRelocatedAddress(func, orig);
            
        // For ppc64le, we check relocated addresses using both
        // global entry function and local entry function.
        if (reloc == 0 && func->getNoPowerPreambleFunc() != nullptr) {
            reloc = findRelocatedAddress(func->getNoPowerPreambleFunc(), orig);
        }
        if (reloc == 0 && func->getPowerPreambleFunc() != nullptr) {
            reloc = findRelocatedAddress(func->getPowerPreambleFunc(), orig);
        }

        if (reloc == 0) {
            fprintf(stderr, "Cannot find relocated address for %lx for jump table at %lx for function %s at %lx\n", orig, jumpAddr, func->name().c_str(), func->addr());
        }
        assert(reloc);

        // 3. Calculate the new table entry
        NewTableEntryVisitor ntev(reloc);
        jt.jumpTargetExpr->accept(&ntev);
        int64_t newEntry = ntev.newValue;
        relocation_cerr << "\t table address " << hex << addr
            << " original target address " << orig
            << " new target address " << reloc
            << " new entry value " << dec << newEntry << endl;

        // 4. Record new target and new entry value
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
        
        // Detect conflict writes to the same address
        // from different jump tabels
        auto oit = overwritten.find(addr);
        if (oit != overwritten.end() && oit->second != newEntry) {
            fprintf(stderr, "ERROR: jump table relocation twice for address %lx\n", addr);
        } else {
            overwritten.emplace(addr, newEntry);
        }
    }    

}

Address JumpTableMover::findRelocatedAddress(func_instance* func, Address orig) {
    block_instance* block = func->getBlock(orig);
    if (block == NULL) return 0;
    return as->getRelocPreAddr(orig, block, func);
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

bool JumpTableMover::modifyJumpTargetBase(func_instance* func, Address newBase, ParseAPI::Function::JumpTableInstance& jt) {
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
                Address relocated_insn_addr = findRelocatedAddress(func, s->assign()->addr());
                relocation_cerr << "\t modify instruction " << s->assign()->insn().format() << " at " << hex
                    << s->assign()->addr() << ", relocated to " << relocated_insn_addr << ", to reference " << newBase << dec << endl;
                
                codeGen gen;
                gen.invalidate();
                gen.allocate(s->assign()->insn().size() * 2);
                gen.setAddrSpace(as);
                gen.setAddr(relocated_insn_addr);

                instruction ugly_insn(s->assign()->insn().ptr(), (gen.width() == 8));
                insnCodeGen::modifyData(newBase, ugly_insn, gen);
                // Currently, we do jump table relocation after normal relocation is done.
                // So, this PC relative is compensated to reference original jump base,
                // which does not fit in one instruction
                insnCodeGen::generateNOOP(gen, 4);
                codeGens.emplace_back(gen);
                break;
            }
            queue.push(s);
        }
    }
    return true;
}

JumpTableEntryVisitor::JumpTableEntryVisitor(AddressSpace* s, Address mem, bool ze, int m) {
    as = s;
    readAddress = mem;
    isZeroExtend = ze;
    memoryReadSize = m;
}

AST::Ptr JumpTableEntryVisitor::visit(DataflowAPI::RoseAST *ast) {
    unsigned totalChildren = ast->numChildren();
    for (unsigned i = 0 ; i < totalChildren; ++i) {
        ast->child(i)->accept(this);
    }

    switch (ast->val().op) {
        case ROSEOperation::addOp:
	    results.insert(make_pair(ast, results[ast->child(0).get()] + results[ast->child(1).get()]));
	    break;
	case ROSEOperation::invertOp:
	    results.insert(make_pair(ast, ~results[ast->child(0).get()]));
	    break;
	case ROSEOperation::andOp: 
	    results.insert(make_pair(ast, results[ast->child(0).get()] & results[ast->child(1).get()]));
	    break;
	case ROSEOperation::sMultOp:
	case ROSEOperation::uMultOp:
	    results.insert(make_pair(ast, results[ast->child(0).get()] * results[ast->child(1).get()]));
	    break;
	case ROSEOperation::shiftLOp:
	case ROSEOperation::rotateLOp:
	    results.insert(make_pair(ast, results[ast->child(0).get()] << results[ast->child(1).get()]));
	    break;
	case ROSEOperation::shiftROp:
	case ROSEOperation::rotateROp:
	    results.insert(make_pair(ast, results[ast->child(0).get()] >> results[ast->child(1).get()]));
	    break;
	case ROSEOperation::derefOp: 
		results.insert(make_pair(ast, ReadTableEntry()));
	    break;
	case ROSEOperation::orOp: 
	    results.insert(make_pair(ast, results[ast->child(0).get()] | results[ast->child(1).get()]));
	    break;
	default:
        assert(0);
	    break;
    }
    targetAddress = results[ast];
    if (as->getAddressWidth() == 4) {
        targetAddress &= 0xffffffff;
    }

    return AST::Ptr();
   
}

AST::Ptr JumpTableEntryVisitor::visit(DataflowAPI::ConstantAST *ast) {
    const Constant &v = ast->val();
    int64_t value = v.val;
    if (v.size != 1 && v.size != 64 && (value & (1ULL << (v.size - 1)))) {
        // Compute the two complements in bits of v.size
	// and change it to a negative number
        value = -(((~value) & ((1ULL << v.size) - 1)) + 1);
    }
    results.insert(make_pair(ast, value));
    return AST::Ptr();
}


AST::Ptr JumpTableEntryVisitor::visit(DataflowAPI::VariableAST * var) {
    return AST::Ptr();
}


#define SIGNEX_64_32 0xffffffff00000000LL
#define SIGNEX_64_16 0xffffffffffff0000LL
#define SIGNEX_64_8  0xffffffffffffff00LL
#define SIGNEX_32_16 0xffff0000
#define SIGNEX_32_8 0xffffff00    

int64_t JumpTableEntryVisitor::ReadTableEntry() {
    Address addr = readAddress;
    int addressWidth = as->getAddressWidth();
    int64_t v = 0;
    as->readTextSpace((const void*)addr, memoryReadSize, (void*) &v);

    switch (memoryReadSize) {
        case 8:
            break;
        case 4:
            if (isZeroExtend) break;
            if ((addressWidth == 8) && (v & 0x80000000)) {
                v |= SIGNEX_64_32;
            }
            break;
        case 2:
            if (isZeroExtend) break;
            if ((addressWidth == 8) && (v & 0x8000)) {
                v |= SIGNEX_64_16;
            }
            if ((addressWidth == 4) && (v & 0x8000)) {
                v |= SIGNEX_32_16;
            }
            break;
        case 1:
            if (isZeroExtend) break;
            if ((addressWidth == 8) && (v & 0x80)) {
                v |= SIGNEX_64_8;
            }
            if ((addressWidth == 4) && (v & 0x80)) {
                v |= SIGNEX_32_8;
            }
            break;	    
        default:
            assert(0);
    }
    return v;
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


