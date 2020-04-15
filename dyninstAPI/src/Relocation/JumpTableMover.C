#include "JumpTableMover.h"
#include "dyninstAPI/src/addressSpace.h" 
#include "dyninstAPI/src/function.h"
#include "dyninstAPI/src/debug.h"

#include "SymEval.h"


using namespace std;
using namespace Dyninst;
using namespace Dyninst::DataflowAPI;
using namespace Relocation;

JumpTableMover::Ptr JumpTableMover::create(FuncSet::const_iterator begin,
        FuncSet::const_iterator end,
        AddressSpace *s) {
    Ptr ret = Ptr(new JumpTableMover(s));                      

    for (; begin != end; ++begin) {
        func_instance *func = *begin;
        if (!func->isInstrumentable()) {
            continue;
        }
        ret->moveJumpTableInFunction(func);
    }

    return ret;
}

static std::set<Address> overwritten;

void JumpTableMover::moveJumpTableInFunction(func_instance *func) {
    parse_func * f = func->ifunc();
    const map<Address,ParseAPI::Function::JumpTableInstance>& jt_list = f->getJumpTables();
    for (auto jit = jt_list.begin(); jit != jt_list.end(); ++jit) {
        const ParseAPI::Function::JumpTableInstance &jt = jit->second;
        codeGen gen;
        gen.invalidate();
        gen.allocate(jt.tableEnd - jt.tableStart);
        gen.setAddrSpace(as);
        gen.setAddr(jt.tableStart);

        for (Address addr = jt.tableStart; addr < jt.tableEnd; addr += jt.indexStride) {
            if (overwritten.find(addr) != overwritten.end()) {
                fprintf(stderr, "ERROR: jump table relocation twice for address %lx\n", addr);
            }
            overwritten.insert(addr);
            // 1. Calculate the original target
            JumpTableEntryVisitor jtev(as, addr, jt.isZeroExtend, jt.memoryReadSize);
            jt.jumpTargetExpr->accept(&jtev);
            Address orig = jtev.targetAddress;
            
            // 2. Lookup the relocated address
            Address reloc = findRelocatedAddress(func, orig);
            if (reloc == 0) {
                fprintf(stderr, "Cannot find relocated address for %lx for jump table at %lx for function %s at %lx\n", orig, jit->first, func->name().c_str(), func->addr());
            }
            assert(reloc);

            // 3. Calculate the new table entry
            NewTableEntryVisitor ntev(reloc);
            jt.jumpTargetExpr->accept(&ntev);
            int64_t newEntry = ntev.newValue;
            //fprintf(stderr, "Relocation jump table entry from %lx to %lx for jump table at %lx for function %s at %lx, new entry value %lx, %d %d\n",
            //        orig, reloc, jit->first, func->name().c_str(), func->addr(), newEntry, jt.indexStride, jt.memoryReadSize);
            //fprintf(stderr, "\t%s\n", jt.jumpTargetExpr->format().c_str());

            // 4. Put the new table entry to a codegen object
            // TODO: assume the new entry fits in the table.
            //       Note that we are having a new jump target,
            //       so the offset can change and may not necessarilly
            //       fit in the original table entry. This can be fixed
            //       by copying the whole table to a new location
            GET_PTR(insn,gen);
            if (jt.indexStride == 8) {
                *((int64_t*)insn) = newEntry;
                insn += sizeof(int64_t);
            } else if (jt.indexStride == 4) {
                *((int32_t*)insn) = (int32_t)newEntry;
                insn += sizeof(int32_t);
            } else {
                fprintf(stderr, "Unhandled jump table stride %d for indirect jump %lx\n", jt.indexStride, jit->first);
                assert(0);
            }
            SET_PTR(insn,gen);
        }
        newTables.push_back(gen);
    }    
}

Address JumpTableMover::findRelocatedAddress(func_instance* func, Address orig) {
    block_instance* block = func->getBlock(orig);
    if (block == NULL) return 0;
    return as->getRelocPreAddr(orig, block, func);
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


