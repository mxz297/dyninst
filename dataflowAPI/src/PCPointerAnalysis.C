#include "PCPointerAnalysis.h"
#include "debug_dataflow.h"
#include "CFG.h"
#include "Instruction.h"

using namespace std;
using namespace Dyninst;
using namespace ParseAPI;
using namespace InstructionAPI;
using namespace DataflowAPI;

PCPointerAnalyzer::PCPointerAnalyzer(ParseAPI::Function* func) {
    f = func;
    analyze();
}

bool PCPointerAnalyzer::queryPostInstructionValue(Address addr, MachRegister reg, Address &val) {
    auto it = instructionData.find(addr);
    if (it == instructionData.end()) return false;
    return it->second.query(reg, val);
}

bool PCPointerAnalyzer::queryBlockInputValue(Block* b, MachRegister reg, Address& val) {
    auto it = blockInputData.find(b);
    if (it == blockInputData.end()) return false;
    return it->second.query(reg, val);
}

void PCPointerAnalyzer::analyze() {
    bool done = false;
    while (!done) {
        done = true;
        for (auto b : f->blocks()) {
            PCPointerFact newBlockInput;
            if (pathJoin(b, newBlockInput)) done = false;

            Block::Insns insns;
            b->getInsns(insns);
            Address prevAddr = 0;
            for (auto & it : insns) {
                Instruction &i = it.second;
                Address addr = it.first;
                PCPointerFact newInsnFact;
                if (prevAddr == 0) {
                    if (analyzeInstruction(addr, i, newBlockInput, newInsnFact))
                        done = false;
                } else {
                    if (analyzeInstruction(addr, i, instructionData[prevAddr], newInsnFact))
                        done = false;
                }
                prevAddr = addr;
            }
        }
    }
}

bool PCPointerAnalyzer::pathJoin(ParseAPI::Block *b, PCPointerFact& fact) {
    for (auto e : b->sources()) {
        if (e->interproc()) continue;
        PCPointerFact& d = instructionData[e->src()->last()];
        fact.join(d);
    }
    if (blockInputData[b] == fact) return false;
    blockInputData[b] = fact;
    return true;
}

bool PCPointerAnalyzer::analyzeInstruction(Address& addr,
        Instruction& i,
        PCPointerFact& input,
        PCPointerFact& output) {

    // First, check if the instruction generate a PC-relative value
    MachRegister target;
    Address val;
    output = input;
    if (analyzePCRelativeInstruction(addr, i, target, val) ||
        analyzeArithmeticInstruction(input, i, target, val)) {
        pcpointer_printf("Generate new value %s at %lx, reg %s, val %lx\n",
                i.format().c_str(), addr, target.name().c_str(), val);
        output.update(target, val);
    } else {
        std::set<RegisterAST::Ptr> regs;
        i.getWriteSet(regs);
        for (auto &r : regs) {
            output.setTop(r->getID().getBaseRegister());
        }
    }
    if (output == instructionData[addr]) return false;
    instructionData[addr] = output;
    return true;
}

bool PCPointerAnalyzer::analyzePCRelativeInstruction(Address& cur,
        InstructionAPI::Instruction& i,
        MachRegister& reg,
        Address& val) {
    entryID e = i.getOperation().getID();
    if (e == aarch64_op_adrp ||
        e == aarch64_op_adr) {
        // Get the register that will have the PC-relative value
        std::set<RegisterAST::Ptr> regs;
        i.getWriteSet(regs);
        for (auto &r : regs) {
            reg = r->getID().getBaseRegister();
        }

        // Get the PC-relative value
        Expression::Ptr thePC(new RegisterAST(MachRegister::getPC(i.getArch())));
        vector<Operand> operands;
        i.getOperands(operands);
        for (auto& o : operands) {
            // If we can bind the PC, then we're in the operand we want.
            Expression::Ptr exp = o.getValue();
            if (exp->bind(thePC.get(), Result(s64, cur))) {
                // Bind succeeded, eval to get target address
                Result res = exp->eval();
                assert(res.defined);
                val = res.convert<Address>();
                // adrp clears the bottom 12 bits
                if (e == aarch64_op_adrp) val = (val >> 12) << 12;
            }
        }
        return true;
    }   
    return false;
}

bool PCPointerAnalyzer::analyzeArithmeticInstruction(PCPointerFact& fact,
        InstructionAPI::Instruction& i,
        MachRegister& reg,
        Address& val) {
    entryID e = i.getOperation().getID();
    if (e == aarch64_op_add_addsub_imm) {
        MachRegister srcReg;
        Address inputVal;
        std::set<RegisterAST::Ptr> regs;
        i.getReadSet(regs);
        for (auto &r : regs) {
            srcReg = r->getID().getBaseRegister();
        }
        if (!fact.query(srcReg, inputVal)) return false;
        regs.clear();
        i.getWriteSet(regs);
        for (auto &r : regs) {
            reg = r->getID().getBaseRegister();
        }
        int64_t imm = 0;

        vector<Operand> operands;
        i.getOperands(operands);
        for (auto& o : operands) {
            Expression::Ptr exp = o.getValue();
            Result res = exp->eval();
            if (!res.defined) continue;
            imm = res.convert<int64_t>();
        }
        val = inputVal + imm;
        return true;
    }
    return false;
}

bool PCPointerFact::operator==(const PCPointerFact& rhs) const {
    return (top == rhs.top) && (val == rhs.val);
}

void PCPointerFact::update(const MachRegister& reg, const Address &addr) {
    top.erase(reg);
    val[reg] = addr;
}

void PCPointerFact::setTop(const MachRegister& reg) {
    val.erase(reg);
    top.insert(reg);
}

void PCPointerFact::join(PCPointerFact& rhs) {
    // Every top register in rhs will still be top
    for (auto& r : rhs.top) {
        setTop(r);
    }

    for (auto& it : val) {
        const MachRegister & reg = it.first;
        const Address & addr = it.second;

        // A top register remains top
        if (top.find(reg) != top.end()) continue;

        if (val.find(reg) == val.end()) {
            // A bottom register is set to have the rhs value
            val.emplace(reg, addr);
        } else if (val[reg] != addr) {
            // A register that has different values on different paths
            // is set to be top. We have a flat lattice
            setTop(reg);
        }
    }
}

bool PCPointerFact::query(const MachRegister& r, Address &addr) {
    auto it = val.find(r);
    if (it == val.end()) return false;
    addr = it->second;
    return true;
}
