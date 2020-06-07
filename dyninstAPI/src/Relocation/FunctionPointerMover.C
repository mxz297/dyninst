#include "FunctionPointerMover.h"
#include "dyninstAPI/src/addressSpace.h" 
#include "dyninstAPI/src/function.h"
#include "dyninstAPI/src/debug.h"
#include "dyninstAPI/src/mapped_object.h"

#include "Instruction.h"
#include "InstructionDecoder.h"
#include "Expression.h"
#include "Result.h"

using namespace std;
using namespace Dyninst;
using namespace Relocation;

#define PTR_SIZE (sizeof(void*))

FunctionPointerMover::Ptr FunctionPointerMover::create(AddressSpace *s) {
    Ptr ret = Ptr(new FunctionPointerMover(s));                      
    ret->movePointersInDataSection(".data");
    ret->movePointersInDataSection(".rodata");
    ret->movePointersInCodeSection();
    return ret;
}

void FunctionPointerMover::movePointersInDataSection(const char *secName) {
   for (auto mit = as->mappedObjects().begin(); mit != as->mappedObjects().end(); ++mit) {
       mapped_object* mo = *mit;
       SymtabAPI::Region* dataReg = NULL;
       mo->parse_img()->getObject()->findRegion(dataReg, secName);
       if (dataReg == NULL) continue; 
       Address start = dataReg->getMemOffset();
       unsigned size = dataReg->getMemSize();
       for (unsigned off = 0; off < size; off += PTR_SIZE) {
           Address addr = start + off;
           Address value = 0;
           as->readTextSpace( (const void*)addr, PTR_SIZE, (void*)&value);      
           Address newValue = movePointer(value);
           if (newValue == 0) continue;
           codeGen gen;
           gen.invalidate();
           gen.allocate(PTR_SIZE);
           gen.setAddrSpace(as);
           gen.setAddr(addr);
           gen.copy(&newValue, PTR_SIZE);
           newPointers.push_back(gen);
       }
   }
}

Address FunctionPointerMover::movePointer(Address addr) {
    func_instance* func = as->findFuncByEntry(addr);
    if (func == NULL) return 0;
    block_instance* block = func->getBlockByEntry(addr);
    if (block == NULL) return 0;

    std::list<Address> relocs;
    as->getRelocAddrs(addr, block, func, relocs, true);
    if (relocs.size() == 0) return 0;
    Address ret = 0;
    for (auto ait = relocs.begin(); ait != relocs.end(); ++ait) {
        if (ret == 0 || ret > *ait) ret = *ait;
    }
    relocation_cerr << "move function pointer from " << hex << addr <<
        " to " << ret << dec << endl;
    return ret;
}

static unsigned char* buf = NULL;
static int bufSize = 0;

void FunctionPointerMover::movePointersInCodeSection() {
    AddressSpace::CodeTrackers& trackers = as->relocatedCode_;
    CodeTracker* tracker = trackers.back();
    for (auto code_tracker_iter = tracker->trackers().begin(); code_tracker_iter != tracker->trackers().end(); ++code_tracker_iter) {
        const TrackerElement *t = *code_tracker_iter;
        if (t->size() > bufSize) {
            if (buf != NULL) free(buf);
            buf = (unsigned char*) malloc(t->size());
            bufSize = t->size();
        }
        as->readTextSpace((const void*)t->reloc(), t->size(), (void*)buf);
        InstructionAPI::InstructionDecoder dec(buf, t->size(), as->getArch());
        InstructionAPI::Instruction ins;
	    Address curAddr = t->reloc();
        long curOffset = 0;
	    while (curOffset < t->size()) {
            ins = dec.decode();
            if (!ins.isValid()) break;
            curOffset += ins.size();
            if (ins.getOperation().getID() == e_lea) {
                Address orig = getOriginalPCAddress(ins, curAddr);
                if (orig == 0) {
                    curAddr += ins.size();
                    continue;
                }
                Address newValue = movePointer(orig);
                if (newValue == 0) {
                    curAddr += ins.size();
                    continue;
                }
                codeGen gen;
                gen.invalidate();
                gen.allocate(ins.size());
                gen.setAddrSpace(as);
                gen.setAddr(curAddr);
               
 			    instruction ugly_insn(ins.ptr(), true);
			    insnCodeGen::modifyData(newValue, ugly_insn, gen);
                newPointers.push_back(gen);
            } else {
                Address orig = getImmediateOperand(ins);
                if (orig == 0) {
                    curAddr += ins.size();
                    continue;
                }
                Address newValue = movePointer(orig);
                if (newValue == 0) {
                    curAddr += ins.size();
                    continue;
                }
                //fprintf(stderr, "Change imm for instruction %s at %lx, from %lx to %lx\n", ins.format().c_str(), curAddr, orig, newValue);
                codeGen gen;
                gen.invalidate();
                gen.allocate(ins.size());
                gen.setAddrSpace(as);
                gen.setAddr(curAddr);

                std::vector<unsigned char> byte_vec;
                const unsigned char* bytes = (const unsigned char*)ins.ptr();
                for (int i = 0; i < ins.size(); ++i)
                    byte_vec.push_back(bytes[i]);
                for (int i = 4; i >= 1; --i) {
                    unsigned char byte = newValue & 0xff;
                    newValue >>= 8;
                    byte_vec[byte_vec.size() - i] = byte;
                }
                gen.copy(byte_vec);
                newPointers.push_back(gen);
            }
	        curAddr += ins.size();
	    }
    }
}

Address FunctionPointerMover::getOriginalPCAddress(InstructionAPI::Instruction &ins, Address curAddr) {
    InstructionAPI::Expression::Ptr op = ins.getOperand(1).getValue();
    InstructionAPI::Expression::Ptr thePC(new InstructionAPI::RegisterAST(MachRegister::getPC(ins.getArch())));
    op->bind(thePC.get(), InstructionAPI::Result(InstructionAPI::u64, curAddr + ins.size()));
    InstructionAPI::Result res = op->eval();
    if (res.defined) {
        return res.convert<Address>();
    }
    return 0;
}

Address FunctionPointerMover::getImmediateOperand(InstructionAPI::Instruction &ins) {
    vector<InstructionAPI::Operand> operands;
    ins.getOperands(operands);
    for (auto oit = operands.begin(); oit != operands.end(); ++oit) {
        InstructionAPI::Expression::Ptr op = oit->getValue();
        InstructionAPI::Result res = op->eval();
        if (res.defined) {
            return res.convert<Address>();
        }
    }
    return 0;

}
