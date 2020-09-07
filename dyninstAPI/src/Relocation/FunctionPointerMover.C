#include "FunctionPointerMover.h"
#include "dyninstAPI/src/addressSpace.h" 
#include "dyninstAPI/src/function.h"
#include "dyninstAPI/src/debug.h"
#include "dyninstAPI/src/mapped_object.h"

#include "Instruction.h"
#include "InstructionDecoder.h"
#include "Expression.h"
#include "Result.h"
#include "Register.h"

#include "PCPointerAnalysis.h"

using namespace std;
using namespace Dyninst;
using namespace Relocation;

#define PTR_SIZE (sizeof(void*))

FunctionPointerMover::Ptr FunctionPointerMover::create(AddressSpace *s) {
    Ptr ret = Ptr(new FunctionPointerMover(s));                      
    ret->movePointersInDataSection(".data");
    ret->movePointersInDataSection(".rodata");
    ret->movePointersInDataSection(".init_array");
    ret->movePointersInDataSection(".fini_array");

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
           Address newValue = movePointer(value, addr);
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

Address FunctionPointerMover::movePointer(Address addr, Address ptr_addr) {
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
    relocation_cerr << "move function pointer at " << hex << ptr_addr << " from " 
        << addr << " to " << ret << dec << endl;
    return ret;
}

static unsigned char* buf = NULL;
static int bufSize = 0;

void FunctionPointerMover::movePointersInCodeSection() {
    switch (as->getArch()) {
        case Arch_x86:
        case Arch_x86_64:
            movePointersInCodeSectionX86();
            break;
        case Arch_ppc64:
        case Arch_aarch64:
            movePointersInCodeSectionWithPCPointerAnalysis();
            break;
        default:
            break;
    }
}

void FunctionPointerMover::movePointersInCodeSectionX86() {
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
                Address newValue = movePointer(orig, curAddr);
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
                Address newValue = movePointer(orig, curAddr);
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

void FunctionPointerMover::movePointersInCodeSectionWithPCPointerAnalysis() {
    std::map<ParseAPI::Function*, DataflowAPI::PCPointerAnalyzer*> pointerAnalysisMap;
    AddressSpace::CodeTrackers& trackers = as->relocatedCode_;    
    CodeTracker* tracker = trackers.back();
    buildTrackerMap(tracker);

    for (auto code_tracker_iter = tracker->trackers().begin(); code_tracker_iter != tracker->trackers().end(); ++code_tracker_iter) {
        // Only need to rewrite fucntion pointers in original code
        const TrackerElement *t = *code_tracker_iter;
        if (t->type() != TrackerElement::original) continue;

        // Get PCPointerAnalyzer pointer
        ParseAPI::Function* parse_f = static_cast<ParseAPI::Function*>(t->func()->ifunc());
        ParseAPI::SymtabCodeSource* scs = (ParseAPI::SymtabCodeSource*)(parse_f->obj()->cs());
        Address tocBase = scs->getSymtabObject()->getObjectTOCAddress();

        auto it = pointerAnalysisMap.find(parse_f);
        DataflowAPI::PCPointerAnalyzer* pca = NULL;
        if (it == pointerAnalysisMap.end()) {
            DataflowAPI::PCPointerAnalyzer* new_pca = new DataflowAPI::PCPointerAnalyzer(parse_f);
            pointerAnalysisMap.emplace(parse_f, new_pca);
            pca = new_pca;
        } else {
            pca = it->second;
        }

        if (t->size() > bufSize) {
            if (buf != NULL) free(buf);
            buf = (unsigned char*) malloc(t->size());
            bufSize = t->size();
        }
        as->readTextSpace((const void*)t->reloc(), t->size(), (void*)buf);
        InstructionAPI::InstructionDecoder dec(buf, t->size(), as->getArch());
        InstructionAPI::Instruction ins;
        long curOffset = 0;
	    while (curOffset < t->size()) {
            ins = dec.decode();
            if (!ins.isValid()) break;
            Address relocAddr = t->reloc() + curOffset;
            Address origAddr = t->relocToOrig(relocAddr);
            curOffset += ins.size();

            // Check if the dst reg has a pre-determined value
            MachRegister dstReg;
            std::set<InstructionAPI::RegisterAST::Ptr> regs;           
            ins.getWriteSet(regs);
            for (auto &r : regs) {
                dstReg = r->getID().getBaseRegister();
            }
            if (dstReg == ppc64::r2) continue;
            Address val;
            if (!pca->queryPostInstructionValue(origAddr, dstReg, val)) continue;

            // Check if the dst reg matches a function entry
            Address newValue = movePointer(val, origAddr);
            if (newValue == 0) continue;

            MachRegister srcReg;
            regs.clear();
            ins.getReadSet(regs);
            for (auto &r : regs) {
                srcReg = r->getID().getBaseRegister();
            }
            if (srcReg == ppc64::r2) continue;
            Address origin;
            if (!pca->queryPreInstructionValueOrigin(origAddr, srcReg, origin)) continue;
            //fprintf(stderr, "original one %lx, two %lx\n", origin, origAddr);
            switch (as->getArch()) {
                case Arch_ppc64:
                    rewritePPCPointer(relocAddr, origin, newValue, tocBase);
                    break;
                case Arch_aarch64:
                    rewriteARMPointer(relocAddr, origin, newValue);
                    break;
                default:
                    fprintf(stderr, "Unsupport architecture\n");
                    assert(0);
            }
        }
    }
    for (auto& it : pointerAnalysisMap) {
        delete it.second;
    }
}

void FunctionPointerMover::rewritePPCPointer(Address reloc1, Address orig2, Address newValue, Address tocBase) {
     
    const TrackerElement *t = lookupTrackerElement(orig2);
    assert(t);
    Address reloc2 = t->origToReloc(orig2);

    //fprintf(stderr, "PPC Function pointer rewriting, high address at %lx, low address at %lx, new value %lx\n", reloc2, reloc1, newValue);

    // Assume that the reloc2 is the addis dst2, r2, IMM@high
    // and reloc1 is the addi dst1, dst2, IMM@low
    uint64_t val = newValue - tocBase;
    uint16_t high = (uint16_t)((val + 0x8000) >> 16);
    uint16_t low = (uint16_t)(val & 0xFFFF);
    
    codeGen gen1;
    gen1.invalidate();
    gen1.allocate(4);
    gen1.setAddrSpace(as);
    gen1.setAddr(reloc1);
    gen1.copy(&low, 2);
    newPointers.push_back(gen1);

    codeGen gen2;
    gen2.invalidate();
    gen2.allocate(4);
    gen2.setAddrSpace(as);
    gen2.setAddr(reloc2);
    gen2.copy(&high, 2);
    newPointers.push_back(gen2);
}

void FunctionPointerMover::rewriteARMPointer(Address reloc1, Address orig2, Address newValue) {
    const TrackerElement *t = lookupTrackerElement(orig2);
    assert(t);
    Address reloc2 = t->origToReloc(orig2);

    //fprintf(stderr, "ARM Function pointer rewriting, high address at %lx, low address at %lx, new value %lx\n", reloc2, reloc1, newValue);

#define UNSET_BITS(inst, x, y) \
    (inst) &= ~(((1UL << ((y) - (x) + 1)) - 1) << (x));
#define SET_BITS(inst, x, v) \
    (inst) |= (((uint32_t)(v)) << (x));
    // Rewrite adrp instruction
    int32_t pageOffset = (newValue >> 12) - (reloc2 >> 12);
    uint32_t inst;
    as->readTextSpace((const void*)reloc2, 4, (void*)(&inst));
    UNSET_BITS(inst, 29, 30);
    SET_BITS(inst, 29, pageOffset & 0x3);
    UNSET_BITS(inst, 5, 23);
    SET_BITS(inst, 5, (pageOffset >> 2) & 0x7FFFF);

    codeGen gen2;
    gen2.invalidate();
    gen2.allocate(4);
    gen2.setAddrSpace(as);
    gen2.setAddr(reloc2);
    gen2.copy(&inst, 4);
    newPointers.push_back(gen2);

    // Rewrite add instruction
    as->readTextSpace((const void*)reloc1, 4, (void*)(&inst));
    UNSET_BITS(inst, 10, 21);
    SET_BITS(inst, 10, newValue & 0xFFF);

    codeGen gen1;
    gen1.invalidate();
    gen1.allocate(4);
    gen1.setAddrSpace(as);
    gen1.setAddr(reloc1);
    gen1.copy(&inst, 4);
    newPointers.push_back(gen1);
#undef SET_BITS
#undef UNSET_BITS
}

void FunctionPointerMover::buildTrackerMap(CodeTracker* ct) {
    trackerMap.clear();
    for (const auto *te : ct->trackers()) {
        if (te->type() != TrackerElement::original &&
            te->type() != TrackerElement::emulated) continue;
        trackerMap[make_pair(te->orig(), te->orig() + te->size())] = te;
    }
}

const TrackerElement* FunctionPointerMover::lookupTrackerElement(Address orig) {
    auto it = trackerMap.upper_bound( make_pair(orig, std::numeric_limits<Address>::max()) );
    if (it == trackerMap.end()) return NULL;
    --it;
    if (it->first.first <= orig && orig < it->first.second) {
        return it->second;
    } else {
        return NULL;
    }
}
