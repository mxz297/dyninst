#ifndef FUNCTION_POINTER_MOVER_H
#define FUNCTION_POINTER_MOVER_H

#include "dyninstAPI/src/codegen.h"
#include "Instruction.h"

namespace Dyninst {
namespace Relocation {

class CodeTracker;
class TrackerElement;

class FunctionPointerMover {
public:
    typedef boost::shared_ptr<FunctionPointerMover> Ptr;
    static Ptr create(AddressSpace * as);
    std::vector<codeGen> newPointers;

    void movePointersInDataSection(const char* name);
    void movePointersInCodeSection();

private:
    FunctionPointerMover(AddressSpace* s): as(s) {}
    AddressSpace *as;

    Address movePointer(Address, Address);
    Address getOriginalPCAddress(InstructionAPI::Instruction&, Address);
    Address getImmediateOperand(InstructionAPI::Instruction&);
    void movePointersInCodeSectionX86();
    void movePointersInCodeSectionWithPCPointerAnalysis();

    std::map< std::pair<Address, Address>, const TrackerElement* > trackerMap;
    const TrackerElement* lookupTrackerElement(Address);
    void buildTrackerMap(CodeTracker*);
    void rewritePPCPointer(Address, Address, Address, Address);
    void rewriteARMPointer(Address, Address, Address);

};


};
};

#endif
