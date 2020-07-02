#ifndef PC_POINTER_ANALYSIS_H
#define PC_POINTER_ANALYSIS_H

#include <set>
#include <map>

#include "dyn_regs.h"

namespace Dyninst{

namespace ParseAPI{
    class Function;
    class Block;   
};

namespace InstructionAPI {
    class Instruction;
};

namespace DataflowAPI {

class DATAFLOW_EXPORT PCPointerFact {
    // Registers in this set are set to be top
    std::set<MachRegister> top;

    // Registers in this set have determined PC values
    std::map<MachRegister, Address> val;

    // Registers not in top or val are bottom
public:
    void join(PCPointerFact& rhs);
    bool operator==(const PCPointerFact& rhs) const;
    void update(const MachRegister &, const Address &);
    void setTop(const MachRegister &);
    bool query(const MachRegister&, Address&);
};

class DATAFLOW_EXPORT PCPointerAnalyzer {

    ParseAPI::Function* f;
    // Block start dataflow fact.
    // Computed by joining output from source blocks
    std::map<ParseAPI::Block*, PCPointerFact> blockInputData;

    // Dataflow fact for post-instruction
    std::map<Address, PCPointerFact> instructionData;

    void analyze();
    bool pathJoin(ParseAPI::Block*b, PCPointerFact&);
    bool analyzeInstruction(Address&,
            InstructionAPI::Instruction&,
            PCPointerFact&,
            PCPointerFact&);
    bool analyzePCRelativeInstruction(Address&,
            InstructionAPI::Instruction&,
            MachRegister&,
            Address&);
    bool analyzeArithmeticInstruction(PCPointerFact&,
            InstructionAPI::Instruction&,
            MachRegister&,
            Address&);


public:
    PCPointerAnalyzer(ParseAPI::Function*);
    bool queryPostInstructionValue(Address, MachRegister, Address&); 
    bool queryBlockInputValue(ParseAPI::Block*, MachRegister, Address&);

};

}

}
#endif
