#if !defined(_R_JUMP_TABLE_MOVER_H)
#define _R_JUMP_TABLE_MOVER_H

#include "dyninstAPI/src/codegen.h"
#include "dyninstAPI/src/function.h"
#include "DynAST.h"
#include "Symtab.h"

namespace Dyninst{

namespace PatchAPI{
class PatchBlock;
class PatchFunction;
};

namespace Relocation {

class JumpTableMover {
public:
    typedef boost::shared_ptr<JumpTableMover> Ptr;
    static Ptr create(FuncSetOrderdByLayout::const_iterator begin,
                      FuncSetOrderdByLayout::const_iterator end,
                      AddressSpace * as);
    vector<codeGen> codeGens;    
private:
    JumpTableMover(AddressSpace* s): as(s) {}
    AddressSpace* as;
    std::set<func_instance*> processed_funcs;
    std::map<Address, std::pair<Address, int64_t> > newTable;    

    void moveJumpTableInFunction(func_instance*);
    Address findRelocatedVersionedAddress(func_instance*, int, Address);
    Address findRelocatedBlockStart(func_instance*, PatchAPI::PatchBlock*);
    void moveOneJumpTable(func_instance*, PatchAPI::PatchBlock*, const PatchAPI::PatchFunction::PatchJumpTableInstance&);
    bool computeNewTableEntries(func_instance*, PatchAPI::PatchBlock*, const PatchAPI::PatchFunction::PatchJumpTableInstance&);
    void fillNewTableEntries(codeGen&, Address, int);
    bool modifyJumpTargetBaseInstructions(func_instance*, int, Address, const PatchAPI::PatchFunction::PatchJumpTableInstance&);
    bool modifyJumpTableBaseInstructions(func_instance*, int, Address, const PatchAPI::PatchFunction::PatchJumpTableInstance&);
    void setupRelocationEntries();
    bool checkFitInAndAdjust(Address&, const PatchAPI::PatchFunction::PatchJumpTableInstance&);
    
    std::map<Address, SymtabAPI::relocationEntry*> relocs;
};

class NewTableEntryVisitor: public ASTVisitor {
public:
    using ASTVisitor::visit;
    int64_t newValue;
    int64_t curImmValue;

    NewTableEntryVisitor(Address r) : newValue(r) {}
    virtual ASTPtr visit(DataflowAPI::RoseAST *ast);
    virtual ASTPtr visit(DataflowAPI::ConstantAST *ast);

};

};
};
#endif
