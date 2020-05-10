#if !defined(_R_JUMP_TABLE_MOVER_H)
#define _R_JUMP_TABLE_MOVER_H

#include "dyninstAPI/src/codegen.h"
#include "dyninstAPI/src/function.h"
#include "DynAST.h"

namespace Dyninst{
namespace Relocation {

class JumpTableMover {
public:
    typedef boost::shared_ptr<JumpTableMover> Ptr;
    static Ptr create(FuncSetOrderdByLayout::const_iterator begin,
                      FuncSetOrderdByLayout::const_iterator end,
                      AddressSpace * as);
    vector<codeGen> newTables;

    void moveJumpTableInFunction(func_instance*);
    Address findRelocatedAddress(func_instance*, Address);
private:
    JumpTableMover(AddressSpace* s): as(s) {}
    AddressSpace* as;
};

class JumpTableEntryVisitor: public ASTVisitor {
public:
    using ASTVisitor::visit;
    AddressSpace *as;
    Address targetAddress;
    Address readAddress;
    int memoryReadSize;
    bool isZeroExtend;


    // This tracks the results of computation for each sub AST
    std::map<AST*, int64_t> results;
    JumpTableEntryVisitor(AddressSpace*, Address mem, bool ze, int m);
    virtual ASTPtr visit(DataflowAPI::RoseAST *ast);
    virtual ASTPtr visit(DataflowAPI::ConstantAST *ast);
    virtual ASTPtr visit(DataflowAPI::VariableAST *ast);
    int64_t ReadTableEntry();
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
