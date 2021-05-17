#include "PatchInlineSnippet.h"
#include "PatchLabel.h"

#include "Visitor.h"
#include "Expression.h"

using namespace std;
using namespace Dyninst;
using namespace PatchAPI;

// Use asmjit to emit x86_64 instructions

static std::map<Dyninst::MachRegister, asmjit::x86::Gp> dyninstToasmjitRegMap = {
    {x86_64::rax, asmjit::x86::rax}, {x86_64::rbx, asmjit::x86::rbx}, {x86_64::rcx, asmjit::x86::rcx},
    {x86_64::rdx, asmjit::x86::rdx}, {x86_64::rsp, asmjit::x86::rsp}, {x86_64::rbp, asmjit::x86::rbp},
    {x86_64::rsi, asmjit::x86::rsi}, {x86_64::rdi, asmjit::x86::rdi}, {x86_64::r8, asmjit::x86::r8},
    {x86_64::r9, asmjit::x86::r9},   {x86_64::r10, asmjit::x86::r10}, {x86_64::r11, asmjit::x86::r11},
    {x86_64::r12, asmjit::x86::r12}, {x86_64::r13, asmjit::x86::r13}, {x86_64::r14, asmjit::x86::r14},
    {x86_64::r15, asmjit::x86::r15}
};

class AssemblerHolder {
   public:

   AssemblerHolder();
   ~AssemblerHolder();
   asmjit::x86::Assembler* GetAssembler() { return assembler_; }
   asmjit::StringLogger* GetStringLogger() { return logger_; }
   asmjit::CodeHolder* GetCode() { return code_; }
   bool CopyToDyninstBuffer(Dyninst::Buffer &buf);

   private:

   asmjit::JitRuntime* rt_;
   asmjit::CodeHolder* code_;
   asmjit::x86::Assembler* assembler_;
   asmjit::StringLogger* logger_;
};

// Error handler that just prints the error and lets AsmJit ignore it.
class PrintErrorHandler : public asmjit::ErrorHandler {
   public:
   // Return `true` to set last error to `err`, return `false` to do nothing.
   void handleError(asmjit::Error , const char* message, asmjit::BaseEmitter*) override {
      std::cerr << "ERROR : " << message;
   }
};

AssemblerHolder::AssemblerHolder() {
   rt_ = new asmjit::JitRuntime();

   code_ = new asmjit::CodeHolder;
   code_->init(rt_->codeInfo());
   code_->setErrorHandler(new PrintErrorHandler());

   logger_ = new asmjit::StringLogger();
   code_->setLogger(logger_);

   assembler_ = new asmjit::x86::Assembler(code_);
}

AssemblerHolder::~AssemblerHolder() {
   delete assembler_;
   delete code_;
   delete rt_;
}

bool AssemblerHolder::CopyToDyninstBuffer(Dyninst::Buffer &buf) {
   size_t size = GetCode()->codeSize();
   char* temp_buf = (char*)malloc(size);

   GetCode()->relocateToBase((uint64_t)temp_buf);

   size = GetCode()->codeSize();
   GetCode()->copyFlattenedData(temp_buf, size, asmjit::CodeHolder::kCopyWithPadding);

   buf.copy(temp_buf, size);
   free(temp_buf);
   return true;
}


bool AdjustSPSnippet::generate(Dyninst::PatchAPI::Point* , Dyninst::Buffer& buf) {
   AssemblerHolder ah;
   asmjit::x86::Assembler* a = ah.GetAssembler();
   a->lea(asmjit::x86::rsp, asmjit::x86::ptr(asmjit::x86::rsp, adjustment, 8));
   return ah.CopyToDyninstBuffer(buf);
}

// Converts memory operand from InstructionAPI representation to asmjit representation
class MemoryEffectiveAddressVisitor : public Dyninst::InstructionAPI::Visitor {
   public:
   virtual void visit(Dyninst::InstructionAPI::BinaryFunction* b) {
      if(b->isAdd()) {
         if (!immStack.empty()) {
            a.setOffset(getLastImm());
         }
         if (!regStack.empty()) {
            a.setBase(getLastReg());
         }
         if (!regStack.empty()) {
            a.setIndex(getLastReg(), 0);
         }
      } else if(b->isMultiply()) {
         assert(!immStack.empty());
         assert(!regStack.empty());
         long val = getLastImm();
         int scale = 0;
         switch (val) {
            case 8: scale = 3; break;
            case 4: scale = 2; break;
            case 2: scale = 1; break;
            case 1: scale = 0; break;
            default: assert(0);
         }
         a.setIndex(getLastReg(), scale);
      } else {
         assert(0);
      }
   }
   virtual void visit(Dyninst::InstructionAPI::Dereference*) {}
   virtual void visit(Dyninst::InstructionAPI::Immediate* i) { immStack.emplace_back(i); }
   virtual void visit(Dyninst::InstructionAPI::RegisterAST* r) {
      assert(!r->getID().isPC());
      regStack.emplace_back(r);
   }
   MemoryEffectiveAddressVisitor(asmjit::x86::Mem& access) : a(access) {}
   void finalize() {
      if (!regStack.empty()) {
         a.setBase(getLastReg());
      }
   }
   private:
   asmjit::x86::Mem &a;

   std::vector<Dyninst::InstructionAPI::Immediate*> immStack;
   std::vector<Dyninst::InstructionAPI::RegisterAST*> regStack;

   asmjit::x86::Gp getLastReg() {
      Dyninst::InstructionAPI::RegisterAST* r = regStack.back();
      regStack.pop_back();
      auto rit = dyninstToasmjitRegMap.find(r->getID().getBaseRegister());
      assert(rit != dyninstToasmjitRegMap.end());
      return rit->second;
   }

   long getLastImm() {
      Dyninst::InstructionAPI::Immediate* imm = immStack.back();
      immStack.pop_back();
      return imm->eval().convert<long>();
   }
};

bool EmulatedReturnAddressSnippet::generate(Dyninst::PatchAPI::Point* , Dyninst::Buffer& buf) {
   AssemblerHolder ah;
   asmjit::x86::Assembler* a = ah.GetAssembler();
   a->mov(asmjit::x86::ptr(asmjit::x86::rsp, 0, 8), 0);
   ah.CopyToDyninstBuffer(buf);
   Dyninst::PatchAPI::PatchLabel::generateALabel(f, b, buf.curAddr() - 4);
   return true;
}

// Implementation to emit indirect call promotion
bool IndirectCallInlineSnippet::generate(Dyninst::PatchAPI::Point* , Dyninst::Buffer& buf) {
   AssemblerHolder ah;
   asmjit::x86::Assembler* a = ah.GetAssembler();
   asmjit::x86::Gp reg;

   // Assume indirect calls conform to ABI
   if (i.readsMemory()) {
      // rax should be dead at a call site
      reg = asmjit::x86::rax;
      a->mov(asmjit::x86::rax, GetCallTargetMemory());
   } else {
      // Direct reuse the register that contains the call target
      reg = GetCallTargetRegister();
   }

   asmjit::Label indCall = a->newLabel();
   // We emit either a linear cmp-je sequence
   // or a binary search sequence
   if (addrs.size() > 5) {
      sort(addrs.begin(), addrs.end());
      asmjit::Label start = a->newLabel();
      EmitBinarySearch(a, reg, 0, addrs.size() - 1, start, indCall);
   } else {
      for (auto addr : addrs) {
         a->cmp(reg, addr);
         // Provide a large displacment to ensure the
         // target is outside the code region
         a->je(2048);
      }
   }
   a->bind(indCall);
   a->call(reg);
   return ah.CopyToDyninstBuffer(buf);
}

asmjit::x86::Gp IndirectCallInlineSnippet::GetCallTargetRegister() {
   std::vector<Dyninst::InstructionAPI::Operand> ops;
   i.getOperands(ops);
   auto o = ops[0];

   std::set<InstructionAPI::RegisterAST::Ptr> regsRead;
   o.getReadSet(regsRead);
   assert(regsRead.size() == 1);
   for (auto r : regsRead) {
      MachRegister reg = r->getID().getBaseRegister();
      auto it = dyninstToasmjitRegMap.find(reg);
      assert(it != dyninstToasmjitRegMap.end());
      return it->second;
   }
   return asmjit::x86::Gp();
}

asmjit::x86::Mem IndirectCallInlineSnippet::GetCallTargetMemory() {
   std::set<Dyninst::InstructionAPI::Expression::Ptr> memAccess;
   i.getMemoryReadOperands(memAccess);
   assert(memAccess.size() == 1);
   for (auto m : memAccess) {
      asmjit::x86::Mem a;
      MemoryEffectiveAddressVisitor meav(a);
      m->apply(&meav);
      meav.finalize();
      return a;
   }
   return asmjit::x86::Mem();
}

void IndirectCallInlineSnippet::EmitBinarySearch(
   asmjit::x86::Assembler* a,
   asmjit::x86::Gp & reg,
   size_t l,
   size_t r,
   asmjit::Label &cur,
   asmjit::Label &indCall
) {
   a->bind(cur);
   if (r - l < 3) {
      for (size_t i = l; i <= r; i++) {
         a->cmp(reg, addrs[i]);
         a->je(2048);
      }
      a->jmp(indCall);
      return;
   }
   size_t m = (l + r) / 2;
   a->cmp(reg, addrs[m]);
   // Provide a large displacment to ensure the
   // target is outside the code region
   a->je(2048);
   asmjit::Label above = a->newLabel();
   a->ja(above);
   asmjit::Label below = a->newLabel();
   EmitBinarySearch(a, reg, l, m - 1, below, indCall);
   EmitBinarySearch(a, reg, m + 1, r, above, indCall);
}