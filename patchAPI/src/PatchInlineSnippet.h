#include "Snippet.h"
#include "asmjit/asmjit.h"

namespace Dyninst {

namespace PatchAPI {

class PatchFunction;
class PatchBlock;

class AdjustSPSnippet : public Dyninst::PatchAPI::Snippet {
   public:
   explicit AdjustSPSnippet(int adj): adjustment(adj) {}
   bool generate(Dyninst::PatchAPI::Point* , Dyninst::Buffer& buf) override;
   private:
   int adjustment;
};

class EmulatedReturnAddressSnippet : public Dyninst::PatchAPI::Snippet {
   public:
   explicit EmulatedReturnAddressSnippet(PatchFunction* func, PatchBlock* targetBlock):
               f(func), b(targetBlock) {}

   bool generate(Dyninst::PatchAPI::Point* , Dyninst::Buffer& buf) override;
   PatchBlock* getBlock() { return b; }

   private:
   PatchFunction *f;
   PatchBlock* b;
};

class IndirectCallInlineSnippet : public Dyninst::PatchAPI::Snippet {
   public:
   explicit IndirectCallInlineSnippet(const InstructionAPI::Instruction &insn, std::vector<Address> &addresses):
      i(insn), addrs(addresses) {}
   bool generate(Dyninst::PatchAPI::Point* , Dyninst::Buffer& buf) override;
   private:

   const InstructionAPI::Instruction& i;
   std::vector<Address>& addrs;
   std::vector<asmjit::Label> labels;

   asmjit::x86::Gp GetCallTargetRegister();
   asmjit::x86::Mem GetCallTargetMemory();

   void EmitBinarySearch(
      asmjit::x86::Assembler* a,
      asmjit::x86::Gp & reg,
      size_t l,
      size_t r,
      asmjit::Label &cur,
      asmjit::Label &indCall
   );
};

}

}