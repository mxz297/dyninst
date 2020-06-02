#include "AlignmentPatch.h"
#include "Widget.h"

#include "../CodeBuffer.h"

using namespace Dyninst;
using namespace Relocation;
using namespace InstructionAPI;

bool AlignmentPatch::apply(codeGen &gen, CodeBuffer *buf) {
    Address cur = gen.currAddr();
    if (((cur >> n) << n) == cur) return true;
    unsigned total = (((cur >> n) + 1) << n) - cur;
    if (total == 0) return true;
    std::vector<unsigned char> vec(total, 0x90);
    gen.copy(vec);
    return true;
}
