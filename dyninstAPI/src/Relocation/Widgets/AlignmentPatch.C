#include "AlignmentPatch.h"
#include "Widget.h"

#include "../CodeBuffer.h"

using namespace Dyninst;
using namespace Relocation;
using namespace InstructionAPI;

static std::vector< std::vector<unsigned char> > nops = {
    {},
    {0x90}, // 1-byte nop
    {}, // 2-byte nop
    {0x0f, 0x1f, 0x00}, // 3-byte nop
    {0x0f, 0x1f, 0x40, 0x00}, // 4-byte nop
    {0x0f, 0x1f, 0x44, 0x00, 0x00}, // 5-byte nop
    {0x66, 0x0f, 0x1f, 0x44, 0x00, 0x00}, // 6-byte nop
    {0x0f, 0x1f, 0x80, 0x00, 0x00, 0x00, 0x00}, // 7-byte nop
    {0x0f, 0x1f, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00}, // 8-byte nop
    {0x66, 0x0f, 0x1f, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00}, // 9-byte nop
    {0x66, 0x2e, 0x0f, 0x1f, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00} // 10-byte nop
};

bool AlignmentPatch::apply(codeGen &gen, CodeBuffer *buf) {
    Address cur = gen.currAddr();
    if (((cur >> n) << n) == cur) return true;
    unsigned total = (((cur >> n) + 1) << n) - cur;
    if (total == 0) return true;
    while (total > 0) {
        int nop_size;
        if (total >= 10) {
            nop_size = 10;
        } else if (total == 2) {
            nop_size = 1;
        } else {
            nop_size = total;
        }
        gen.copy(nops[nop_size]);
        total -= nop_size;
    }
    return true;
}
