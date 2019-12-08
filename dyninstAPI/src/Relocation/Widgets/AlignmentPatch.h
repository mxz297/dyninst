#ifndef ALIGNMENT_PATCH_H
#define ALIGNMENT_PATCH_H

#include "Widget.h"

namespace Dyninst{
namespace Relocation {

// Generate nop until the lower n bit of the
// address is 0. 
struct AlignmentPatch : public Patch {   
    int n;

    AlignmentPatch(int bit): n(bit) {}
    
    virtual bool apply(codeGen &gen, CodeBuffer *buf);
    virtual unsigned estimate(codeGen &templ) { return 0; };

};

}
}

#endif
