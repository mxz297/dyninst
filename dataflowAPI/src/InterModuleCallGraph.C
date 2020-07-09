#include <iostream>
#include <stdexcept>
#include <stdio.h>
#include <string>

#include "InterModuleCallGraph.h"
#include "CodeObject.h"
#include "CodeSource.h"

#include "Instruction.h"
#include "InstructionCategories.h"

using namespace std;
using namespace Dyninst;
using namespace ParseAPI;
using namespace DataflowAPI;
using namespace InstructionAPI;

static set<string> ignoreFuncs {
    "malloc_printerr",
    "__libc_message",
    "__cxa_allocate_exception",
    "__cxa_throw"
};

static string terminal_command(string command) {
   char buffer[128];
   string result = "";

   // Open pipe to file
   FILE* pipe = popen(command.c_str(), "r");
   if (!pipe) {
      return "popen failed!";
   }

   // read till end of process:
   while (!feof(pipe)) {

      // use buffer to read and add to result
      if (fgets(buffer, 128, pipe) != NULL)
         result += buffer;
   }

   pclose(pipe);
   return result;
}

InterModuleCallGraphAnalyzer::InterModuleCallGraphAnalyzer(string inputPath) {
    path = inputPath;
    vector<string> deps;
    deps.emplace_back(path);
    findInputDependencies(deps);

    for (auto& p : deps) {
        SymtabCodeSource *sts;
        CodeObject *co;
        sts = new SymtabCodeSource((char*)p.c_str());
        co = new CodeObject(sts);
        co->parse();
        objectMap[p] = co;
        pathMap[co] = p;
    }

    retrievePLTs();
    resolvePLTs();
}

InterModuleCallGraphAnalyzer::~InterModuleCallGraphAnalyzer() {
    for (auto &it : objectMap) {
        SymtabCodeSource *sts = (SymtabCodeSource*)it.second->cs();
        delete it.second;
        delete sts;
    }
}

void InterModuleCallGraphAnalyzer::findInputDependencies(vector<string>& deps) {
    /* Sample ldd output:
     *  linux-vdso.so.1 =>  (0x00007ffdb55f2000)
     *  libstdc++.so.6 => /projects/spack/opt/spack/linux-rhel7-x86_64/gcc-4.8.5/gcc-7.3.0-qrjpi76aeo4bysagruwwfii6oneh56lj/lib64/libstdc++.so.6 (0x00007f20d82a1000)
     *  libm.so.6 => /lib64/libm.so.6 (0x00007f20d7f9f000)
     *  libgcc_s.so.1 => /projects/spack/opt/spack/linux-rhel7-x86_64/gcc-4.8.5/gcc-7.3.0-qrjpi76aeo4bysagruwwfii6oneh56lj/lib64/libgcc_s.so.1 (0x00007f20d7d88000)
     *  libc.so.6 => /lib64/libc.so.6 (0x00007f20d79ba000)
     *  /lib64/ld-linux-x86-64.so.2 (0x00007f20d8828000)
     */
    string cmd = "ldd " + path;
    string lddOutput = terminal_command(cmd);

    std::stringstream ss(lddOutput);
    std::string to;

    while(std::getline(ss,to,'\n')) {
        size_t index = to.find("=>");
        if (index == string::npos) continue;
        size_t end_pos = to.find("(");
        if (end_pos == string::npos) continue;
        string depsPath = to.substr(index + 3, end_pos - index - 4);
        if (depsPath.empty()) continue;
        //fprintf(stderr, "find path %s\n", depsPath.c_str());
        deps.emplace_back(depsPath);
    }
}

void InterModuleCallGraphAnalyzer::retrievePLTs() {
    for (auto & it : objectMap) {
        CodeObject * co = it.second;
        for (auto & PLTit : co->cs()->linkage()) {
            if (PLTMap.find(PLTit.second) != PLTMap.end()) continue;
            PLTMap[PLTit.second] = std::vector<Function*>();
        }
    }
}

void InterModuleCallGraphAnalyzer::resolvePLTs() {
    for (auto & it : objectMap) {
        CodeObject * co = it.second;
        dyn_hash_map<Address, set<string> > globalFuncSymbols;
        analyzeSymbols(co, globalFuncSymbols);
        for (auto f : co->funcs()) {
            // Ignore function definitions in .plt.
            // We need to find actual functions
            if (co->cs()->linkage().find(f->addr()) != co->cs()->linkage().end()) continue;

            // We only use global function dynamic symbol
            // In addition, ParseAPI::Function may use demangled name.
            // We need to use mangled name.
            auto nit = globalFuncSymbols.find(f->addr());
            if (nit == globalFuncSymbols.end()) continue;

            // A global function may have several alias names.
            // For example, fwrite can also be named as _IO_fwrite
            for (auto& name : nit->second) {
                auto plt_it = PLTMap.find(name);
                if (plt_it == PLTMap.end()) continue;

                /* A symbol can have multiple definitions due to symbol versioning.
                * Example,
                * 1030: 000000000013e5a0    38 FUNC    GLOBAL DEFAULT   12 pthread_cond_broadcast@GLIBC_2.2.5
                * 1031: 000000000010c2b0    38 FUNC    GLOBAL DEFAULT   12 pthread_cond_broadcast@@GLIBC_2.3.2
                *
                * Here pthread_cond_broadcast has two different versions.
                * Ideally, our implementation should reolsve symbol versioning as the dynamic loader.
                */
                plt_it->second.emplace_back(f);
            }
        }
    }
}

bool InterModuleCallGraphAnalyzer::analyzeSymbols(ParseAPI::CodeObject* co, dyn_hash_map<Address, set<string> >& syms) {
    SymtabCodeSource *scs = (SymtabCodeSource*)(co->cs());
    SymtabAPI::Symtab *symObj = scs->getSymtabObject();

    std::vector<SymtabAPI::Symbol *> symbols;
    symObj->getAllSymbolsByType(symbols, SymtabAPI::Symbol::ST_FUNCTION);

    // How to hanlde ifunc?

    for (auto s : symbols) {
        if (!s->isInDynSymtab()) continue;
        if (s->getLinkage() == SymtabAPI::Symbol::SL_LOCAL) continue;
        if (s->getLinkage() == SymtabAPI::Symbol::SL_UNKNOWN) continue;
        syms[s->getOffset()].insert(s->getMangledName());
    }
}

bool InterModuleCallGraphAnalyzer::queryPLTCallTargets(std::string& name, std::vector<Function*>& targets) {
    auto it = PLTMap.find(name);
    if (it == PLTMap.end()) return false;
    targets = it->second;
    return !targets.empty();
}

bool InterModuleCallGraphAnalyzer::queryObjectPath(ParseAPI::CodeObject* co, std::string& p) {
    auto it = pathMap.find(co);
    if (it == pathMap.end()) return false;
    p = it->second;
    return true;
}

ParseAPI::CodeObject* InterModuleCallGraphAnalyzer::getMainObject() {
    return objectMap[path];
}

void InterModuleCallGraphAnalyzer::getAllObjects(vector<CodeObject*>& v) {
    for (auto &it : objectMap) {
        v.emplace_back(it.second);
    }
}

void InterModuleCallGraphAnalyzer::buildCallGraphFromMainObject(bool intermodule) {
    CodeObject* mainObj = getMainObject();
    for (auto f: mainObj->funcs()) {        
        buildCallGraphFromFunction(f, intermodule);
    }

    buildSCC();
}

void InterModuleCallGraphAnalyzer::buildCallGraphFromFunction(Function* func, bool intermodule) {
    if (ignoreFuncs.find(func->name()) != ignoreFuncs.end()) return;
    auto it = trg.find(func);
    auto plts = func->obj()->cs()->linkage();
    if (it != trg.end()) return;

    vector<Edge*> calls;
    for (auto b : func->blocks()) {
        for (auto e : b->targets()) {
            if (e->interproc() && e->type() != RET) {                
                if (e->type() == CALL && e->sinkEdge()) {
                    Instruction i = b->getInsn(b->last());
                    if (i.getCategory() == c_SyscallInsn) continue;
                }
                calls.emplace_back(e);
            }
        }
    }

    for (auto e : calls) {
        if (e->sinkEdge()) {
            // Find an indirect call or indirect tail call
            trg[func].insert(nullptr);
        } else if (plts.find(e->trg()->start()) != plts.end()) {
            // Find a plt call.
            bool findCallee = false;
            if (intermodule) {
                /// Get the PLT callee
                std::string & name = plts[e->trg()->start()];
                vector<Function*> targets;
                if (queryPLTCallTargets(name, targets)) {
                    for (auto callee : targets) {
                        trg[func].insert(callee);
                        src[callee].insert(func);
                        buildCallGraphFromFunction(callee, intermodule);
                    }
                    findCallee = true;
                }
            }
            if (!findCallee) {
                // We either do not build intermodule call graph, or fail to resolve PLT.
                // Treat this as an indirect call
                trg[func].insert(nullptr);
            }
        } else {
            // Find a call inside the module
            Function* callee = func->obj()->findFuncByEntry(func->region(), e->trg()->start());
            assert(callee);
            trg[func].insert(callee);
            src[callee].insert(func);
            buildCallGraphFromFunction(callee, intermodule);
        }
    }
}

void InterModuleCallGraphAnalyzer::buildSCC() {
    nodeColor.clear();
    reverseOrder.clear();
    sccIndex.clear();

    for (auto & fit : trg) {
        Function* f = fit.first;
        if (nodeColor.find(f) == nodeColor.end()) {
            NaturalDFS(f);
        }
    }

    nodeColor.clear();
    orderStamp = 0;
    for (auto fit = reverseOrder.rbegin(); fit != reverseOrder.rend(); ++fit) {
        if (nodeColor.find(*fit) == nodeColor.end()) {
            ++orderStamp;
            ReverseDFS(*fit);
        }
    }
}

void InterModuleCallGraphAnalyzer::NaturalDFS(Function* cur) {
    if (cur == nullptr) return;
    nodeColor[cur] = true;
    auto it = trg.find(cur);
    if (it != trg.end()) {
        for (auto f: it->second) {
            if (nodeColor.find(f) == nodeColor.end()) {
                NaturalDFS(f);
            }
        }
    }
    reverseOrder.emplace_back(cur);
}

void InterModuleCallGraphAnalyzer::ReverseDFS(Function* cur) {
    if (cur == nullptr) return;
    nodeColor[cur] = true;
    sccIndex[cur] = orderStamp;
    auto it = src.find(cur);
    if (it == src.end()) return;
    for (auto f : it->second) {
        if (nodeColor.find(f) == nodeColor.end()) {
            ReverseDFS(f);
        }
    }
}

void InterModuleCallGraphAnalyzer::getCallGraphSCC(std::vector< std::vector<ParseAPI::Function* > > & scc) {
    scc.clear();
    scc.resize(orderStamp);
    for (auto & fit : sccIndex) {
        scc[fit.second - 1].emplace_back(fit.first);
    }
}

std::set<ParseAPI::Function*>& InterModuleCallGraphAnalyzer::getCalleeFunctions(Function* f) {
    return trg[f];
}

std::set<ParseAPI::Function*>& InterModuleCallGraphAnalyzer::getCallerFunctions(Function* f) {
    return src[f];
}

