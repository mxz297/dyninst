#include <iostream>
#include <stdexcept>
#include <stdio.h>
#include <string>

#include "InterModuleCallGraph.h"
#include "CodeObject.h"
#include "CodeSource.h"

using namespace std;
using namespace Dyninst;
using namespace ParseAPI;
using namespace DataflowAPI;

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
            PLTMap[PLTit.second] = std::vector<PLTInfo>();             
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
                PLTInfo info;
                info.obj = co;
                info.f = f;

                /* A symbol can have multiple definitions due to symbol versioning.                
                * Example, 
                * 1030: 000000000013e5a0    38 FUNC    GLOBAL DEFAULT   12 pthread_cond_broadcast@GLIBC_2.2.5
                * 1031: 000000000010c2b0    38 FUNC    GLOBAL DEFAULT   12 pthread_cond_broadcast@@GLIBC_2.3.2
                *
                * Here pthread_cond_broadcast has two different versions.
                * Ideally, our implementation should reolsve symbol versioning as the dynamic loader.            
                */
                plt_it->second.emplace_back(info);
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

bool InterModuleCallGraphAnalyzer::queryPLTCallTargets(std::string& name, std::vector<PLTInfo>& targets) {
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