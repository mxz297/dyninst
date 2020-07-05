#ifndef INTER_MODULE_CALL_GRAPH_H
#define INTER_MODULE_CALL_GRAPH_H

#include <set>
#include <map>
#include <string>
#include <vector>

#include "dyn_regs.h"

namespace Dyninst{

namespace ParseAPI{
    class Function;
    class Block;
    class CodeObject;
};

namespace DataflowAPI {



class DATAFLOW_EXPORT InterModuleCallGraphAnalyzer {
public:
    struct PLTInfo {
        ParseAPI::CodeObject* obj;
        ParseAPI::Function* f;
    };

private:
    std::string path;
    dyn_hash_map<std::string, ParseAPI::CodeObject*> objectMap;
    dyn_hash_map<ParseAPI::CodeObject*, std::string> pathMap;
    dyn_hash_map<std::string, std::vector<PLTInfo> >PLTMap;

    void findInputDependencies(std::vector<std::string>&);
    void retrievePLTs();
    void resolvePLTs();
    bool analyzeSymbols(ParseAPI::CodeObject*, dyn_hash_map<Address, std::set<std::string> >&);


public:
    InterModuleCallGraphAnalyzer(std::string);
    ~InterModuleCallGraphAnalyzer();
    bool queryPLTCallTargets(std::string&, std::vector<PLTInfo>&);
    bool queryObjectPath(ParseAPI::CodeObject*, std::string&);
    ParseAPI::CodeObject* getMainObject();
    void getAllObjects(std::vector<ParseAPI::CodeObject*>&);
};

}

}
#endif
