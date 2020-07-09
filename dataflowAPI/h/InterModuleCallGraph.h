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
    std::string path;
    dyn_hash_map<std::string, ParseAPI::CodeObject*> objectMap;
    dyn_hash_map<ParseAPI::CodeObject*, std::string> pathMap;
    dyn_hash_map<std::string, std::vector<ParseAPI::Function*> >PLTMap;

    dyn_hash_map<ParseAPI::Function*, std::set<ParseAPI::Function*> > src, trg;
    dyn_hash_map<ParseAPI::Function*, int> sccIndex;

    void findInputDependencies(std::vector<std::string>&);
    void retrievePLTs();
    void resolvePLTs();
    bool analyzeSymbols(ParseAPI::CodeObject*, dyn_hash_map<Address, std::set<std::string> >&);
    void buildCallGraphFromFunction(ParseAPI::Function*, bool);
    void buildSCC();

    dyn_hash_map<ParseAPI::Function*, bool> nodeColor;
    std::vector<ParseAPI::Function*> reverseOrder;
    int orderStamp;

    void NaturalDFS(ParseAPI::Function*);
    void ReverseDFS(ParseAPI::Function*);

public:
    InterModuleCallGraphAnalyzer(std::string);
    ~InterModuleCallGraphAnalyzer();
    bool queryPLTCallTargets(std::string&, std::vector<ParseAPI::Function*>&);
    bool queryObjectPath(ParseAPI::CodeObject*, std::string&);
    ParseAPI::CodeObject* getMainObject();
    void getAllObjects(std::vector<ParseAPI::CodeObject*>&);
    void buildCallGraphFromMainObject(bool);
    void getCallGraphSCC(std::vector< std::vector<ParseAPI::Function* > > &);
    std::set<ParseAPI::Function*>& getCalleeFunctions(ParseAPI::Function*);
    std::set<ParseAPI::Function*>& getCallerFunctions(ParseAPI::Function*);
};

}

}
#endif
