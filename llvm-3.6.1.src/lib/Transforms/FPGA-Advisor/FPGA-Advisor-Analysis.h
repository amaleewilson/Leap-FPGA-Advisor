//===- FPGA-Advisor-Analysis.h - Main FPGA-Advisor pass definition -------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the class declarations for all the analysis
// that are useful for the FPGA-Advisor-Analysis.

#ifndef LLVM_LIB_TRANSFORMS_FPGA_ADVISOR_ANALYSIS_H
#define LLVM_LIB_TRANSFORMS_FPGA_ADVISOR_ANALYSIS_H

#include "llvm/Pass.h"
#include "llvm/PassManager.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/Analysis/CallGraph.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/Analysis/DependenceAnalysis.h"
#include "llvm/Analysis/MemoryDependenceAnalysis.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/InstVisitor.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/TypeBuilder.h"
#include "llvm/IR/Dominators.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/CommandLine.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <vector>
#include <unordered_map>
#include <map>
#include <list>
#include <string>

using namespace llvm;

namespace {
typedef struct {
	std::vector<Loop*> subloops;
	uint64_t maxIter;
	uint64_t parIter;
} LoopIterInfo;

typedef struct {
	Function *function;
	LoopInfo *loopInfo;
	std::vector<BasicBlock *> bbList;
	std::vector<Instruction *> instList;
	std::vector<LoopIterInfo> loopList;
	std::vector<LoadInst *> loadList;
	std::vector<StoreInst *> storeList;
} FunctionInfo;

typedef struct {
	BasicBlock *basicblock;
	uint64_t ID;
	int cycStart;
	int cycEnd;
} BBSchedElem;

// Graph type:
// STL list container for OutEdge List
// STL vector container for vertices
// Use directed edges
//typedef boost::adjacency_list< boost::listS, boost::vecS, boost::directedS > digraph;
// trace graph property
//typedef boost::property<boost::vertex_index_t, BBSchedElem> VertexProperty;
//typedef boost::adjacency_list< boost::listS, boost::vecS, boost::directedS, VertexProperty > TraceGraph;
typedef boost::adjacency_list< boost::listS, boost::vecS, boost::directedS, BBSchedElem > TraceGraph;
typedef std::list<TraceGraph> TraceGraphList; 
typedef std::map<Function *, TraceGraphList> ExecGraph;

// iterators
typedef TraceGraph::vertex_iterator TraceGraph_iterator;
typedef TraceGraphList::iterator TraceGraphList_iterator; 
typedef ExecGraph::iterator ExecGraph_iterator;

// vertex descriptor
typedef TraceGraph::vertex_descriptor TraceGraph_descriptor;

// edge iterators
typedef TraceGraph::out_edge_iterator TraceGraph_out_edge_iterator;
typedef TraceGraph::edge_iterator TraceGraph_edge_iterator;

//typedef std::map<Function *, std::list<std::list<BBSchedElem> > > ExecTrace;
//typedef std::list<std::list<BBSchedElem> > FuncExecTrace;
//typedef std::list<BBSchedElem> Trace;
//typedef ExecTrace::iterator ExecTrace_iterator;
//typedef FuncExecTrace::iterator FuncExecTrace_iterator;
//typedef Trace::iterator Trace_iterator;

class AdvisorAnalysis : public ModulePass, public InstVisitor<AdvisorAnalysis> {
	public:
		static char ID;
		void getAnalysisUsage(AnalysisUsage &AU) const override {
			AU.setPreservesAll();
			AU.addRequired<CallGraphWrapperPass>();
			AU.addRequired<LoopInfo>();
			AU.addRequired<DominatorTreeWrapperPass>();
			AU.addRequired<MemoryDependenceAnalysis>();
		}
		AdvisorAnalysis() : ModulePass(ID) {}
		bool runOnModule(Module &M);
		void visitFunction(Function &F);
		void visitBasicBlock(BasicBlock &BB);
		void visitInstruction(Instruction &I);

	private:
		// functions
		void find_recursive_functions(Module &M);
		void does_function_recurse(Function *func, CallGraphNode *CGN, std::vector<Function *> &stack);
		void print_recursive_functions();
		bool run_on_function(Function *F);
		bool has_unsynthesizable_construct(Function *F);
		bool is_recursive_function(Function *F);
		bool has_recursive_call(Function *F);
		bool does_function_call_recursive_function(CallGraphNode *CGN);
		bool has_external_call(Function *F);
		bool does_function_call_external_function(CallGraphNode *CGN);
		//void instrument_function(Function *F);
		//void instrument_basicblock(BasicBlock *BB);

		void print_statistics();

		bool get_program_trace(std::string fileIn);
		bool check_trace_sanity();
		BasicBlock *find_basicblock_by_name(std::string funcName, std::string bbName);
		Function *find_function_by_name(std::string funcName);

		// functions that do analysis on trace
		bool find_maximal_configuration_for_all_calls(Function *F);
		bool find_maximal_configuration_for_call(Function *F, TraceGraphList_iterator graph_it);
		bool basicblock_is_dependent(BasicBlock *child, BasicBlock *parent, TraceGraph &graph);
		bool instruction_is_dependent(Instruction *inst1, Instruction *inst2);
		bool true_dependence_exists(Instruction *inst1, Instruction *inst2);
		bool basicblock_control_flow_dependent(BasicBlock *child, BasicBlock *parent, TraceGraph &graph);

		// define some data structures for collecting statistics
		std::vector<Function *> functionList;
		std::vector<Function *> recursiveFunctionList;
		//std::vector<std::pair<Loop *, bool> > loopList;

		// recursive and external functions are included
		std::unordered_map<Function *, FunctionInfo *> functionMap;
	
		Module *mod;
		CallGraph *callGraph;

		raw_ostream *outputLog;

		// exeuctionTrace contains the execution traces separated by function
		// the value for each key (function) is a vector, where each vector element
		// represents the basicblock execution of one call to that function
		//std::map<Function *, std::list<std::list<BBSchedElem> > > executionTrace;

		ExecGraph executionGraph;

}; // end class AdvisorAnalysis
} // end anonymous namespace

char AdvisorAnalysis::ID = 0;
static RegisterPass<AdvisorAnalysis> X("fpga-advisor-analysis", "FPGA-Advisor Analysis Pass -- to be executed after instrumentation and program run", false, false);

#endif
