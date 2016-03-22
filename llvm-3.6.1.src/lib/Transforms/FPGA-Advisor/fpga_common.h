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
//===----------------------------------------------------------------------===//
// Author: chenyuti
//===----------------------------------------------------------------------===//
//
// This file contains the shared typedefs for FPGA analyses

#ifndef LLVM_LIB_TRANSFORMS_FPGA_ADVISOR_COMMON_H
#define LLVM_LIB_TRANSFORMS_FPGA_ADVISOR_COMMON_H

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
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/graphviz.hpp>

#include <algorithm>
#include <vector>
#include <unordered_map>
#include <map>
#include <list>
#include <string>

using namespace llvm;

namespace fpga {

// Dependence Graph type:
// STL list container for OutEdge list
// STL vector container for vertices
// Use directed edges
typedef boost::adjacency_list< boost::listS, boost::vecS, boost::bidirectionalS, BasicBlock * >
		DepGraph;
typedef DepGraph::vertex_iterator DepGraph_iterator;
typedef DepGraph::vertex_descriptor DepGraph_descriptor;
typedef DepGraph::out_edge_iterator DepGraph_out_edge_iterator;
typedef DepGraph::in_edge_iterator DepGraph_in_edge_iterator;
typedef DepGraph::edge_iterator DepGraph_edge_iterator;

class DependenceGraph : public FunctionPass {

	public:
		static char ID;
		void getAnalysisUsage(AnalysisUsage &AU) const override {
			AU.setPreservesAll();
			AU.addRequired<DominatorTreeWrapperPass>();
			AU.addRequired<MemoryDependenceAnalysis>();
		}
		DependenceGraph() : FunctionPass(ID) {
			//initializeDependenceGraph(*PassRegistry::getPassRegistry());
		}
		bool runOnFunction(Function &F);
		DepGraph &getDepGraph() {
			return DG;
		}
		static DepGraph_descriptor get_vertex_descriptor_for_basic_block(BasicBlock *BB, DepGraph &DG);
		static bool is_basic_block_dependent(BasicBlock *BB1, BasicBlock *BB2, DepGraph &DG);
	
	private:
		void add_vertices(Function &F);
		void add_edges();
		void insert_dependent_basic_block(std::vector<BasicBlock *> &list, BasicBlock *BB);
		void insert_dependent_basic_block_all(std::vector<BasicBlock *> &list);
		void insert_dependent_basic_block_all_memory(std::vector<BasicBlock *> &list);
		bool unsupported_memory_instruction(Instruction *I);

		Function *func;
		MemoryDependenceAnalysis *MDA;
		DominatorTree *DT;
		DepGraph DG;
		std::vector<std::string> NameVec;
		// a list of basic blocks that may read or write memory
		//std::vector<DepGraph_descriptor> MemoryBBs;
		std::vector<BasicBlock *> MemoryBBs;
}; // end class DependenceGraph

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
	public:
		void set_start(int _start) const { cycStart = _start;}
		void set_end(int _end) const { cycEnd = _end;}
	BasicBlock *basicblock;
	uint64_t ID;
	int mutable cycStart;
	int mutable cycEnd;
	std::string name;
} BBSchedElem;

// Graph type:
// STL list container for OutEdge List
// STL vector container for vertices
// Use directed edges
//typedef boost::adjacency_list< boost::listS, boost::vecS, boost::directedS > digraph;
// trace graph property
//typedef boost::property<boost::vertex_index_t, BBSchedElem> VertexProperty;
//typedef boost::adjacency_list< boost::listS, boost::vecS, boost::directedS, VertexProperty > TraceGraph;
typedef boost::adjacency_list< boost::listS, boost::vecS, boost::bidirectionalS, BBSchedElem > TraceGraph;
//typedef boost::adjacency_list< boost::listS, boost::vecS, boost::directedS, BBSchedElem > TraceGraph;
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
typedef TraceGraph::in_edge_iterator TraceGraph_in_edge_iterator;
typedef TraceGraph::edge_iterator TraceGraph_edge_iterator;

//typedef std::map<Function *, std::list<std::list<BBSchedElem> > > ExecTrace;
//typedef std::list<std::list<BBSchedElem> > FuncExecTrace;
//typedef std::list<BBSchedElem> Trace;
//typedef ExecTrace::iterator ExecTrace_iterator;
//typedef FuncExecTrace::iterator FuncExecTrace_iterator;
//typedef Trace::iterator Trace_iterator;

class FunctionScheduler : public FunctionPass , public InstVisitor<FunctionScheduler> {
	public:
		static char ID;
		FunctionScheduler() : FunctionPass(ID) {}
		void getAnalysisUsage(AnalysisUsage &AU) const override {
			AU.setPreservesAll();
		}
		bool runOnFunction(Function &F) {
			visit(F);
			return true;
		}
		static int get_basic_block_latency(std::map<BasicBlock *, int> &LT, BasicBlock *BB) {
			auto search = LT.find(BB);
			assert(search != LT.end());
			return search->second;
		}
		int get_basic_block_latency(BasicBlock *BB) {
			auto search = latencyTable.find(BB);
			assert(search != latencyTable.end());
			return search->second;
		}
		std::map<BasicBlock *, int> &getLatencyTable() {
			return latencyTable;
		}
	
		void visitBasicBlock(BasicBlock &BB) {
			int latency = 0;
			// approximate latency of basic block as number of instructions
			for (auto I = BB.begin(); I != BB.end(); I++) {
				latency++;
			}
			latencyTable.insert(std::make_pair(BB.getTerminator()->getParent(), latency));
		}
	
		std::map<BasicBlock *, int> latencyTable;
	
}; // end class FunctionScheduler


class ScheduleVisitor : public boost::default_dfs_visitor {
	public:
		int mutable lastCycle;
		TraceGraph *graph_ref;
		int *lastCycle_ref;
		std::map<BasicBlock *, int> &LT;
		ScheduleVisitor(TraceGraph &graph, std::map<BasicBlock *, int> &_LT, int &lastCycle) : graph_ref(&graph), LT(_LT), lastCycle_ref(&lastCycle) {}

		void discover_vertex(TraceGraph_descriptor v, const TraceGraph &graph) const {
			// find the latest finishing parent
			// if no parent, start at 0
			int start = -1;
			TraceGraph_in_edge_iterator ii, ie;
			for (boost::tie(ii, ie) = boost::in_edges(v, graph); ii != ie; ii++) {
				start = std::max(start, graph[boost::source(*ii, graph)].cycEnd);
			}
			start += 1;

			int end = start;
			end += FunctionScheduler::get_basic_block_latency(LT, graph[v].basicblock);

			std::cerr << "Schedule vertex: " << graph[v].basicblock->getName().str() <<
						" start: " << start << " end: " << end << "\n";
			(*graph_ref)[v].set_start(start);
			(*graph_ref)[v].set_end(end);

			// keep track of the last cycle as seen by the scheduler
			*lastCycle_ref = std::max(*lastCycle_ref, end);
			std::cerr << "LastCycle: " << *lastCycle_ref << "\n";
		}
}; // end class ScheduleVisitor



class AdvisorAnalysis : public ModulePass, public InstVisitor<AdvisorAnalysis> {
	public:
		static char ID;
		void getAnalysisUsage(AnalysisUsage &AU) const override {
			AU.setPreservesAll();
			AU.addRequired<CallGraphWrapperPass>();
			AU.addRequired<LoopInfo>();
			AU.addRequired<DominatorTreeWrapperPass>();
			AU.addRequired<MemoryDependenceAnalysis>();
			AU.addRequired<DependenceGraph>();
			AU.addRequired<FunctionScheduler>();
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
		bool find_maximal_configuration_for_call(Function *F, TraceGraphList_iterator graph_it, std::vector<TraceGraph_descriptor> &rootVertices);
		bool basicblock_is_dependent(BasicBlock *child, BasicBlock *parent, TraceGraph &graph);
		bool instruction_is_dependent(Instruction *inst1, Instruction *inst2);
		bool true_dependence_exists(Instruction *inst1, Instruction *inst2);
		bool basicblock_control_flow_dependent(BasicBlock *child, BasicBlock *parent, TraceGraph &graph);
		void find_new_parents(std::vector<TraceGraph_descriptor> &newParents, TraceGraph_descriptor child, TraceGraph_descriptor parent, TraceGraph &graph);
		bool annotate_schedule_for_call(Function *F, TraceGraphList_iterator graph_it, std::vector<TraceGraph_descriptor> &rootVertices, int &lastCycle);
		bool find_maximal_resource_requirement(Function *F, TraceGraphList_iterator graph_it, std::vector<TraceGraph_descriptor> &rootVertices, int lastCycle);

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

		//DepGraph depGraph;

}; // end class AdvisorAnalysis

} // end fpga namespace

#endif
