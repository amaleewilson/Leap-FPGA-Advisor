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
#include <boost/graph/breadth_first_search.hpp>
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


// TraceGraph vertex property struct representing each individual
// scheduling element (basic block granularity)
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

// TraceGraph edge weight property representing transition delay
// between fpga and cpu
typedef boost::property<boost::edge_weight_t, unsigned> TransitionDelay;
// Graph type:
// STL list container for OutEdge List
// STL vector container for vertices
// Use directed edges
//typedef boost::adjacency_list< boost::listS, boost::vecS, boost::directedS > digraph;
// trace graph property
//typedef boost::property<boost::vertex_index_t, BBSchedElem> VertexProperty;
//typedef boost::adjacency_list< boost::listS, boost::vecS, boost::directedS, VertexProperty > TraceGraph;
typedef boost::adjacency_list< boost::listS, boost::vecS, boost::bidirectionalS, BBSchedElem, TransitionDelay > TraceGraph;
//typedef boost::adjacency_list< boost::listS, boost::vecS, boost::directedS, BBSchedElem > TraceGraph;
typedef std::list<TraceGraph> TraceGraphList; 
typedef std::map<Function *, TraceGraphList> ExecGraph;

// iterators
typedef TraceGraph::vertex_iterator TraceGraph_iterator;
typedef TraceGraphList::iterator TraceGraphList_iterator; 
typedef ExecGraph::iterator ExecGraph_iterator;

// vertex descriptor
typedef TraceGraph::vertex_descriptor TraceGraph_vertex_descriptor;
typedef TraceGraph::edge_descriptor TraceGraph_edge_descriptor;

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


// The FunctionAreaEstimator class performs crude area estimation for the basic blocks
// in a function
// The main goal of this class is not to determine the exact area/resources required to
// implement the design on an FPGA, the main motivation is to discourage the tool to
// suggest putting portions of designs onto the FPGA where there are limited resources
// such as operations requiring DSPs, a lot of long routes which may decrease the
// clock speed of the design, memory... ?
class FunctionAreaEstimator : public FunctionPass, public InstVisitor<FunctionAreaEstimator> {
	public:
		static char ID;
		FunctionAreaEstimator() : FunctionPass(ID) {}
		void getAnalysisUsage(AnalysisUsage &AU) const override {
			AU.setPreservesAll();
		}
		bool runOnFunction(Function &F) {
			visit(F);
			return true;
		}
		static int get_basic_block_area(std::map<BasicBlock *, int> &AT, BasicBlock *BB) {
			auto search = AT.find(BB);
			assert(search != AT.end());
			return search->second;
		}
		std::map<BasicBlock *, int> &getAreaTable() {
			return areaTable;
		}

		void visitBasicBlock(BasicBlock &BB) {
			int area = 0;
			// approximate area of basic block as a weighted sum
			// the weight is the complexity of the instruction
			// the sum is over all compute instructions
			// W = 1 + x1y1 + x2y2 + ... + xnyn
			// x1 is the complexity of the operation
			// y1 is the number of this operation existing in the basic block
			for (auto I = BB.begin(); I != BB.end(); I++) {
				area += instruction_area_complexity(I);
			}
			areaTable.insert(std::make_pair(BB.getTerminator()->getParent(), area));
		}

		// the area complexity of an instruction is determined by several factors:
		// routing and compute resources on a typical FPGA
		// NOTE: if a basic block purely consists of very basic operations
		// for example integer addition, shifts etc, we won't incur any additional
		// area costs because we want to encourage such designs for the FPGA
		// the instructions which will incur an area cost will be the following types:
		// 1) floating point operations - these are likely to be impl on the FP DSP
		//	units, which are a limited resource
		// 2) memory instructions - this highly depends on the memory architecture,
		//	but we can assume that accesses to global memory all require a lot of
		//	routing and muxing logic
		// 3) switch statement/phi nodes with a large number of inputs
		//	these we can effectively think of as muxes, if there are a large number
		//	of inputs (e.g. 8/16 or more) then the mux will be very large, which we
		//	would want to discourage (FIXME: however, this is really more of a latency
		//	issue than an area issue)
		// 4) ambiguous pointers??? TODO
		int instruction_area_complexity(Instruction *I) {
			int complexity = 0;
			if (instruction_needs_fp(I)) {
				complexity += get_fp_area_cost();
			}
			if (instruction_needs_global_memory(I)) {
				complexity += get_global_memory_area_cost();
			}
			if (instruction_needs_muxes(I)) {
				complexity += get_mux_area_cost(I);
			}
			return complexity;
		}

		bool instruction_needs_fp(Instruction *I) {
			switch(I->getOpcode()) {
				case Instruction::FAdd:
				case Instruction::FSub:
				case Instruction::FMul:
				case Instruction::FDiv:
				case Instruction::FRem:
				case Instruction::FPToUI:
				case Instruction::FPToSI:
				case Instruction::UIToFP:
				case Instruction::SIToFP:
				case Instruction::FPTrunc:
				case Instruction::FPExt:
				case Instruction::FCmp:
					return true;
					break;
				default:
					return false;
					break;
			}
			// here we don't consider call instructions that may
			// possibly return a float
		}

		bool instruction_needs_global_memory(Instruction *I) {
			// check that instruction is memory instruction
			if (!I->mayReadOrWriteMemory()) {
				return false;
			}
			// reads/writes memory, check the location of access
			// this may either be a load/store or function call
			// look into memory dependence analysis and memory location
			// which gives info about the size and starting location of
			// the location pointed to by a pointer...
			// TODO FIXME need to finish this function
			return true;
		}

		bool instruction_needs_muxes(Instruction *I) {
			// instructions that need muxing are:
			// switch instructions and phi nodes
			if (isa<SwitchInst>(I)) {
				return true;
			} else if (isa<PHINode>(I)) {
				return true;
			}
			return false;
		}

		// area estimators
		// I currently have no plan for this, will need to be calibrated
		int get_fp_area_cost() {
			return 1;
		}

		int get_global_memory_area_cost() {
			// TODO FIXME this should depend on the size of the memory location
			return 1;
		}

		int get_mux_area_cost(Instruction *I) {
			// only incur cost to large muxes
			if (SwitchInst *SwI = dyn_cast<SwitchInst>(I)) {
				// proportional to the size
				return (int) SwI->getNumCases() / 16;
			} else if (PHINode *PN = dyn_cast<PHINode>(I)) {
				// proportional to the size
				return (int) PN->getNumIncomingValues() / 16;
			}
			return 0;
		}

		std::map<BasicBlock *, int> areaTable;

}; // end class FunctionAreaEstimator


class ScheduleVisitor : public boost::default_dfs_visitor {
	public:
		int mutable lastCycle;
		TraceGraphList_iterator graph_ref;
		int *lastCycle_ref;
		std::map<BasicBlock *, int> &LT;
		ScheduleVisitor(TraceGraphList_iterator graph, std::map<BasicBlock *, int> &_LT, int &lastCycle) : graph_ref(graph), LT(_LT), lastCycle_ref(&lastCycle) {}

		void discover_vertex(TraceGraph_vertex_descriptor v, const TraceGraph &graph) const {
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

			//std::cerr << "Schedule vertex: (" << v << ") " << graph[v].basicblock->getName().str() <<
			//			" start: " << start << " end: " << end << "\n";
			(*graph_ref)[v].set_start(start);
			(*graph_ref)[v].set_end(end);

			// keep track of the last cycle as seen by the scheduler
			*lastCycle_ref = std::max(*lastCycle_ref, end);
			//std::cerr << "LastCycle: " << *lastCycle_ref << "\n";
		}
}; // end class ScheduleVisitor


class ConstrainedScheduleVisitor : public boost::default_bfs_visitor {
	public:
		int mutable lastCycle;
		TraceGraph *graph_ref;
		int *lastCycle_ref;
		int *cpuCycle_ref;
		std::map<BasicBlock *, int> &LT;
		std::map<BasicBlock *, std::pair<bool, std::vector<unsigned> > > &resourceTable;
		ConstrainedScheduleVisitor(TraceGraph &graph, std::map<BasicBlock *, int> &_LT, int &lastCycle, int &cpuCycle, std::map<BasicBlock *, std::pair<bool, std::vector<unsigned> > > &_resourceTable) : graph_ref(&graph), LT(_LT), lastCycle_ref(&lastCycle), cpuCycle_ref(&cpuCycle),  resourceTable(_resourceTable) {}

		void discover_vertex(TraceGraph_vertex_descriptor v, const TraceGraph &graph) const {
			// find the latest finishing parent
			// if no parent, start at 0
			int start = -1;
			TraceGraph_in_edge_iterator ii, ie;
			for (boost::tie(ii, ie) = boost::in_edges(v, graph); ii != ie; ii++) {
				int transitionDelay = (int) boost::get(boost::edge_weight_t(), graph, *ii);
				start = std::max(start, graph[boost::target(*ii, graph)].cycEnd + transitionDelay);
			}
			start += 1;

			// this differs from the maximal parallelism configuration scheduling
			// in that it also considers resource requirement
			
			// first sort the vector
			auto search = resourceTable.find(graph[v].basicblock);
			//assert(search != resourceTable.end());
			if (search == resourceTable.end()) {
				// if not found, could mean that either
				// a) basic block to be executed on cpu
				// b) resource table not initialized properly o.o
				std::cerr << "Basic block " << graph[v].name << " not found in resource table.\n";
				assert(0);
			}

			bool cpu = (search->second).first;
			int resourceReady = UINT_MAX;
			std::vector<unsigned> &resourceVector = search->second.second;
			if (cpu) { // cpu resource flag
				resourceReady = *cpuCycle_ref;
			} else {
				std::sort(resourceVector.begin(), resourceVector.end());
				resourceReady = resourceVector.front();
			}

			start = std::max(start, resourceReady);

			int end = start;
			end += FunctionScheduler::get_basic_block_latency(LT, graph[v].basicblock);
			
			// update the occupied resource with the new end cycle
			if (cpu) {
				*cpuCycle_ref = end;
			} else {
				resourceVector.front() = end;
			}

			//std::cerr << "Schedule vertex: " << graph[v].basicblock->getName().str() <<
			//			" start: " << start << " end: " << end << "\n";
			(*graph_ref)[v].set_start(start);
			(*graph_ref)[v].set_end(end);

			// keep track of last cycle as seen by scheduler
			*lastCycle_ref = std::max(*lastCycle_ref, end);
			//std::cerr << "LastCycle: " << *lastCycle_ref << "\n";
		}
}; // end class ConstrainedScheduleVisitor


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
			AU.addRequired<FunctionAreaEstimator>();
		}
		AdvisorAnalysis() : ModulePass(ID) {}
		bool runOnModule(Module &M);
		void visitFunction(Function &F);
		void visitBasicBlock(BasicBlock &BB);
		void visitInstruction(Instruction &I);
		static int get_basic_block_instance_count(BasicBlock *BB);

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
		bool find_maximal_configuration_for_call(Function *F, TraceGraphList_iterator graph_it, std::vector<TraceGraph_vertex_descriptor> &rootVertices);
		bool basicblock_is_dependent(BasicBlock *child, BasicBlock *parent, TraceGraph &graph);
		bool instruction_is_dependent(Instruction *inst1, Instruction *inst2);
		bool true_dependence_exists(Instruction *inst1, Instruction *inst2);
		bool basicblock_control_flow_dependent(BasicBlock *child, BasicBlock *parent, TraceGraph &graph);
		void find_new_parents(std::vector<TraceGraph_vertex_descriptor> &newParents, TraceGraph_vertex_descriptor child, TraceGraph_vertex_descriptor parent, TraceGraph &graph);
		bool annotate_schedule_for_call(Function *F, TraceGraphList_iterator graph_it, std::vector<TraceGraph_vertex_descriptor> &rootVertices, int &lastCycle);
		bool find_maximal_resource_requirement(Function *F, TraceGraphList_iterator graph_it, std::vector<TraceGraph_vertex_descriptor> &rootVertices, int lastCycle);
		void modify_resource_requirement(Function *F, TraceGraphList_iterator graph_it);
		void find_optimal_configuration_for_all_calls(Function *F);
		int incremental_gradient_descent(Function *F, BasicBlock *&removeBB);
		bool decrement_basic_block_instance_count(BasicBlock *BB);
		bool increment_basic_block_instance_count(BasicBlock *BB);
		void find_root_vertices(std::vector<TraceGraph_vertex_descriptor> &roots, TraceGraphList_iterator graph_it);
		unsigned schedule_with_resource_constraints(std::vector<TraceGraph_vertex_descriptor> &roots, TraceGraphList_iterator graph_it, Function *F);
		void initialize_resource_table(Function *F, std::map<BasicBlock *, std::pair<bool, std::vector<unsigned> > > &resourceTable);
		unsigned get_area_requirement(Function *F);
		void update_transition_delay(TraceGraphList_iterator graph);
		unsigned get_transition_delay(BasicBlock *source, BasicBlock *target, bool CPUToHW);

		void print_basic_block_configuration(Function *F);

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

// put after AdvisorAnalysis class -- uses a function from class
// TraceGraph custom vertex writer for execution trace graph output to dotfile
template <class TraceGraph>
class TraceGraphVertexWriter {
	public:
		TraceGraphVertexWriter(TraceGraph& _graph) : graph(_graph) {}
		template <class TraceGraph_vertex_descriptor>
		void operator()(std::ostream& out, const TraceGraph_vertex_descriptor &v) const {
			/*
			out << "[shape = \"record\" label=\"<r0 fontcolor=Red> " << graph[v].cycStart << "| <r1>"
				<< graph[v].name << "| <r2>"
				<< graph[v].cycEnd << "\"]";
			*/
			out << "[shape=\"none\" label=<<table border=\"0\" cellspacing=\"0\">";
			out	<< "<tr><td bgcolor=\"green\" border=\"1\"> " << graph[v].cycStart << "</td></tr>";
			if (AdvisorAnalysis::get_basic_block_instance_count(graph[v].basicblock) > 0) {
				out	<< "<tr><td bgcolor=\"gray\" border=\"1\"> " << graph[v].name << "</td></tr>";
			} else {
				out	<< "<tr><td border=\"1\"> " << graph[v].name << "</td></tr>";
			}
			out	<< "<tr><td bgcolor=\"cyan\" border=\"1\"> " << graph[v].cycEnd << "</td></tr>";
			out	<< "</table>>]";
		}
	private:
		TraceGraph &graph;
}; // end class TraceGraphVertexWriter

template <class TraceGraph>
class TraceGraphEdgeWriter {
	public:
		TraceGraphEdgeWriter(TraceGraph& _graph) : graph(_graph) {}
		template <class TraceGraph_edge_descriptor>
		void operator()(std::ostream& out, const TraceGraph_edge_descriptor &e) const {
			unsigned delay = boost::get(boost::edge_weight_t(), graph, e);
			if (delay > 0) {
				out << "[color=\"blue\" penwidth=\"3\" label=\"" << delay << "\"]";
			}
		}
	private:
		TraceGraph &graph;
}; // end class TraceGraphEdgeWriter



} // end fpga namespace

#endif
