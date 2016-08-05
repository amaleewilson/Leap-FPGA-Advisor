//===- FPGA-Advisor-Analysis.h - Main FPGA-Advisor pass definition -------*- C++ -*-===//
//
// Copyright (c) 2016, Intel Corporation
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// Neither the name of the Intel Corporation nor the names of its contributors
// may be used to endorse or promote products derived from this software
// without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
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
#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/Analysis/CallGraph.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/Analysis/DependenceAnalysis.h"
#include "llvm/Analysis/MemoryDependenceAnalysis.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Instruction.def"
#include "llvm/IR/InstVisitor.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/TypeBuilder.h"
#include "llvm/IR/Dominators.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/CommandLine.h"

// include tbb components 
#include "tbb/task.h"
#include "tbb/task_group.h"
#include "tbb/task_scheduler_init.h"
#include "tbb/concurrent_queue.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/lockfree/stack.hpp>
#include <boost/lockfree/queue.hpp>

#include <algorithm>
#include <vector>
#include <unordered_map>
#include <map>
#include <list>
#include <string>
#include <mutex>

#include <dlfcn.h>

#define SINGLE_THREAD_TID 0

using namespace llvm;

namespace fpga {

// Common helper functions
// Function: get_basic_block_instance_count
// Return: the number of instances of this basic block from metadata
/*static int get_basic_block_instance_count_meta(BasicBlock *BB) {
	assert(BB);
	std::string MDName = "FPGA_ADVISOR_REPLICATION_FACTOR_";
	MDName += BB->getName().str();
	MDNode *M = BB->getTerminator()->getMetadata(MDName);

	int repFactor = -1;

	assert(M);
	assert(M->getOperand(0));

	std::string repFactorStr = cast<MDString>(M->getOperand(0))->getString().str();
	repFactor = stoi(repFactorStr);

	//*outputLog << __func__ << " metadata: ";
	//BB->getTerminator()->print(*outputLog);
	//*outputLog << " replication factor: " << repFactor << "\n";

        std::cerr << "Basic block" << BB << " replication factor: " << repFactor << "\n";

	return repFactor;
}

static void set_basic_block_instance_count_meta(BasicBlock *BB, int value) {
	std::string MDName = "FPGA_ADVISOR_REPLICATION_FACTOR_";
	MDName += BB->getName().str();
	LLVMContext &C = BB->getContext();
	MDNode *N = MDNode::get(C, MDString::get(C, std::to_string(value)));
	BB->getTerminator()->setMetadata(MDName, N);
}
*/
// Dependence Graph type:
// STL list container for OutEdge list
// STL vector container for vertices
// Use directed edges
// the boolean is an edge property which is true when the dependence edge exists due to a true dependence
// it may also have memory dependences
//typedef boost::property <edge_custom_t, bool> TrueDependence;
struct true_dependence_t {
	typedef boost::edge_property_tag kind;
};

typedef boost::property<true_dependence_t, bool> TrueDependence;
//typedef boost::property<boost::edge_index_t, bool> TrueDependence;
typedef boost::adjacency_list< boost::listS, boost::vecS, boost::bidirectionalS, BasicBlock *, TrueDependence>
		DepGraph;
typedef DepGraph::vertex_iterator DepGraph_iterator;
typedef DepGraph::vertex_descriptor DepGraph_descriptor;
typedef DepGraph::out_edge_iterator DepGraph_out_edge_iterator;
typedef DepGraph::in_edge_iterator DepGraph_in_edge_iterator;
typedef DepGraph::edge_iterator DepGraph_edge_iterator;
typedef DepGraph::edge_descriptor DepGraph_edge_descriptor;

//BOOKMARK change this to module pass...
class DependenceGraph : public FunctionPass {

	public:
		static char ID;
		void getAnalysisUsage(AnalysisUsage &AU) const override {
			AU.addPreserved<AliasAnalysis>();
			AU.setPreservesAll();
			AU.addRequired<DominatorTreeWrapperPass>();
			AU.addRequiredTransitive<AliasAnalysis>();
			//AU.addPreserved<MemoryDependenceAnalysis>();
			AU.addRequiredTransitive<MemoryDependenceAnalysis>();
		}
		DependenceGraph() : FunctionPass(ID) {
			//initializeDependenceGraph(*PassRegistry::getPassRegistry());
			initializeBasicAliasAnalysisPass(*PassRegistry::getPassRegistry());
		}
		bool runOnFunction(Function &F);
		DepGraph &getDepGraph() {
			return DG;
		}
		static DepGraph_descriptor get_vertex_descriptor_for_basic_block(BasicBlock *BB, DepGraph &depGraph);
		static bool is_basic_block_dependent(BasicBlock *BB1, BasicBlock *BB2, DepGraph &DG);
		static void get_all_basic_block_dependencies(DepGraph &depGraph, BasicBlock *BB, std::vector<BasicBlock *> &deps);
		static bool is_basic_block_dependence_true(BasicBlock *BB1, BasicBlock *BB2, DepGraph &DG);
	
	private:
		void add_vertices(Function &F);
		void add_edges();
		void insert_dependent_basic_block(std::vector<std::pair<BasicBlock *, bool> > &list, BasicBlock *BB, bool trueDep);
		void insert_dependent_basic_block_all(std::vector<std::pair<BasicBlock *, bool> > &list, bool trueDep);
		void insert_dependent_basic_block_all_memory(std::vector<std::pair<BasicBlock *, bool> > &list, bool trueDep);
		bool unsupported_memory_instruction(Instruction *I);
		void output_graph_to_file(raw_ostream *outputFile);

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
class BBSchedElem 
{
  private:
		// cycStart and cycEnd are the actual schedules
                int *cycStart; 
                int *cycEnd;

  public:
                void set_min_start(int _start) const { minCycStart = _start;}
                void set_min_end(int _end) const { minCycEnd = _end;}
                void set_start(int _start, int tid) const { cycStart[tid] = _start;}
                void set_end(int _end, int tid) const { cycEnd[tid] = _end;}
		int get_min_start() const { return minCycStart;}
		int get_min_end() const { return minCycEnd;}
		int get_start(int tid) const { return cycStart[tid];}
		int get_end(int tid) const { return cycEnd[tid];}

                BBSchedElem();

		Function *function;
		BasicBlock *basicblock;
		uint64_t ID;
		// the min cycles represent the earliest the basic block may
		// execute, this equates to the scheduling without resource
		// constraint
		int mutable minCycStart;
		int mutable minCycEnd;
		int64_t cpuCycles;
		std::string name;
		// a memory access tuple for each store/load
		// first field stores the starting address, second field stores the width of access in bytes
		std::vector<std::pair<uint64_t, uint64_t> > memoryWriteTuples;
		std::vector<std::pair<uint64_t, uint64_t> > memoryReadTuples;
};

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

// ExecutionOrder map definition
typedef std::map<BasicBlock *, std::pair<int, std::vector<TraceGraph_vertex_descriptor> > > ExecutionOrder;
typedef std::list<ExecutionOrder> ExecutionOrderList;
typedef std::map<Function *, ExecutionOrderList> ExecutionOrderListMap;

// iterators
typedef ExecutionOrder::iterator ExecutionOrder_iterator;
typedef ExecutionOrderList::iterator ExecutionOrderList_iterator;
typedef ExecutionOrderListMap::iterator ExecutionOrderListMap_iterator;

typedef struct {
  int acceleratorLatency;
  int acceleratorII;
  int cpuLatency;
} LatencyStruct;


class FunctionScheduler : public FunctionPass , public InstVisitor<FunctionScheduler> {
	public:
		static char ID;
                static void *analyzerLibHandle;
                static int (*getBlockLatency) (BasicBlock *BB);
                static int (*getBlockII)      (BasicBlock *BB);
                static bool useDefault;

		FunctionScheduler() : FunctionPass(ID) {
                  char * analyzerLib;

                  if ((analyzerLib = getenv("FPGA_ADVISOR_USE_DYNAMIC_ANALYZER"))) {
                    useDefault = false;
                    // Load up the library
                    analyzerLibHandle = dlopen(analyzerLib, RTLD_GLOBAL | RTLD_LAZY);       
                    
                    if (analyzerLibHandle == NULL) {
                      printf("failed to load %s\n", analyzerLib);
                      std::cerr << dlerror() << std::endl;
                      exit(1);
                    }
                    
                    // pull objects out of the library         
                    getBlockLatency = (int (*)(BasicBlock* BB)) dlsym(analyzerLibHandle, "getBlockLatency");

                    if (getBlockLatency == NULL) {
                      printf("failed to load getBlockLatency\n");  
                      exit(1);
                    }                         

                    getBlockII = (int (*)(BasicBlock* BB)) dlsym(analyzerLibHandle, "getBlockII");

                    if (getBlockII == NULL) {
                      printf("failed to load getBlockII\n");  
                      exit(1);
                    }                         
                  }
                }
		void getAnalysisUsage(AnalysisUsage &AU) const override {
			//AU.addPreserved<AliasAnalysis>();
			//AU.addPreserved<MemoryDependenceAnalysis>();
			//AU.addPreserved<DependenceGraph>();
			AU.setPreservesAll();
		}
		bool runOnFunction(Function &F) {
			visit(F);
			return true;
		}

		static int get_basic_block_latency_accelerator(std::map<BasicBlock *, LatencyStruct> &LT, BasicBlock *BB) {
			int latency = 0;
			auto search = LT.find(BB);
			assert(search != LT.end());

			return search->second.acceleratorLatency;
		}


		static int get_basic_block_latency_cpu(std::map<BasicBlock *, LatencyStruct> &LT, BasicBlock *BB) {
			int latency = 0;
			auto search = LT.find(BB);
			assert(search != LT.end());

			return search->second.cpuLatency;
		}

		/*
		int get_basic_block_latency(BasicBlock *BB) {
			auto search = latencyTable.find(BB);
			assert(search != latencyTable.end());
			return search->second;
		}
		*/

		int get_instruction_latency(Instruction *I) {
			int latency = 0;
			switch(I->getOpcode()) {
				// simple binary and logical operations
				case  Instruction::Add :
				case  Instruction::Sub :
				case  Instruction::Shl :
                case  Instruction::LShr:
                case  Instruction::AShr:
                case  Instruction::And :
                case  Instruction::Or  :
                case  Instruction::Xor : latency = 1; break;

				// complicated binary operations
				case  Instruction::Mul :
				case  Instruction::UDiv:
				case  Instruction::SDiv:
				case  Instruction::URem:
				case  Instruction::SRem: latency = 10; break;

				// FP operations
				case  Instruction::FAdd:
				case  Instruction::FSub:
				case  Instruction::FMul:
				case  Instruction::FDiv:
				case  Instruction::FRem: latency = 15; break;

				// memory operations
                case  Instruction::Alloca: latency = 0; break;
                case  Instruction::GetElementPtr: latency = 1; break;
                case  Instruction::Load  :
                case  Instruction::Store :
                case  Instruction::Fence :
                case  Instruction::AtomicCmpXchg:
				case  Instruction::AtomicRMW : latency = 5; break;

				// cast operations
				// these shouldn't take any cycles
                case  Instruction::Trunc   :
                case  Instruction::ZExt    :
                case  Instruction::SExt    :
                case  Instruction::PtrToInt:
                case  Instruction::IntToPtr:
                case  Instruction::BitCast : latency = 0; break;

				// more complicated cast operations
                case  Instruction::FPToUI  :
                case  Instruction::FPToSI  :
                case  Instruction::UIToFP  :
                case  Instruction::SIToFP  :
                case  Instruction::FPTrunc :
                case  Instruction::FPExt   :
                case  Instruction::AddrSpaceCast: latency = 5; break;

				// other
                case  Instruction::ICmp   :
                case  Instruction::FCmp   :
                case  Instruction::PHI    :
                case  Instruction::Select :
                case  Instruction::UserOp1:
                case  Instruction::UserOp2:
                case  Instruction::VAArg  :
                case  Instruction::ExtractElement:
                case  Instruction::InsertElement:
                case  Instruction::ShuffleVector:
                case  Instruction::ExtractValue:
                case  Instruction::InsertValue:
                case  Instruction::LandingPad: latency = 5; break;
				
                case  Instruction::Call   : latency = 100; break; // can be more sophisticated!!!

                case  Instruction::Ret        :
                case  Instruction::Br         :
                case  Instruction::Switch     :
                case  Instruction::Resume     :
                case  Instruction::Unreachable: latency = 0; break;
                case  Instruction::Invoke     : latency = 100; break; // can be more sophisticated!!!
                case  Instruction::IndirectBr : latency = 10; break;

				default: latency = 1;
					std::cerr << "Warning: unknown operation " << I->getOpcodeName() << "\n";
					break;
			}

			return latency;
		}

		std::map<BasicBlock *, LatencyStruct> &getFPGALatencyTable() {
                  return latencyTableFPGA;
		}
	
		void visitBasicBlock(BasicBlock &BB) {
			LatencyStruct latencyStruct;
                        latencyStruct.cpuLatency = 0;    

                        for (auto I = BB.begin(); I != BB.end(); I++) {
                          latencyStruct.cpuLatency += get_instruction_latency(I);
                        }

                        if (useDefault) {
                          // approximate latency of basic block as number of instructions
                          latencyStruct.acceleratorLatency = latencyStruct.cpuLatency;
                          latencyStruct.acceleratorII = latencyStruct.cpuLatency;
                        } else {
                          latencyStruct.acceleratorLatency = getBlockLatency(&BB); 
                          latencyStruct.acceleratorII = getBlockII(&BB); 
                        }                            

			latencyTableFPGA.insert(std::make_pair(BB.getTerminator()->getParent(), latencyStruct));
		}
	
		std::map<BasicBlock *, LatencyStruct> latencyTableFPGA;
	
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
                static void *analyzerLibHandle;
                static int (*getBlockArea)(BasicBlock *BB);
                static bool useDefault;

                FunctionAreaEstimator() : FunctionPass(ID) {
                  char * analyzerLib;

                  if ((analyzerLib = getenv("FPGA_ADVISOR_USE_DYNAMIC_ANALYZER"))) {
                    useDefault = false;
                    // Load up the library
                    analyzerLibHandle = dlopen(analyzerLib, RTLD_GLOBAL | RTLD_LAZY);       
                    
                    if (analyzerLibHandle == NULL) {
                      printf("failed to load %s\n", analyzerLib);
                      std::cerr << dlerror() << std::endl;
                      exit(1);
                    }
                    
                    // pull objects out of the library         
                    getBlockArea = (int (*)(BasicBlock* BB)) dlsym(analyzerLibHandle, "getBlockArea");

                    if (getBlockArea == NULL) {
                      printf("failed to load getBlockArea\n");  
                      exit(1);
                    }                         
                  }
                }        
                
		void getAnalysisUsage(AnalysisUsage &AU) const override {
			AU.addPreserved<AliasAnalysis>();
			AU.addPreserved<MemoryDependenceAnalysis>();
			AU.addPreserved<DependenceGraph>();
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
                        
                        if (useDefault) {
                          // use some fallback estimator code
                          // approximate area of basic block as a weighted sum
                          // the weight is the complexity of the instruction
                          // the sum is over all compute instructions
                          // W = 1 + x1y1 + x2y2 + ... + xnyn
                          // x1 is the complexity of the operation
                          // y1 is the number of this operation existing in the basic block

                          for (auto I = BB.begin(); I != BB.end(); I++) {
                            area += instruction_area_complexity(I);
                          }
                        } else {
                          area = getBlockArea(&BB); 
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
			// basic complexity of instruction is 1
			int complexity = 1;
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

class AdvisorAnalysis;

// Unconstrained scheduler -- does not account for resource limitations in scheduling
class ScheduleVisitor : public boost::default_dfs_visitor {
	public:
		TraceGraphList_iterator graph_ref;
		std::map<BasicBlock *, LatencyStruct> &LT;
		int mutable lastCycle;
		int *lastCycle_ref;
                AdvisorAnalysis *parent;
                int tid;

                ScheduleVisitor(TraceGraphList_iterator graph, AdvisorAnalysis* _parent, std::map<BasicBlock *, LatencyStruct> &_LT, int &lastCycle, int _tid) : graph_ref(graph), parent(_parent), LT(_LT), lastCycle_ref(&lastCycle), tid(_tid) {}

                void discover_vertex(TraceGraph_vertex_descriptor v, const TraceGraph &graph);

}; // end class ScheduleVisitor


class ConstrainedScheduleVisitor : public boost::default_bfs_visitor {
	public:
		TraceGraphList_iterator graph_ref;
		std::map<BasicBlock *, LatencyStruct> &LT;
		int mutable lastCycle;
		int64_t *lastCycle_ref;
		int64_t *cpuCycle_ref;
                std::unordered_map<BasicBlock *, std::vector<unsigned> > *resourceTable; 
                int tid;

  ConstrainedScheduleVisitor(TraceGraphList_iterator graph, std::map<BasicBlock *, LatencyStruct> &_LT, int64_t &lastCycle, int64_t &cpuCycle, std::unordered_map<BasicBlock *,  std::vector<unsigned> > *_resourceTable, int _tid) : graph_ref(graph), LT(_LT), lastCycle_ref(&lastCycle), cpuCycle_ref(&cpuCycle),  resourceTable(_resourceTable), tid(_tid) {}

  void discover_vertex(TraceGraph_vertex_descriptor v, const TraceGraph &graph);

}; // end class ConstrainedScheduleVisitor


class AdvisorAnalysis : public ModulePass, public InstVisitor<AdvisorAnalysis> {
	// node to keep track of where to record trace information
	typedef struct {
		Function *function;
		TraceGraphList_iterator graph;
		TraceGraph_vertex_descriptor vertex;
		ExecutionOrderList_iterator executionOrder;
	} FunctionExecutionRecord;

        typedef struct {
          int      blockCount;
          uint64_t latencyStart;
          uint64_t latencyFinish;
          double   grad;
        } GradientPoint;

        typedef struct {
          std::vector<GradientPoint> gradientPoints;
          double gradientCoefficient;
        } Gradient;

	public:
		static char ID;
                const int THREADS = 16;
                const bool useThreading = true;
		void getAnalysisUsage(AnalysisUsage &AU) const override {
			AU.addPreserved<AliasAnalysis>();
			AU.setPreservesAll();
			AU.addRequired<CallGraphWrapperPass>();
			AU.addRequired<LoopInfo>();
			AU.addRequired<DominatorTreeWrapperPass>();
			AU.addPreserved<DependenceGraph>();
			AU.addRequired<DependenceGraph>();
			AU.addRequired<FunctionScheduler>();
			AU.addRequired<FunctionAreaEstimator>();
		}
                AdvisorAnalysis();
		bool runOnModule(Module &M);
		void visitFunction(Function &F);
		void visitBasicBlock(BasicBlock &BB);
		void visitInstruction(Instruction &I);
                int  get_basic_block_instance_count(BasicBlock *BB);
		void set_basic_block_instance_count(BasicBlock *BB, int value);
                void adjust_all_thread_pool_resource_tables(BasicBlock *BB, int value);
                void set_all_thread_pool_basic_block_instance_counts(BasicBlock *BB, int value); 
                void set_thread_pool_basic_block_instance_count(BasicBlock *BB, int value);
                int  get_thread_pool_basic_block_instance_count(BasicBlock *BB);
                void handle_basic_block_gradient(BasicBlock * BB, std::unordered_map<BasicBlock *, double> * gradient, int initialLatency, int initialArea);

                // Create a TBB queue to give thread ids to the TBB threads.
                boost::lockfree::stack<int> tidPool;  

                // state for each thread. We index these by basic block, such that we might have a thread per basic block. 
                std::unordered_map<BasicBlock*, int> bbInstanceCounts;
                std::unordered_map<BasicBlock *, std::unordered_map<BasicBlock*,int>* > threadPoolInstanceCounts;
                std::unordered_map<BasicBlock *, std::unordered_map<BasicBlock *, std::vector<unsigned> >* > threadPoolResourceTables;

                 
	private:
                // thread pool for gradient. 
                tbb::task_scheduler_init init;
                tbb::task_group group;
                std::mutex threadPoolMutex;

   	        unsigned areaConstraint;                                
                std::vector<double> thresholds;
                
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
		bool process_time(const std::string &line, TraceGraphList_iterator lastTraceGraph, TraceGraph_vertex_descriptor lastVertex, bool start);
		bool process_function_return(const std::string &line, Function **function, std::stack<FunctionExecutionRecord> &stack, TraceGraphList_iterator &lastTraceGraph, TraceGraph_vertex_descriptor &lastVertex, ExecutionOrderList_iterator &lastExecutionOrder);
		bool process_load(const std::string &line, Function *function, TraceGraphList_iterator lastTraceGraph, TraceGraph_vertex_descriptor lastVertex);
		bool process_store(const std::string &line, Function *function, TraceGraphList_iterator lastTraceGraph, TraceGraph_vertex_descriptor lastVertex);
		bool process_basic_block_entry(const std::string &line, int &ID, TraceGraphList_iterator lastTraceGraph, TraceGraph_vertex_descriptor &lastVertex, ExecutionOrderList_iterator lastExecutionOrder);
		bool process_function_entry(const std::string &line, Function **function, TraceGraphList_iterator &latestTraceGraph, TraceGraph_vertex_descriptor &latestVertex, ExecutionOrderList_iterator &latestExecutinoOrder, std::stack<FunctionExecutionRecord> &stack);
		void getCPULatencyTable(Function *F, std::map<BasicBlock *, LatencyStruct> *LT, ExecutionOrderList &executionOrderList, TraceGraphList &executionGraphList);

		bool check_trace_sanity();
		BasicBlock *find_basicblock_by_name(std::string funcName, std::string bbName);
		Function *find_function_by_name(std::string funcName);

		// functions that do analysis on trace
		bool find_maximal_configuration_for_all_calls(Function *F, unsigned &fpgaOnlyLatency, unsigned &fpgaOnlyArea);
		bool find_maximal_configuration_for_call(Function *F, TraceGraphList_iterator graph, ExecutionOrderList_iterator execOrder, std::vector<TraceGraph_vertex_descriptor> &rootVertices);
                bool prune_basic_block_configuration_to_device_area(Function *F);
                int  get_total_basic_block_instances(Function *F);

		//bool find_maximal_configuration_for_call(Function *F, TraceGraphList_iterator graph_it, std::vector<TraceGraph_vertex_descriptor> &rootVertices);
		bool basicblock_is_dependent(BasicBlock *child, BasicBlock *parent, TraceGraph &graph);
		bool instruction_is_dependent(Instruction *inst1, Instruction *inst2);
		bool true_dependence_exists(Instruction *inst1, Instruction *inst2);
		bool basicblock_control_flow_dependent(BasicBlock *child, BasicBlock *parent, TraceGraph &graph);
		void find_new_parents(std::vector<TraceGraph_vertex_descriptor> &newParents, TraceGraph_vertex_descriptor child, TraceGraph_vertex_descriptor parent, TraceGraph &graph);
		bool annotate_schedule_for_call(Function *F, TraceGraphList_iterator graph_it, int &lastCycle);
		bool find_maximal_resource_requirement(Function *F, TraceGraphList_iterator graph_it, std::vector<TraceGraph_vertex_descriptor> &rootVertices, int lastCycle);
		bool latest_parent(TraceGraph_out_edge_iterator edge, TraceGraphList_iterator graph);
		void modify_resource_requirement(Function *F, TraceGraphList_iterator graph_it);
		void find_optimal_configuration_for_all_calls(Function *F, unsigned &cpuOnlyLatency, unsigned fpgaOnlyLatency, unsigned fpgaOnlyArea);
  bool incremental_gradient_descent(Function *F, std::unordered_map<BasicBlock *, double> &gradient, std::unordered_map<BasicBlock*, int> &removeBBs, int64_t &deltaDelay, unsigned cpuOnlyLatency, unsigned fpgaOnlyLatency, unsigned fpgaOnlyArea, int64_t &initialLatency);
		void initialize_basic_block_instance_count(Function *F);
		bool decrement_basic_block_instance_count(BasicBlock *BB);
		bool decrement_thread_pool_basic_block_instance_count(BasicBlock *BB);
		bool increment_basic_block_instance_count(BasicBlock *BB);
		bool increment_thread_pool_basic_block_instance_count(BasicBlock *BB);
		void update_transition(BasicBlock *BB);
		bool decrement_basic_block_instance_count_and_update_transition(BasicBlock *BB);
                bool decrease_basic_block_instance_count_and_update_transition(std::unordered_map<BasicBlock *, int > &removeBBs);
		bool increment_basic_block_instance_count_and_update_transition(BasicBlock *BB);
		void decrement_all_basic_block_instance_count_and_update_transition(Function *F);
		void find_root_vertices(std::vector<TraceGraph_vertex_descriptor> &roots, TraceGraphList_iterator graph_it);
  void dumpImplementationCounts(Function *F);
  void dumpBlockCounts(Function *F, unsigned cpuLatency);
  uint64_t schedule_with_resource_constraints(TraceGraphList_iterator graph_it, Function *F, std::unordered_map<BasicBlock *,  std::vector<unsigned> > *resourceTable, int tid);
  uint64_t schedule_without_resource_constraints(TraceGraphList_iterator graph_it, Function *F, std::unordered_map<BasicBlock *, std::vector<unsigned> > *resourceTable);
  uint64_t schedule_cpu(TraceGraphList_iterator graph_it, Function *F);
		void initialize_resource_table(Function *F, std::unordered_map<BasicBlock *, std::vector<unsigned> > *resourceTable, bool cpuOnly); 
		unsigned get_cpu_only_latency(Function *F);
		unsigned get_area_requirement(Function *F);
		void update_transition_delay(TraceGraphList_iterator graph);
		unsigned get_transition_delay(BasicBlock *source, BasicBlock *target, bool CPUToHW);
		void remove_redundant_dynamic_dependencies(TraceGraphList_iterator graph, std::vector<TraceGraph_vertex_descriptor> &dynamicDeps);
		void recursively_remove_redundant_dynamic_dependencies(TraceGraphList_iterator graph, std::vector<TraceGraph_vertex_descriptor> &dynamicDeps, std::vector<TraceGraph_vertex_descriptor>::iterator search, TraceGraph_vertex_descriptor v);

		bool dynamic_memory_dependence_exists(TraceGraph_vertex_descriptor child, TraceGraph_vertex_descriptor parent, TraceGraphList_iterator graph);
		bool memory_accesses_conflict(std::pair<uint64_t, uint64_t> &access1, std::pair<uint64_t, uint64_t> &access2);

		void print_basic_block_configuration(Function *F, raw_ostream *out);
		void print_optimal_configuration_for_all_calls(Function *F);
		void print_execution_order(ExecutionOrderList_iterator execOrder);

		// dependence graph construction
		bool get_dependence_graph_from_file(std::string fileName, DepGraph **depGraph, std::string funcName);

		// define some data structures for collecting statistics
		std::vector<Function *> functionList;
		std::vector<Function *> recursiveFunctionList;
		//std::vector<std::pair<Loop *, bool> > loopList;

		// recursive and external functions are included
		std::unordered_map<Function *, FunctionInfo *> functionMap;
	
		Module *mod;
		CallGraph *callGraph;

		raw_ostream *outputLog;

		raw_ostream *outputFile;

		// exeuctionTrace contains the execution traces separated by function
		// the value for each key (function) is a vector, where each vector element
		// represents the basicblock execution of one call to that function
		//std::map<Function *, std::list<std::list<BBSchedElem> > > executionTrace;

		ExecGraph executionGraph;

		ExecutionOrderListMap executionOrderListMap;

                std::unordered_map<BasicBlock *, Gradient*> gradients;

		//DepGraph depGraph;

}; // end class AdvisorAnalysis

// put after AdvisorAnalysis class -- uses a function from class
// TraceGraph custom vertex writer for execution trace graph output to dotfile
template <class TraceGraph>
class TraceGraphVertexWriter {
        AdvisorAnalysis *parent;
	public:
                TraceGraphVertexWriter(TraceGraph& _graph, AdvisorAnalysis *_parent) : graph(_graph), parent(_parent) {}
		template <class TraceGraph_vertex_descriptor>
		void operator()(std::ostream& out, const TraceGraph_vertex_descriptor &v) const {
			/*
			out << "[shape = \"record\" label=\"<r0 fontcolor=Red> " << graph[v].cycStart << "| <r1>"
				<< graph[v].name << "| <r2>"
				<< graph[v].cycEnd << "\"]";
			*/
			out << "[shape=\"none\" label=<<table border=\"0\" cellspacing=\"0\">";
			out	<< "<tr><td bgcolor=\"#AEFDFD\" border=\"1\"> " << graph[v].get_min_start() << "</td></tr>";
			if (parent->get_basic_block_instance_count(graph[v].basicblock) > 0) {
			//if (get_basic_block_instance_count_meta(graph[v].basicblock) > 0) {
				out	<< "<tr><td bgcolor=\"#FFFF33\" border=\"1\"> " << graph[v].name << " (" << v << ")" << "</td></tr>";
			} else {
				out	<< "<tr><td bgcolor=\"#FFFFFF\" border=\"1\"> " << graph[v].name << " (" << v << ") " << "</td></tr>";
			}
			out	<< "<tr><td bgcolor=\"#AEFDFD\"  border=\"1\"> " << graph[v].get_min_end() << "</td></tr>";
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
				out << "[color=\"red\" penwidth=\"4\" label=\"" << delay << "\"]";
			}
		}
	private:
		TraceGraph &graph;
}; // end class TraceGraphEdgeWriter



} // end fpga namespace

#endif
