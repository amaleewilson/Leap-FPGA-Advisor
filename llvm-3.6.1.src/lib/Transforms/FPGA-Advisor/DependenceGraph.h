//===- DependenceGraph.h - Main FPGA-Advisor pass definition -------*- C++ -*-===//
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

#include "llvm/Analysis/DependenceAnalysis.h"
#include "llvm/Analysis/MemoryDependenceAnalysis.h"

#include "common.h"

using namespace llvm;

namespace fpga {
/* moved to common.h
// Graph type:
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
*/

/*
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
	
	private:
		void add_vertices(Function &F);
		void add_edges();
		DepGraph_descriptor get_vertex_descriptor_for_basic_block(BasicBlock *BB);
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
*/

//char DependenceGraph::ID = 0;
//static RegisterPass<DependenceGraph> X("depgraph", "FPGA-Advisor dependence graph generator", false, false);

} // end fpga namespace

#endif
