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
// Graph type:
// STL list container for OutEdge list
// STL vector container for vertices
// Use directed edges
typedef boost::adjacency_list< boost::listS, boost::vecS, boost::bidirectionalS, BasicBlock * > DepGraph;
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
		DependenceGraph() : FunctionPass(ID) {}
		bool runOnFunction(Function &F);
	
	private:
		void add_vertices(Function &F);
		void add_edges();
		DepGraph_descriptor get_vertex_descriptor_for_basic_block(BasicBlock *BB);
		void insert_dependent_basic_block(std::vector<BasicBlock *> &list, BasicBlock *BB);
		void insert_dependent_basic_block_all(std::vector<BasicBlock *> &list);

		Function *func;
		MemoryDependenceAnalysis *MDA;
		DominatorTree *DT;
		DepGraph DG;
}; // end class DependenceGraph
} // end anonymous namespace

char DependenceGraph::ID = 0;
static RegisterPass<DependenceGraph> X("depgraph", "FPGA-Advisor dependence graph generator", false, false);

#endif
