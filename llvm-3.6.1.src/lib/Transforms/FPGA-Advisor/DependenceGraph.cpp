//===- DependenceGraph.cpp ---------------------------------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the FPGA-Advisor Dependence Graph constructor
//
//===----------------------------------------------------------------------===//
//
// Author: chenyuti
//
//===----------------------------------------------------------------------===//

#include "fpga_common.h"

#define DEBUG_TYPE "fpga-advisor-dependence"

using namespace llvm;
using namespace fpga;

//===----------------------------------------------------------------------===//
// Globals
//===----------------------------------------------------------------------===//

raw_ostream *outputLog;
std::error_code DEC;

//===----------------------------------------------------------------------===//
// Dependence Graph Pass options
//===----------------------------------------------------------------------===//

static cl::opt<bool> PrintGraph("print-dg", cl::desc("Enable printing of dependence graph in dot format"),
		cl::Hidden, cl::init(false));

//===----------------------------------------------------------------------===//
// Helper functions
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// DependenceGraph Class functions
//===----------------------------------------------------------------------===//

// Function: runOnFunction
bool DependenceGraph::runOnFunction(Function &F) {
	//std::cerr << "runOnFunction: " << F.getName().str() << "\n";
	// create output log
	raw_fd_ostream OL("dependence-graph.log", DEC, sys::fs::F_RW);
	outputLog = &OL;
	DEBUG(outputLog = &dbgs());

	*outputLog << "FPGA-Advisor Dependence Graph Pass for function: " << F.getName() << ".\n";

	if (F.isDeclaration()) return false;

	func = &F;
	DG.clear();
	NameVec.clear();
	MemoryBBs.clear();

	// get analyses
	MDA = &getAnalysis<MemoryDependenceAnalysis>();
	DT = &getAnalysis<DominatorTreeWrapperPass>().getDomTree();

	// add each BB into DG
	add_vertices(F);
	//if (PrintGraph) {
	//	boost::write_graphviz(std::cerr, DG, boost::make_label_writer(&NameVec[0]));
	//}

	// now process each vertex by adding edge to the vertex that
	// the current vertex depends on
	add_edges();
	//boost::write_graphviz(std::cerr, DG);
	if (PrintGraph) {
		std::ofstream outfile("dg.dot");
		boost::write_graphviz(outfile, DG, boost::make_label_writer(&NameVec[0]));
	}
	return true;
}


void DependenceGraph::add_vertices(Function &F) {
	for (auto BB = F.begin(); BB != F.end(); BB++) {
		//bool memoryInst = false;
		for (auto I = BB->begin(); I != BB->end(); I++) {
			if (I->mayReadOrWriteMemory()) {
				//memoryInst = true;
				MemoryBBs.push_back(BB);
				break;
			}
		}
		DepGraph_descriptor currVertex = boost::add_vertex(DG);
		DG[currVertex] = BB;
		NameVec.push_back(BB->getName().str());
		//if (memoryInst) {
			//MemoryBBs.push_back(currVertex);
		//}
	}
}


void DependenceGraph::add_edges() {
	DepGraph_iterator vi, ve;
	for (boost::tie(vi, ve) = vertices(DG); vi != ve; vi++) {
		BasicBlock *currBB = DG[*vi];
		std::vector<BasicBlock *> depBBs;
		*outputLog << "===--------------------------------------------------------------===\n";
		*outputLog << "Examining dependencies for basic block: " << currBB->getName() << "\n";
		// analyze each instruction within the basic block
		// for each operand, find the originating definition
		// for each load/store operator, analyze the memory
		// dependence
		// TODO: I could set the edges to contain a list of
		//	the instructions that caused the dependence
		// Here we only consider true dependences
		for (auto I = currBB->begin(); I != currBB->end(); I++) {
			*outputLog << "***Looking at dependencies for instruction: ";
			I->print(*outputLog);
			*outputLog << "\n";

			// operands
			User *user = dyn_cast<User>(I);
			for (auto op = user->op_begin(); op != user->op_end(); op++) {
				if (Instruction *dep = dyn_cast<Instruction>(op->get())) {
					BasicBlock *depBB = dep->getParent();
					if (depBB == currBB) {
						continue; // don't add self
					}
					*outputLog << "-Dependent on instruction: ";
					dep->print(*outputLog);
					*outputLog << " from basic block: " << depBB->getName() << "\n";
					insert_dependent_basic_block(depBBs, depBB);
				}
			}

			// if store or load
			if (I->mayReadOrWriteMemory()) {
				*outputLog << "This instruction may read/modify memory: ";
				I->print(*outputLog);
				*outputLog << "\n";

				// we cannot analyze function call instructions
				if (unsupported_memory_instruction(I)) {
					// do something
					*outputLog << "Not a supported memory instruction but may read or write memory.\n";
					insert_dependent_basic_block_all_memory(depBBs);
					continue;
				}

				// take a look only at local and non-local dependencies
				// local (within the same basic block) dependencies will matter
				// if control flow ever iterates through the same basic block more
				// than once
				// non-local (within the same function, but different basic blocks)
				// non-func-local (will matter for basic blocks that call functions
				// but for now we can restrict these, or inline the functions
				MemDepResult MDR = MDA->getDependency(I);
				if (MDR.isNonFuncLocal()) {
					*outputLog << "not handling non function local memory dependencies.\n";
				} else if (MDR.isNonLocal()) {
					*outputLog << "Non-local dependence.\n";
					
					SmallVector<NonLocalDepResult, 0> queryResult;
					MDA->getNonLocalPointerDependency(I, queryResult);
					
					for (SmallVectorImpl<NonLocalDepResult>::const_iterator qi = queryResult.begin(); qi != queryResult.end(); qi++) {
						NonLocalDepResult NLDR = *qi;
						const MemDepResult nonLocalMDR = NLDR.getResult();
						Instruction *dep = nonLocalMDR.getInst();
						if (nonLocalMDR.isUnknown() || dep == NULL) {
							*outputLog << "+Unknown/Other type dependence!!!\n";
							insert_dependent_basic_block_all_memory(depBBs);
							break;
						}
						BasicBlock *depBB = dep->getParent();
						insert_dependent_basic_block(depBBs, depBB);

						*outputLog << "+Dependent memory instruction: ";
						dep->print(*outputLog);
						*outputLog << "from basic block: " << depBB->getName() << "\n";
					}
				} else if (MDR.isUnknown()) {
					// we will have to mark every basic block (including self) as dependent
					*outputLog << "+Unknown dependence!!! We have to handle this!!!\n";
					insert_dependent_basic_block_all_memory(depBBs);
				} else {
					*outputLog << "Local dependence.\n";
					Instruction *dep = MDR.getInst();
					// should be same as I->getParent()
					BasicBlock *depBB = dep->getParent();
					*outputLog << "+Dependent memory instruction: ";
					dep->print(*outputLog);
					*outputLog << " from basic block: " << depBB->getName() << "\n";
					insert_dependent_basic_block(depBBs, depBB);
				}
			}
		}

		// add all the dependent edges
		for (auto di = depBBs.begin(); di != depBBs.end(); di++) {
			BasicBlock *depBB = *di;
			DepGraph_descriptor depVertex = get_vertex_descriptor_for_basic_block(depBB, DG);
			DepGraph_descriptor currVertex = get_vertex_descriptor_for_basic_block(currBB, DG);
			boost::add_edge(currVertex, depVertex, DG);
		}
	}
}


DepGraph_descriptor DependenceGraph::get_vertex_descriptor_for_basic_block(BasicBlock *BB, DepGraph &DG) {
//DependenceGraph::DepGraph_descriptor DependenceGraph::get_vertex_descriptor_for_basic_block(BasicBlock *BB) {
	DepGraph_iterator vi, ve;
	for (boost::tie(vi, ve) = vertices(DG); vi != ve; vi++) {
		if (DG[*vi] == BB) {
			return *vi;
		}
	}
	*outputLog << "Error: Could not find basic block in graph.\n";
	assert(0);
}

void DependenceGraph::insert_dependent_basic_block(std::vector<BasicBlock *> &list, BasicBlock *BB) {
	if (std::find(list.begin(), list.end(), BB) == list.end()) {
		list.push_back(BB);
	}
}

void DependenceGraph::insert_dependent_basic_block_all(std::vector<BasicBlock *> &list) {
	for (auto BB = func->begin(); BB != func->end(); BB++) {
		insert_dependent_basic_block(list, BB);
	}
}

// Function insert_dependent_basic_block_all_memory
// adds all basic blocks with memory instructions into dependency list
void DependenceGraph::insert_dependent_basic_block_all_memory(std::vector<BasicBlock *> &list) {
	for (auto BB = MemoryBBs.begin(); BB != MemoryBBs.end(); BB++) {
		insert_dependent_basic_block(list, *BB);
	}
}


bool DependenceGraph::unsupported_memory_instruction(Instruction *I) {
	unsigned opcode = I->getOpcode();
	if (opcode != Instruction::Store && opcode != Instruction::Load &&
		opcode != Instruction::VAArg && opcode != Instruction::AtomicCmpXchg &&
		opcode != Instruction::AtomicRMW) {
		return true;
	}
	return false;
}


// Function: is_basic_block_dependent
// Return: true if BB1 must execute after BB2 due to dependence
// this function only cares about direct dependences, i.e. is there an edge from BB1 to BB2 in DG
bool DependenceGraph::is_basic_block_dependent(BasicBlock *BB1, BasicBlock *BB2, DepGraph &DG) {
	DepGraph_descriptor bb1 = get_vertex_descriptor_for_basic_block(BB1, DG);
	DepGraph_descriptor bb2 = get_vertex_descriptor_for_basic_block(BB2, DG);

	// unfortunately I need to iterate through all the out edges of bb1
	DepGraph_out_edge_iterator oi, oe;
	for (boost::tie(oi, oe) = boost::out_edges(bb1, DG); oi != oe; oi++) {
		if (bb2 == boost::target(*oi, DG)) {
			return true;
		}
	}
	return false;
}



char DependenceGraph::ID = 0;
static RegisterPass<DependenceGraph> X("depgraph", "FPGA-Advisor dependence graph generator", false, false);

