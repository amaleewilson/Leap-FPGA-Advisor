//===- FPGA-Advisor.cpp ----------------------------------- ---------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the FPGA-Advisor Analysis and Instrumentation pass
// This pass is used in the first stage of FPGA-Advisor tool and will provide
// both static compile time statistics as well as instrument the program
// which allows dynamic run time statistics. The list of statistics this pass
// will gather is listed below:
//
// Static statistics:
//	- # functions
//	- # basic blocks in each function
//	- # loops in each function
//	- # parallelizable loops in each function
//	- loop size (determined by)
//		- # instructions within loop
//		- # operations within loop
//
// Dynamic statistics:
// 	- # of times each basic block is run
//
// Beyond these statistics, the pass will also notify the user when a program
// is not expected to perform well on an FPGA as well as when it contains
// constructs which cannot be implemented on the FPGA.
//
//===----------------------------------------------------------------------===//
//
// Author: chenyuti
//
//===----------------------------------------------------------------------===//

#include "FPGA-Advisor.h"

using namespace llvm;

// List of statistics -- not necessarily the statistics listed above, this is
// at a module level
STATISTIC(FunctionCounter, "Counts number of functions in module");
STATISTIC(BasicBlockCounter, "Total number of basic blocks in all functions in module");
STATISTIC(LoopCounter, "Total number of loops in all functions in module");
STATISTIC(ParallelizableLoopCounter, "Total number of parallelizable loops in all functions in module");
STATISTIC(InstructionCounter, "Total number of instructions in all functions in module");
STATISTIC(LoopInstructionCounter, "Total number of instructions in all loops in all functions in module");
STATISTIC(ParallelizableLoopInstructionCounter, "Total number of instructions in all parallelizable loops in all functions in module");

bool Advisor::runOnModule(Module &M) {
	DEBUG(dbgs() << "FPGA-Advisor Analysis and Instrumentation Pass starting.\n");

	mod = &M;

	find_recursive_functions(M);

	// For each function
	for (auto F = M.begin(), FE = M.end(); F != FE; F++) {
		run_on_function(F);
	}

	return true;
}

// Function: find_recursive_functions
// Return: nothing
void Advisor::find_recursive_functions(Module &M) {
	DEBUG(dbgs() << __func__ << "\n");
	// look at call graph for loops
	CallGraph &CG = getAnalysis<CallGraphWrapperPass>().getCallGraph();
	DEBUG(CG.print(dbgs()); dbgs() << "\n");
	//CallGraph &CG = CallGraphAnalysis::run(&M);

	// do a depth first search to find the recursive functions
	// a function is recursive if any of its called functions is
	// either itself or contains a call to itself
	// (ironically), use recursion for this...
	// store onto the recursiveFunctionList
	for (auto F = M.begin(), FE = M.end(); F != FE; F++) {
		if (!F->isDeclaration()) {
			DEBUG(dbgs() << "Calling does_function_recurse on function: " << F->getName() << "\n");
			std::vector<Function *> fStack;
			does_function_recurse(F, CG.getOrInsertFunction(F), fStack); // function will modify recursiveFunctionList directly
			assert(fStack.empty());
		} else {
			errs() << __func__ << " ignored.\n";
		}
	}
	DEBUG(print_recursive_functions());
}

// Function: does_function_recurse
// Return: nothing
// Modifies recursiveFunctionList vector
void Advisor::does_function_recurse(Function *func, CallGraphNode *CGN, std::vector<Function *> &stack) {
	DEBUG(dbgs() << "does_function_recurse: " << CGN->getFunction()->getName() << "\n");
	DEBUG(dbgs() << "stack size: " << stack.size() << "\n");
	// if this function exists within the stack, function recurses and add to list
	if ((stack.size() > 0) && (std::find(stack.begin(), stack.end(), CGN->getFunction()) != stack.end())) {
		DEBUG(dbgs() << "Function recurses: " << CGN->getFunction()->getName() << "\n");
		
		// delete functions off "stack"
		//while (stack[stack.size()-1] != CGN->getFunction()) {
			// pop off stack
		//	stack.pop_back();
		//	DEBUG(dbgs() << "stack size: " << stack.size() << "\n");
		//}
		//stack.pop_back();
		//DEBUG(dbgs() << "stack size: " << stack.size() << "\n");
		// add to recursiveFunctionList only if this is the function we are checking to be recursive or not
		// this avoids the situation where a recursive function is added to the list multiple times
		if (CGN->getFunction() == func) {
			recursiveFunctionList.push_back(CGN->getFunction());
		}
		return;
	}

	// else, add the function to the stack and call does_function_recurse on each of the functions
	// contained by this CGN
	stack.push_back(CGN->getFunction());
	for (auto it = CGN->begin(), et = CGN->end(); it != et; it++) {
		CallGraphNode *calledGraphNode = it->second;
		DEBUG(dbgs() << "Found a call to function: " << calledGraphNode->getFunction()->getName() << "\n");
		//stack.push_back(calledGraphNode->getFunction());
		// ignore this function if its primary definition is outside current module
		if (! calledGraphNode->getFunction()->isDeclaration()) {
			does_function_recurse(func, calledGraphNode, stack);
		} else { // print a warning
			errs() << __func__ << " is being ignored, it is declared outside of this translational unit.\n";
		}
		DEBUG(dbgs() << "Returned from call to function: " << calledGraphNode->getFunction()->getName() << "\n");
	}
	// pop off the stack
	stack.pop_back();
	DEBUG(dbgs() << "stack size: " << stack.size() << "\n");
	return;
}


void Advisor::print_recursive_functions() {
	dbgs() << "Found recursive functions: \n";
	for (auto r = recursiveFunctionList.begin(), re = recursiveFunctionList.end(); r != re; r++) {
		Function *F = *r;
		dbgs() << "  " << F->getName() << "\n";
	}
}

// Function: run_on_function
// Return: fals if function cannot be synthesized
bool Advisor::run_on_function(Function *F) {
	DEBUG(dbgs() << "Examine function: " << F->getName() << "\n");
	// Find constructs that are not supported by HLS
	if (has_unsynthesizable_construct(F)) {
		DEBUG(dbgs() << "Function contains unsynthesizable constructs, moving on.\n");
		return false;
	}

	return true;
}

// Function: has_unsynthesizable_construct
// Return: true if module contains code which is not able to be run through HLS tools
// These contain:
// 	- Recursion
// 	- Dynamic memory allocation
//	- Arbitrary pointer accesses
// 	- Some tools do not support pthread/openmp but LegUp does (so we will ignore it)
bool Advisor::has_unsynthesizable_construct(Function *F) {
	// no recursion
	if (has_recursive_call(F)) {
		DEBUG(dbgs() << "Function has recursive call.\n");
		return true;
	}

	// no external function calls

	// examine memory accesses

	return false;
}

// Function: has_recursive_call
// Return: true if function could possibly recurse on itself
// A function recurses if it or any of the functions it calls calls itself
bool Advisor::has_recursive_call(Function *F) {
	return false;
}
