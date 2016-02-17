//===- FPGA-Advisor.h - Main FPGA-Advisor pass definition -------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TRANSFORMS_FPGA_ADVISOR_H
#define LLVM_LIB_TRANSFORMS_FPGA_ADVISOR_H

#include "llvm/Pass.h"
#include "llvm/IR/Function.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/IR/InstVisitor.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/Debug.h"
#include "llvm/Analysis/CallGraph.h"
//#include "llvm/PassManager.h"

#include <vector>

#define DEBUG_TYPE "fpga"

namespace llvm {
class Advisor : public ModulePass {
	public:
		static char ID;
		void getAnalysisUsage(AnalysisUsage &AU) const override {
			AU.addRequired<CallGraphWrapperPass>();
		}
		Advisor() : ModulePass(ID) {
			//initializeAdvisorPass(*PassRegistry::getPassRegistry());
		}
		bool runOnModule(Module &M);
	
	private:
		// functions
		void find_recursive_functions(Module &M);
		void does_function_recurse(Function *func, CallGraphNode *CGN, std::vector<Function *> &stack);
		void print_recursive_functions();
		bool run_on_function(Function *F);
		bool has_unsynthesizable_construct(Function *F);
		bool has_recursive_call(Function *F);

		// define some data structures for collecting statistics
		std::vector<Function *> functionList;
		std::vector<Function *> recursiveFunctionList;
		//std::vector<std::pair<Loop *, bool> > loopList;
		
		Module *mod;

}; // end class Advisor

char Advisor::ID = 0;
//INITIALIZE_PASS_BEGIN(Advisor, "fpga-advisor", "FPGA-Advisor Analysis and Instrumentation Pass", false, false)
//INITIALIZE_PASS_DEPENDENCY(CallGraphWrapperPass)
//INITIALIZE_PASS_END(Advisor, "fpga-advisor", "FPGA-Advisor Analysis and Instrumentation Pass", false, false)

static RegisterPass<Advisor> X("fpga-advisor", "FPGA-Advisor Analysis and Instrumentation Pass", false, false);

} // end namespace llvm

#endif
