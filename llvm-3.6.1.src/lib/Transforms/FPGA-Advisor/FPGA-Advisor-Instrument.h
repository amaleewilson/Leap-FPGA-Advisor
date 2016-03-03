//===- FPGA-Advisor-Instrument.h - Main FPGA-Advisor pass definition -------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the class declarations for all the analysis and Advisor class
// that are useful for the FPGA-Advisor-Instrument.

#ifndef LLVM_LIB_TRANSFORMS_FPGA_ADVISOR_INSTRUMENT_H
#define LLVM_LIB_TRANSFORMS_FPGA_ADVISOR_INSTRUMENT_H

#include "llvm/Pass.h"
#include "llvm/IR/Function.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/IR/InstVisitor.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/Debug.h"
#include "llvm/Analysis/CallGraph.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/PassManager.h"
#include "llvm/IR/InstVisitor.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/TypeBuilder.h"
#include "llvm/Support/FileSystem.h"

#include <vector>
#include <unordered_map>
#include <map>

using namespace llvm;

namespace {
class AdvisorInstr : public ModulePass {
	public:
		static char ID;
		AdvisorInstr() : ModulePass(ID) {}
		bool runOnModule(Module &M);
	private:
		void instrument_function(Function *F);
		void instrument_basicblock(BasicBlock *BB);
		Module *mod;
		raw_ostream *outputLog;

}; // end class
} // end anonymous namespace

char AdvisorInstr::ID = 0;
static RegisterPass<AdvisorInstr> X("fpga-advisor-instrument", "FPGA-Advisor Instrumentation Pass", false, false);

#endif
