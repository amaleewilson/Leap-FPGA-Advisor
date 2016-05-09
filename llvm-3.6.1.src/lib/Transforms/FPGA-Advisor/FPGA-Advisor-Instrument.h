//===- FPGA-Advisor-Instrument.h - Main FPGA-Advisor pass definition -------*- C++ -*-===//
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
// This file contains the class declarations for all the analysis and Advisor class
// that are useful for the FPGA-Advisor-Instrument.

#ifndef LLVM_LIB_TRANSFORMS_FPGA_ADVISOR_INSTRUMENT_H
#define LLVM_LIB_TRANSFORMS_FPGA_ADVISOR_INSTRUMENT_H

#include "llvm/Pass.h"
#include "llvm/PassManager.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/Analysis/CallGraph.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/InstVisitor.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/TypeBuilder.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/raw_ostream.h"

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
		void instrument_basic_block(BasicBlock *BB);
		void instrument_store(StoreInst *SI);
		void instrument_load(LoadInst *LI);
		void instrument_timer_for_call(Instruction *I);
		void instrument_rdtsc_before_instruction(Instruction *I, bool start);
		void instrument_rdtsc_after_instruction(Instruction *I, bool start);
		void instrument_rdtsc_for_call(Instruction *I);
		uint64_t get_store_size_in_bytes(StoreInst *SI);
		uint64_t get_load_size_in_bytes(LoadInst *LI);
		//std::string get_value_as_string(const Value *value);
		Module *mod;
		raw_ostream *outputLog;

}; // end class
} // end anonymous namespace

char AdvisorInstr::ID = 0;
static RegisterPass<AdvisorInstr> X("fpga-advisor-instrument", "FPGA-Advisor Instrumentation Pass", false, false);

#endif
