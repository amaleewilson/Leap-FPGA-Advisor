//===- Scheduler.h - Main Scheduler pass definition -------*- C++ -*-===//
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
// that are useful for the Scheduler.

#ifndef LLVM_LIB_TRANSFORMS_SCHEDULER_H
#define LLVM_LIB_TRANSFORMS_SCHEDULER_H

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
#include "llvm/Support/FileSystem.h"

#include <vector>
#include <unordered_map>
#include <map>

using namespace llvm;

namespace {

typedef struct {
	int cycStart;
	int cycEnd;
	Instruction *instruction;
} ScheduleElem;

// The Scheduler class is a module pass which outputs a schedule
// for all the instructions within each basic block in each function
// of the module
class Scheduler : public ModulePass {
	public:
		static char ID;
		void getAnalysisUsage(AnalysisUsage &AU) const override {}
		Scheduler() : ModulePass(ID) {
			//initializeSchedulerPass(*PassRegistry::getPassRegistry());
		}
		bool runOnModule(Module &M);

	private:
		// functions
		void initialize_latency_table();
		void schedule_instructions_in_function(Function *F);
		void schedule_instructions_in_basicblock(BasicBlock *BB);
		bool schedule_instruction(Instruction *I);
		void schedule_terminal_instruction(Instruction *I);
		void fill_schedule();
		int find_num_inst_in_basicblock(BasicBlock *BB);
		bool is_scheduled(Instruction *I);
		int find_operation_latency(Instruction *I);
		int get_end_cycle(Instruction *I);

		// debug
		raw_fd_ostream *scheduleLog;
		raw_ostream *outputLog;
		void print_instruction_schedule(Module &M);
		
		// data structs

		// instSchedule maps each instruction to a ScheduleElem struct
		// which contains the cycle that the instruction should begin
		// execution as well as the cycle that is should finish
		std::map<Instruction *, ScheduleElem *> instSchedule;
		// opLatency stores the operation latency of various operations
		// the first field stores the opcode as an unsigned int and the
		// second field stores the number of cycles latency
		// assume unknown operation latency as 1?? TODO
		std::map<unsigned int, unsigned int> opLatency;
		// schedule stores the instructions to execute in each cycle as
		// a vector of ScheduleElem's, the index of the top level vector
		// represents the clock cycle
		std::vector<std::vector<ScheduleElem> > schedule;

}; // end class Scheduler

char Scheduler::ID = 0;
static RegisterPass<Scheduler> X("module-sched", "Performs simple scheduling of instructions for parallelization potential analysis", false, false);

} // end anonymous namespace

#endif
