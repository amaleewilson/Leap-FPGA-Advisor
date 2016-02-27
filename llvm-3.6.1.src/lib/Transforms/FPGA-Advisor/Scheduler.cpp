//===- FScheduler.cpp -----------------------------------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the FScheduler pass
// This pass is used to build a schedule for the function
//
//===----------------------------------------------------------------------===//
//
// Author: chenyuti
//
//===----------------------------------------------------------------------===//

#include "Scheduler.h"
#include <algorithm>

#define DEBUG_TYPE "module-sched"

using namespace llvm;

std::error_code EC;

bool Scheduler::runOnModule(Module &M) {
	// output log
	raw_fd_ostream SL("schedule.log", EC, sys::fs::F_RW);
	scheduleLog = &SL;
	*scheduleLog << "Scheduling log:\n";

	raw_fd_ostream OL("output.log", EC, sys::fs::F_RW);
	outputLog = &OL;
	// if debugging flag turned on, debugging output will be displayed,
	// else it will still be kept in output.log
	DEBUG(outputLog = &dbgs());
	*outputLog << "Output log:\n";
	
	//DEBUG(*outputLog << "Function begin: " << __func__ << "\n");

	// initialize opLatency table
	initialize_latency_table();

	for (auto F = M.begin(), FE = M.end(); F != FE; F++) {
		// schedule each function
		schedule_instructions_in_function(F);

		// make the actual schedule list
		fill_schedule();
	}

	print_instruction_schedule(M);

	return true;
}


void Scheduler::initialize_latency_table() {
	// leave an empty table for now -- TODO
}


// Function: schedule_instructions_in_function
// This function should fill in the instSchedule table
void Scheduler::schedule_instructions_in_function(Function *F) {
	// The scheduler iterates over all basic blocks, creating a
	// schedule for each basic block
	for (auto BB = F->begin(), BE = F->end(); BB != BE; BB++) {
		schedule_instructions_in_basicblock(BB);
	}
}


// Function: schedule_instructions_in_basicblock
// This function should fill in the instSchedule table for a basic block
void Scheduler::schedule_instructions_in_basicblock(BasicBlock *BB) {
	int numInstScheduled = 0;
	int numInstInBB = find_num_inst_in_basicblock(BB);
	while (numInstScheduled < numInstInBB - 1) {
		for (auto I = BB->begin(), IE = BB->end(); I != IE; I++) {
			if (isa<TerminatorInst>(I)) {
				continue;
			}
			if (schedule_instruction(I)) {
				numInstScheduled++;
			}
		}
	}
	schedule_terminal_instruction(BB->getTerminator());
}


// Function: schedule_instruction
// This function should fill in the instSchedule table for an instruction
// when all its dependencies have been scheduled
// Note: this function does not schedule the terminal instruction, that's
// handled separately by schedule_terminal_instruction
bool Scheduler::schedule_instruction(Instruction *I) {
	assert(!dyn_cast<TerminatorInst>(I));

	if (is_scheduled(I)) {
		return false;
	}

	// How to schedule instructions:
	//	1.	Non-branch (control flow) instructions which have no dependencies or whose
	//		operands are all value constants AND instructions which are only dependent
	//		on input arguments can be scheduled immediately
	//	2.	Non-branch (control flow) instructions which have reaching definitions
	//		from outside of this basic block can be scheduled immediately
	//	3.	Then start to schedule any non-branch instruction for which all of its
	//		operands are ready (be careful to consider the constants
	//	4.	Branch instructions (terminating instructions) can only be scheduled when
	//		all other instructions in the basic block has been scheduled to avoide a
	//		situation where an unconditional branch might be scheduled immediately
	*outputLog << "attempt to schedule: "; I->print(*outputLog); *outputLog << "\n";

	int cycleStart = 0; // relative to this basic block
	
	// get all the operands
	User *user = dyn_cast<User>(I);
	assert(user);

	*outputLog << "uses:\n";
	for (auto op = user->op_begin(); op != user->op_end(); op++) {
		Value *val = op->get();
		*outputLog << ">>> "; val->print(*outputLog); *outputLog << "\n";
		if (Argument *Arg = dyn_cast<Argument>(val)) {
			// operand is an argument
			if (Arg->getParent() != I->getParent()->getParent()) {
				errs() << "Instruction uses argument not belonging to the same function??\n";
				assert(0);
			}
			*outputLog << "DEPENDENT ON ARGUMENT\n\n";
			continue;
		} else if (User *opUser = dyn_cast<User>(val)) {
			// operand is a user
			if (Constant *opC = dyn_cast<Constant>(opUser)) {
				*outputLog << "Constant: "; opC->print(*outputLog); *outputLog << "\n";
				// constants are immutable at runtime -- what do they translate to
				// in assembly?
				// TODO: does this mean instructions depending on constants --
				// including constant expressions are able to run right away?
				continue;
			} else if (Instruction *opI = dyn_cast<Instruction>(opUser)) {
				*outputLog << "Instruction: "; opI->print(*outputLog); *outputLog << "\n";
				// if operand is instruction from outside this basic block then
				// it does not put any constraints on our schedule
				if (opI->getParent() != I->getParent()) {
					*outputLog << "not in same bb\n";
					continue;
				} else if (!is_scheduled(opI)) {
					*outputLog << "not been scheduled\n";
					return false;
				} else {
					cycleStart = std::max(cycleStart, get_end_cycle(opI) + 1);
					continue;
				}
			} else {
				// Operator
				*outputLog << "Operator\n";
				assert(0); // how to handle this???
			}
		} else {
			// operand is either a BasicBlock, inlineAsm, or MetatdataAsValue??
			errs() << "Operand is not of expected type.\n";
			assert(0);
		}
	}

	*outputLog << "Scheduled for cycle: " << cycleStart << "\n";
	// if we got here, all the dependencies are ready, we can create the scheduling element
	ScheduleElem *newElem = new ScheduleElem();
	newElem->instruction = I;
	newElem->cycStart = cycleStart;
	// subtract by one is due to the way cycEnd is defined. It represents the last cycle of
	// execution of this operation. E.g. if an add operation takes 1 cycle to operate and
	// begins on cycle 3, then the cycEnd is also 3 and an instruction that depends on its
	// result can begin in cycle 4.
	newElem->cycEnd = cycleStart + find_operation_latency(I) - 1;
	
	instSchedule.insert(std::pair<Instruction *, ScheduleElem *>(I, newElem));

	*scheduleLog << "scheduled instruction: ";
	I->print(*scheduleLog);
	*scheduleLog << " starting cycle: " << cycleStart << " last cycle: " << newElem->cycEnd << "\n";
	
	return true;
}


// Function: schedule_terminal_instruction
void Scheduler::schedule_terminal_instruction(Instruction *I) {
	assert(dyn_cast<TerminatorInst>(I));
	*outputLog << "attempt to schedule: "; I->print(*outputLog); *outputLog << "\n";
	// do the scheduling, the terminal instruction can at the earliest
	// execute after the latest instruction to start execution has
	// started to execute. We do not require for that instruction to
	// have finished execution, however, if an instruction in subsequent
	// basic blocks require the result of that instruction, then they will
	// have to wait. This, however, is computed at run time.
	int cycleStart = 0;
	for (auto it = instSchedule.begin(), et = instSchedule.end(); it != et; it++) {
		if (it->first->getParent() == I->getParent()) {
			cycleStart = std::max(cycleStart, it->second->cycStart);
		}
	}
	
	*outputLog << "Scheduled for cycle: " << cycleStart << "\n";

	// create the terminal instruction scheduling element
	ScheduleElem *newElem = new ScheduleElem();
	newElem->instruction = I;
	newElem->cycStart = cycleStart;
	newElem->cycEnd = cycleStart + find_operation_latency(I) - 1;

	instSchedule.insert(std::pair<Instruction *, ScheduleElem *>(I, newElem));

	*scheduleLog << "scheduled terminal instruction: ";
	I->print(*scheduleLog);
	*scheduleLog << " starting cycle: " << cycleStart << " last cycle: " << newElem->cycEnd << "\n";
}


// Function: fill_schedule
// This function should fill in the schedule vector
void Scheduler::fill_schedule() {

}


int Scheduler::find_num_inst_in_basicblock(BasicBlock *BB) {
	int count = 0;
	for (auto I = BB->begin(), IE = BB->end(); I != IE; I++) {
		count++;
	}
	return count;
}


// Function: is_scheduled
// Return: true if the instruction has an existing entry in instSchedule
//			i.e. it has been scheduled
bool Scheduler::is_scheduled(Instruction *I) {
	return (instSchedule.find(I) != instSchedule.end());
}


// Function: get_end_cycle
// Return: the last schedule of execution of the instruction
// if the instruction takes 1 cycle to complete then the 
// start and end cycles are equivalent
int Scheduler::get_end_cycle(Instruction *I) {
	ScheduleElem *elem = instSchedule.find(I)->second;
	return elem->cycEnd;
}


// Function: find_operation_latency
// Return: the latency of the operation
int Scheduler::find_operation_latency(Instruction *I) {
	// do a search on the opLatency table to get the latency of operations
	// if the operation does not exist in the table, assume it is 1 cycle??
	if (opLatency.find(I->getOpcode()) == opLatency.end()) {
		*outputLog << "Could not find the latency of operation, default 1. "; I->print(*outputLog); *outputLog << "\n";
		return 1;
	} else {
		return (opLatency.find(I->getOpcode()))->second;
	}
}


void Scheduler::print_instruction_schedule(Module &M) {
	for (auto F = M.begin(), FE = M.end(); F != FE; F++) {
		*outputLog << "Function: " << F->getName() << "\n";
		for (auto BB = F->begin(), BE = F->end(); BB != BE; BB++) {
			*outputLog << "BasicBlock: " << BB->getName() << "\n";
			for (auto I = BB->begin(), IE = BB->end(); I != IE; I++) {
				auto entry = instSchedule.find(I);
				assert(entry != instSchedule.end());
				ScheduleElem *elem = entry->second;
				I->print(*outputLog);
				*outputLog << "\nStart: " << elem->cycStart << "\tEnd: " << elem->cycEnd << "\n";
			}
		}

	}
}
