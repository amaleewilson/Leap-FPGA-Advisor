//===- Instrument.cpp ---------------------------------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the FPGA-Advisor Instrumentation pass
// This pass is used in the first stage of FPGA-Advisor tool and will 
// instrument the program which allows dynamic run time statistics.
//
//===----------------------------------------------------------------------===//
//
// Author: chenyuti
//
//===----------------------------------------------------------------------===//

#include "Instrument.h"

#define DEBUG_TYPE "instrument"

using namespace llvm;

std::error_code IEC;

bool FInstrument::runOnModule(Module &M) {
	mod = &M;
	raw_fd_ostream OL("fpga-advisor-instrument.log", IEC, sys::fs::F_RW);
	outputLog = &OL;
	DEBUG(outputLog = &dbgs());

	*outputLog << "FPGA-Advisor and Instrumentation Pass Starting.\n";

	for (auto F = M.begin(), FE = M.end(); F != FE; F++) {
		instrument_function(F);
		F->print(*outputLog);
	}

	return true;
}

// Function: instrument_function
// Instruments each function and the basic blocks contained in the function
// such that the insrumented IR will print each function execution as well
// as each basic block that is executed in the function
// e.g.) Entering Function: func
void FInstrument::instrument_function(Function *F) {
	// cannot instrument external functions
	if (F->isDeclaration()) {
		return;
	}

	// add printf for basicblocks first that way the function name printf
	// will be printed before the basicblock due to the way the instructions
	// are inserted (at first insertion point in basic block)
	for (auto BB = F->begin(), BE = F->end(); BB != BE; BB++) {
		instrument_basicblock(BB);
	}

	*outputLog << "Inserting printf call for function: " << F->getName() << "\n";

	// get the entry basic block
	BasicBlock *entry = &(F->getEntryBlock());

	// insert call to printf for entry block
	FunctionType *printf_type = TypeBuilder<int(char *, ...), false>::get(getGlobalContext());
	Function *printfFunc = cast<Function>(mod->getOrInsertFunction("printf", printf_type,
						AttributeSet().addAttribute(mod->getContext(), 1U, Attribute::NoAlias)));
	assert(printfFunc);
	std::vector<Value *> printfArgs;

	IRBuilder<> builder(entry->getFirstInsertionPt());
	StringRef funcMsgString = StringRef("Entering Function: %s\n");
	Value *funcMsg = builder.CreateGlobalStringPtr(funcMsgString, "func_msg_string");
	printfArgs.push_back(funcMsg);
	
	Value *funcNameMsg = builder.CreateGlobalStringPtr(F->getName(), "func_name_string");
	printfArgs.push_back(funcNameMsg);

	//ArrayRef printfArgs(printfArgs);
	builder.CreateCall(printfFunc, printfArgs, llvm::Twine("printf"));
}

// Function: instrument_basicblock
// Instruments each basicblock to print the name of the basicblock when it is encountered
// as well as the function to which it belongs:
// e.g.) BasicBlock: %1 Function: func
// Whenever a return instruction is encountered, the function should print a message
// stating that it is returning from function
// e.g.) Returning from: func
void FInstrument::instrument_basicblock(BasicBlock *BB) {
	*outputLog << "Inserting printf call for basic block: " << BB->getName() << "\n";

	// insert call to printf at first insertion point
	FunctionType *printf_type = TypeBuilder<int(char *, ...), false>::get(getGlobalContext());
	Function *printfFunc = cast<Function>(mod->getOrInsertFunction("printf", printf_type,
						AttributeSet().addAttribute(mod->getContext(), 1U, Attribute::NoAlias)));
	assert(printfFunc);
	std::vector<Value *> printfArgs;

	IRBuilder<> builder(BB->getFirstInsertionPt());

	StringRef bbMsgString = StringRef("BasicBlock: %s Function: %s\n");
	Value *bbMsg = builder.CreateGlobalStringPtr(bbMsgString, "bb_msg_string");
	Value *bbNameMsg = builder.CreateGlobalStringPtr(BB->getName(), "bb_name_string");
	Value *funcNameMsg = builder.CreateGlobalStringPtr(BB->getParent()->getName(), "func_name_string");

	printfArgs.push_back(bbMsg);
	printfArgs.push_back(bbNameMsg);
	printfArgs.push_back(funcNameMsg);
	builder.CreateCall(printfFunc, printfArgs, llvm::Twine("printf"));
	printfArgs.clear();

	// if this basicblock returns from a function, print that message
	if (isa<ReturnInst>(BB->getTerminator())) {
		*outputLog << "Inserting printf call for return: ";
		BB->getTerminator()->print(*outputLog);
		*outputLog << "\n";

		StringRef retMsgString = StringRef("Return from: %s\n");
		Value *retMsg = builder.CreateGlobalStringPtr(retMsgString, "ret_msg_string");

		printfArgs.push_back(retMsg);
		printfArgs.push_back(funcNameMsg);
		builder.CreateCall(printfFunc, printfArgs, llvm::Twine("printf"));
		printfArgs.clear();
	}
}
