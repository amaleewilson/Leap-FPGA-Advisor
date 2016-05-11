//===- Instrument.cpp ---------------------------------------------------===//
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
// This file implements the FPGA-Advisor Instrumentation pass
// This pass is used in the first stage of FPGA-Advisor tool and will 
// instrument the program which allows dynamic run time statistics.
//
//===----------------------------------------------------------------------===//
//
// Author: chenyuti
//
//===----------------------------------------------------------------------===//

#include "FPGA-Advisor-Instrument.h"

//#define DEBUG_TYPE "fpga-advisor-instrument"
#define DEBUG_TYPE "fpga-advisor"

using namespace llvm;

//===----------------------------------------------------------------------===//
// Globals
//===----------------------------------------------------------------------===//
std::error_code IEC;
StructType *timespecType;
FunctionType *clock_gettime_type;
Function *clock_gettimeFunc;
Function *get_rdtscFunc;
Function *printfFunc;

bool AdvisorInstr::runOnModule(Module &M) {
	mod = &M;
	raw_fd_ostream OL("fpga-advisor-instrument.log", IEC, sys::fs::F_RW);
	outputLog = &OL;
	DEBUG(outputLog = &dbgs());

	*outputLog << "FPGA-Advisor and Instrumentation Pass Starting.\n";

	// define some types and functions that we will need
	std::vector<Type *> paramType;
	paramType.clear();

	std::vector<Type *> timespecTypeAR;
	timespecTypeAR.clear();
	timespecTypeAR.push_back(Type::getInt64Ty(getGlobalContext()));
	timespecTypeAR.push_back(Type::getInt64Ty(getGlobalContext()));

	timespecType = StructType::create(makeArrayRef(timespecTypeAR));

	paramType.push_back(Type::getInt32Ty(getGlobalContext()));
	paramType.push_back(PointerType::get(timespecType, 0)); // pointer to struct

	clock_gettime_type = FunctionType::get(Type::getInt32Ty(getGlobalContext()), 
					makeArrayRef(paramType), false);

	clock_gettimeFunc = cast<Function>(mod->getOrInsertFunction("clock_gettime", clock_gettime_type));

	// get_rdtsc function
	// unsigned long long get_rdtsc(void)
	// create declaration only
	FunctionType *get_rdtsc_type = FunctionType::get(Type::getInt64Ty(getGlobalContext()), false);
	get_rdtscFunc = cast<Function>(mod->getOrInsertFunction("get_rdtsc", get_rdtsc_type));

	// printf function
	// void printf(...)
	FunctionType *printf_type = TypeBuilder<int(char *, ...), false>::get(getGlobalContext());
	printfFunc = cast<Function>(mod->getOrInsertFunction("printf", printf_type, 
					AttributeSet().addAttribute(mod->getContext(), 1U, Attribute::NoAlias)));

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
void AdvisorInstr::instrument_function(Function *F) {
	// cannot instrument external functions
	if (F->isDeclaration()) {
		return;
	}

	// add printf for basicblocks first that way the function name printf
	// will be printed before the basicblock due to the way the instructions
	// are inserted (at first insertion point in basic block)
	for (auto BB = F->begin(), BE = F->end(); BB != BE; BB++) {
		instrument_basic_block(BB);
	}

	*outputLog << "Inserting printf call for function: " << F->getName() << "\n";

	// get the entry basic block
	BasicBlock *entry = &(F->getEntryBlock());

	// insert call to printf for entry block
	std::vector<Value *> printfArgs;

	IRBuilder<> builder(entry->getFirstInsertionPt());
	StringRef funcMsgString = StringRef("\nEntering Function: %s\n");
	Value *funcMsg = builder.CreateGlobalStringPtr(funcMsgString, "func_msg_string");
	printfArgs.push_back(funcMsg);
	
	Value *funcNameMsg = builder.CreateGlobalStringPtr(F->getName(), "func_name_string");
	printfArgs.push_back(funcNameMsg);

	//ArrayRef printfArgs(printfArgs);
	builder.CreateCall(printfFunc, printfArgs, llvm::Twine("printf"));
}

// Function: instrument_basic_block
// Instruments each basicblock to print the name of the basicblock when it is encountered
// as well as the function to which it belongs:
// e.g.) BasicBlock: %1 Function: func
// Whenever a return instruction is encountered, the function should print a message
// stating that it is returning from function
// e.g.) Returning from: func
void AdvisorInstr::instrument_basic_block(BasicBlock *BB) {
	if (isa<TerminatorInst>(BB->getFirstNonPHI())) {
		// skip instrumentation because we don't care about this block
		// no data dependencies, only control
		return;
	}

	*outputLog << "Inserting printf call for basic block: " << BB->getName() << "\n";

	instrument_rdtsc_before_instruction(BB->getFirstNonPHI(), true);
	instrument_rdtsc_before_instruction(BB->getTerminator(), false);

	//===---------------------------------------------------===//
	// [1] stores and loads
	// do these first since code below adds loads that we
	// don't want to profile
	//===---------------------------------------------------===//
	// now insert calls to printf for memory related instructions
	for (auto I = BB->begin(); I != BB->end(); I++) {
		if (isa<StoreInst>(I)) {
			instrument_store(dyn_cast<StoreInst>(I));
		} else if (isa<LoadInst>(I)) {
			instrument_load(dyn_cast<LoadInst>(I));
		}
	}

	// also insert timer start right before call instruction
	// and insert timer stop right after call instruction
	// ignore external function calls
	for (auto I = BB->begin(); I != BB->end(); I++) {
		if (isa<CallInst>(I)) {
			// only add for function calls that are defined
			Function *calledFunc = dyn_cast<CallInst>(I)->getCalledFunction();
			if (!calledFunc) {
				*outputLog << "I haven't dealt with this yet, indirect call. Fix it.\n";
				assert(0);
			}
			if (calledFunc->isDeclaration()) {
				continue;
			}
			instrument_rdtsc_for_call(I);
		}
	}

	//===---------------------------------------------------===//
	// [2] basic block identification
	//===---------------------------------------------------===//
	// insert call to printf at first insertion point
	std::vector<Value *> printfArgs;

	IRBuilder<> builder(BB->getFirstInsertionPt());

	StringRef bbMsgString = StringRef("\nBasicBlock: %s Function: %s\n");
	Value *bbMsg = builder.CreateGlobalStringPtr(bbMsgString, "bb_msg_string");
	Value *bbNameMsg = builder.CreateGlobalStringPtr(BB->getName(), "bb_name_string");
	Value *funcNameMsg = builder.CreateGlobalStringPtr(BB->getParent()->getName(), "func_name_string");

	printfArgs.push_back(bbMsg);
	printfArgs.push_back(bbNameMsg);
	printfArgs.push_back(funcNameMsg);
	builder.CreateCall(printfFunc, printfArgs, llvm::Twine("printf"));
	printfArgs.clear();

	//===---------------------------------------------------===//
	// [3] timer start	
	//===---------------------------------------------------===//
	/*

	// create a struct timespec type
	timespecType->print(*outputLog);

	Value *tp = builder.CreateAlloca(timespecType, NULL, llvm::Twine("timespec"));

	std::vector<Value *> clock_gettimeArgs;
	clock_gettimeArgs.push_back(ConstantInt::get(Type::getInt32Ty(getGlobalContext()), 0)); // null - CLOCK_MONOTONIC
	clock_gettimeArgs.push_back(tp);

	builder.CreateCall(clock_gettimeFunc, clock_gettimeArgs, llvm::Twine("clock_gettime"));

	// create a print statement for the sec and nsec
	StringRef clock_gettimeMsgString = StringRef("\nBasicBlock Clock get time start: %ld s %ld ns\n");
	Value *clock_gettimeMsg = builder.CreateGlobalStringPtr(clock_gettimeMsgString, "clock_gettime_msg_string");

	// need to create a getelementptr instruction for accessing the struct
	std::vector<Value *> tv_secAR;
	tv_secAR.clear();
	tv_secAR.push_back(ConstantInt::get(Type::getInt32Ty(getGlobalContext()), 0));
	tv_secAR.push_back(ConstantInt::get(Type::getInt32Ty(getGlobalContext()), 0));
	Value *tv_secptr = builder.CreateGEP(tp, makeArrayRef(tv_secAR), "tv_sec");
	Value *tv_sec = builder.CreateLoad(tv_secptr, llvm::Twine("load_sec"));

	std::vector<Value *> tv_nsecAR;
	tv_nsecAR.clear();
	tv_nsecAR.push_back(ConstantInt::get(Type::getInt32Ty(getGlobalContext()), 0));
	tv_nsecAR.push_back(ConstantInt::get(Type::getInt32Ty(getGlobalContext()), 1));
	Value *tv_nsecptr = builder.CreateGEP(tp, makeArrayRef(tv_nsecAR), "tv_nsec");
	Value *tv_nsec = builder.CreateLoad(tv_nsecptr, llvm::Twine("load_nsec"));

	printfArgs.push_back(clock_gettimeMsg);
	printfArgs.push_back(tv_sec);
	printfArgs.push_back(tv_nsec);
	builder.CreateCall(printfFunc, printfArgs, llvm::Twine("printf"));
	printfArgs.clear();
	*/

	// set insertion point at end of basic block right before the terminator
	IRBuilder<> endBuilder(BB->getTerminator());

	//===---------------------------------------------------===//
	// [4] timer stops
	//===---------------------------------------------------===//
	/*
	Value *tp2 = endBuilder.CreateAlloca(timespecType, NULL, llvm::Twine("timespec"));

	std::vector<Value *> clock_gettimeArgs2;
	clock_gettimeArgs2.push_back(ConstantInt::get(Type::getInt32Ty(getGlobalContext()), 0)); // null - CLOCK_MONOTONIC
	clock_gettimeArgs2.push_back(tp2);
	endBuilder.CreateCall(clock_gettimeFunc, clock_gettimeArgs2, llvm::Twine("clock_gettime"));

	// create a print statement for the sec and nsec
	StringRef clock_gettimeMsgString2 = StringRef("\nBasicBlock Clock get time stop: %ld s %ld ns\n");
	Value *clock_gettimeMsg2 = endBuilder.CreateGlobalStringPtr(clock_gettimeMsgString2, "clock_gettime_msg_string2");

	// need to create a getelementptr instruction for accessing the struct
	tv_secAR.clear();
	tv_secAR.push_back(ConstantInt::get(Type::getInt32Ty(getGlobalContext()), 0));
	tv_secAR.push_back(ConstantInt::get(Type::getInt32Ty(getGlobalContext()), 0));
	Value *tv_secptr2 = endBuilder.CreateGEP(tp2, makeArrayRef(tv_secAR), "tv_sec");
	Value *tv_sec2 = endBuilder.CreateLoad(tv_secptr2, llvm::Twine("load_sec"));

	tv_nsecAR.clear();
	tv_nsecAR.push_back(ConstantInt::get(Type::getInt32Ty(getGlobalContext()), 0));
	tv_nsecAR.push_back(ConstantInt::get(Type::getInt32Ty(getGlobalContext()), 1));
	Value *tv_nsecptr2 = endBuilder.CreateGEP(tp2, makeArrayRef(tv_nsecAR), "tv_nsec");
	Value *tv_nsec2 = endBuilder.CreateLoad(tv_nsecptr2, llvm::Twine("load_nsec"));

	printfArgs.push_back(clock_gettimeMsg2);
	printfArgs.push_back(tv_sec2);
	printfArgs.push_back(tv_nsec2);
	endBuilder.CreateCall(printfFunc, printfArgs, llvm::Twine("printf"));
	printfArgs.clear();
	*/

	//===---------------------------------------------------===//
	// [5] return
	//===---------------------------------------------------===//
	// if this basicblock returns from a function, print that message
	if (isa<ReturnInst>(BB->getTerminator())) {
		*outputLog << "Inserting printf call for return: ";
		BB->getTerminator()->print(*outputLog);
		*outputLog << "\n";

		StringRef retMsgString = StringRef("\nReturn from: %s\n");
		Value *retMsg = endBuilder.CreateGlobalStringPtr(retMsgString, "ret_msg_string");

		printfArgs.push_back(retMsg);
		printfArgs.push_back(funcNameMsg);
		endBuilder.CreateCall(printfFunc, printfArgs, llvm::Twine("printf"));
		printfArgs.clear();
	}

}


// Function: instrument_rdtsc_before_instruction
// inserts call to get_rdtsc instruction and printf of result before instruction I
void AdvisorInstr::instrument_rdtsc_before_instruction(Instruction *I, bool start) {
	*outputLog << "Inserting get_rdtsc call before instruction ";
	I->print(*outputLog);
	*outputLog << "\n";

	// insert a call to get_rdtsc function - will be linked later
	CallInst *CI = CallInst::Create(get_rdtscFunc, llvm::Twine("get_rdtsc"), I);	

	// insert printf after callinst
	IRBuilder<> builder(CI);
	std::vector<Value *> printfArgs;
	printfArgs.clear();

	StringRef rdtscMsgString;
	if (start) {
		rdtscMsgString = StringRef("\nBasicBlock Clock get time start: %llu\n");
	} else {
		rdtscMsgString = StringRef("\nBasicBlock Clock get time stop: %llu\n");
	}

	Value *rdtscMsg = builder.CreateGlobalStringPtr(rdtscMsgString, "rdtsc_msg_string");

	printfArgs.push_back(rdtscMsg);
	printfArgs.push_back(CI);
	CallInst::Create(printfFunc, printfArgs, llvm::Twine("printf"), I);
}


// Function: instrument_rdtsc_after_instruction
// inserts call to get_rdtsc instruction and printf of result after instruction I
void AdvisorInstr::instrument_rdtsc_after_instruction(Instruction *I, bool start) {
	*outputLog << "Inserting get_rdtsc call after instruction ";
	I->print(*outputLog);
	*outputLog << "\n";

	IRBuilder<> builder(I);

	CallInst *CI = CallInst::Create(get_rdtscFunc, llvm::Twine("get_rdtsc"));
	CI->insertAfter(I);

	std::vector<Value *> printfArgs;
	printfArgs.clear();

	StringRef rdtscMsgString;
	if (start) {
		rdtscMsgString = StringRef("\nBasicBlock Clock get time start: %llu\n");
	} else {
		rdtscMsgString = StringRef("\nBasicBlock Clock get time stop: %llu\n");
	}

	Value *rdtscMsg = builder.CreateGlobalStringPtr(rdtscMsgString, "rdtsc_msg_string");

	printfArgs.push_back(rdtscMsg);
	printfArgs.push_back(CI);
	CallInst *printCI = CallInst::Create(printfFunc, printfArgs, llvm::Twine("printf"));
	printCI->insertAfter(CI);

}


// Function: instrument_rdtsc_for_call
// insert timer stop before call
// inser timer start after call
void AdvisorInstr::instrument_rdtsc_for_call(Instruction *I) {
	*outputLog << "Inserting rdtsc for call instruction: ";
	I->print(*outputLog);
	*outputLog << "\n";

	instrument_rdtsc_before_instruction(I, false);
	instrument_rdtsc_after_instruction(I, true);
}


// Function: instrument_timer_for_call
// insert timer stop before call
// insert timer start after call
void AdvisorInstr::instrument_timer_for_call(Instruction *I) {
	*outputLog << "Inserting printf call for call instruction: ";
	I->print(*outputLog);
	*outputLog << "\n";

	// insertion point after call inst
	IRBuilder<> builder(I);

	//===--------------------------------------------------------------------------------------===//
	// Add stop timer before call
	//===--------------------------------------------------------------------------------------===//
	//Value *tp = builder.CreateAlloca(timespecType, NULL, llvm::Twine("timespec"));
	Value *tp = new AllocaInst(timespecType, NULL, llvm::Twine("timespec"), I);

	std::vector<Value *> clock_gettimeArgs;
	//clock_gettimeArgs.push_back(Constant::getNullValue(Type::getInt32Ty(getGlobalContext()))); // null - CLOCK_REALTIME
	clock_gettimeArgs.push_back(ConstantInt::get(Type::getInt32Ty(getGlobalContext()), 0)); // null - CLOCK_MONOTONIC
	clock_gettimeArgs.push_back(tp);

	//builder.CreateCall(clock_gettimeFunc, clock_gettimeArgs, llvm::Twine("clock_gettime"));
	CallInst::Create(clock_gettimeFunc, clock_gettimeArgs, llvm::Twine("clock_gettime"), I);

	// create a print statement for the sec and nsec
	StringRef clock_gettimeMsgString = StringRef("\nBasicBlock Clock get time stop before call: %ld s %ld ns\n");
	Value *clock_gettimeMsg = builder.CreateGlobalStringPtr(clock_gettimeMsgString, "clock_gettime_msg_string");

	// need to create a getelementptr instruction for accessing the struct
	std::vector<Value *> tv_secAR;
	tv_secAR.clear();
	tv_secAR.push_back(ConstantInt::get(Type::getInt32Ty(getGlobalContext()), 0));
	tv_secAR.push_back(ConstantInt::get(Type::getInt32Ty(getGlobalContext()), 0));
	//Value *tv_secptr = builder.CreateGEP(tp, makeArrayRef(tv_secAR), "tv_sec");
	//Value *tv_sec = builder.CreateLoad(tv_secptr, llvm::Twine("load_sec"));
	GetElementPtrInst *tv_secptr = GetElementPtrInst::Create(tp, makeArrayRef(tv_secAR), llvm::Twine("tv_sec"), I);
	Value *tv_sec = new LoadInst(tv_secptr, llvm::Twine("load_sec"), I);

	std::vector<Value *> tv_nsecAR;
	tv_nsecAR.clear();
	tv_nsecAR.push_back(ConstantInt::get(Type::getInt32Ty(getGlobalContext()), 0));
	tv_nsecAR.push_back(ConstantInt::get(Type::getInt32Ty(getGlobalContext()), 1));
	//Value *tv_nsecptr = builder.CreateGEP(tp, makeArrayRef(tv_nsecAR), "tv_nsec");
	//Value *tv_nsec = builder.CreateLoad(tv_nsecptr, llvm::Twine("load_nsec"));
	GetElementPtrInst *tv_nsecptr = GetElementPtrInst::Create(tp, makeArrayRef(tv_nsecAR), "tv_nsec", I);
	Value *tv_nsec = new LoadInst(tv_nsecptr, llvm::Twine("load_nsec"), I);
	
	std::vector<Value *> printfArgs;

	printfArgs.push_back(clock_gettimeMsg);
	printfArgs.push_back(tv_sec);
	printfArgs.push_back(tv_nsec);
	builder.CreateCall(printfFunc, printfArgs, llvm::Twine("printf"));
	CallInst::Create(printfFunc, printfArgs, llvm::Twine("printf"), I);
	printfArgs.clear();

	//===--------------------------------------------------------------------------------------===//
	// Add start timer after call
	//===--------------------------------------------------------------------------------------===//

}

// Function: instrument_load
void AdvisorInstr::instrument_load(LoadInst *LI) {
	*outputLog << "Inserting printf call for load instruction: ";
	LI->print(*outputLog);
	*outputLog << "\n";

	// get the arguments for address
	Value *pointer = LI->getPointerOperand();
	*outputLog << "the pointer operand ";
	pointer->print(*outputLog);
	*outputLog << "\n";

	// get the argument for read size
	std::string sizeString = std::to_string(get_load_size_in_bytes(LI));
	*outputLog << "the memory access size " << sizeString << "\n";

	// print function
	std::vector<Value *> printfArgs;

	// print right after the load
	IRBuilder<> builder(LI);
	StringRef loadAddrMsgString = StringRef("\nLoad from address: %p size in bytes: " + sizeString + "\n");
	Value *loadAddrMsg = builder.CreateGlobalStringPtr(loadAddrMsgString, "load_addr_msg_string");
	printfArgs.push_back(loadAddrMsg);

	//std::string pointerString = get_value_as_string(pointer);
	//Value *addrMsg = builder.CreateGlobalStringPtr(pointerString, "addr_msg_string");
	//printfArgs.push_back(addrMsg);
	printfArgs.push_back(pointer);

	builder.CreateCall(printfFunc, printfArgs, llvm::Twine("printf"));
}


// Function: instrument_store
// Instruments each store instruction to print the starting address and the number of bytes it accesses
void AdvisorInstr::instrument_store(StoreInst *SI) {
	*outputLog << "Inserting printf call for store instruction: ";
	SI->print(*outputLog);
	*outputLog << "\n";

	// get the arguments for address
	Value *pointer = SI->getPointerOperand();
	*outputLog << "the pointer operand ";
	pointer->print(*outputLog);
	*outputLog << "\n";

	// get the argument for address size
	std::string sizeString = std::to_string(get_store_size_in_bytes(SI));
	*outputLog << "the memory access size " << sizeString << "\n";

	// print function
	std::vector<Value *> printfArgs;

	// print right after the store
	IRBuilder<> builder(SI);
	StringRef storeAddrMsgString = StringRef("\nStore at address: %p size in bytes: " + sizeString + "\n");
	Value *storeAddrMsg = builder.CreateGlobalStringPtr(storeAddrMsgString, "store_addr_msg_string");
	printfArgs.push_back(storeAddrMsg);

	//std::string pointerString = get_value_as_string(pointer);
	//*outputLog << "Store pointer as string: " << pointerString << "\n";
	//Value *addrMsg = builder.CreateGlobalStringPtr(pointerString, "addr_msg_string");
	//printfArgs.push_back(addrMsg);
	printfArgs.push_back(pointer);

	builder.CreateCall(printfFunc, printfArgs, llvm::Twine("printf"));
}


// get the size of the store
uint64_t AdvisorInstr::get_store_size_in_bytes(StoreInst *SI) {
	// ...
	const DataLayout *DL = SI->getParent()->getParent()->getParent()->getDataLayout();
	uint64_t numBytes = DL->getTypeStoreSize(SI->getValueOperand()->getType());

	*outputLog << "Store width in bytes: " << numBytes << "\n";

	return numBytes;
}


// get the size of the load
uint64_t AdvisorInstr::get_load_size_in_bytes(LoadInst *LI) {
	const DataLayout *DL = LI->getParent()->getParent()->getParent()->getDataLayout();
	Type *pointerType = LI->getPointerOperand()->getType();
	// this must be a pointer type... right?
	assert(pointerType->isPointerTy());

	// ... should the argument always be 0?? TODO FIXME
	uint64_t numBytes = DL->getTypeSizeInBits(pointerType->getContainedType(0));
	numBytes >>= 3;

	*outputLog << "Load width in bytes: " << numBytes << "\n";

	return numBytes;
}


// just copied LLVMPrintValueToString from Core.cpp
/*
std::string AdvisorInstr::get_value_as_string(const Value *value) {
	std::string buf;
	raw_string_ostream os(buf);

	if (value)
		value->print(os);
	else
		*outputLog << "Value to print is null\n";
	
	os.flush();

	return buf;
}
*/















