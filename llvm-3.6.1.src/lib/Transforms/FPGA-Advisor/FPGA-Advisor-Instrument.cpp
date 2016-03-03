//===- FPGA-Advisor-Instrument.cpp ---------------------------------------------------===//
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

#include "FPGA-Advisor-Instrument.h"

#define DEBUG_TYPE "fpga-advisor-instrument"

using namespace llvm;

// List of statistics -- not necessarily the statistics listed above, this is
// at a module level
STATISTIC(FunctionCounter, "Number of functions in module");
STATISTIC(BasicBlockCounter, "Number of basic blocks in all functions in module");
STATISTIC(LoopCounter, "Number of loops in all functions in module");
STATISTIC(ParallelizableLoopCounter, "Number of parallelizable loops in all functions in module");
STATISTIC(InstructionCounter, "Number of instructions in all functions in module");
STATISTIC(LoopInstructionCounter, "Number of instructions in all loops in all functions in module");
STATISTIC(ParallelizableLoopInstructionCounter, "Number of instructions in all parallelizable loops in all functions in module");

std::error_code FEC;

// Function: runOnModule
// The function will first find the recursive functions within the module
// then visit each function, basic block, instruction etc. to gather statistics
// then for each function perform an analysis using the previously gathered
// statistics in run_on_function
// print these statistics
// finally it will perform instrumentation on the code --> this may be impl.
// as a separate pass
bool AdvisorInst::runOnModule(Module &M) {

	//PassManager PM;
	//PM.add(new LoopInfo());
	//PM.run(M);

	raw_fd_ostream OL("fpga-advisor-instrument.log", FEC, sys::fs::F_RW);
	outputLog = &OL;
	DEBUG(outputLog = &dbgs());

	*outputLog << "FPGA-Advisor and Instrumentation Pass Starting.\n";

	mod = &M;

	// get all the analyses
	callGraph = &getAnalysis<CallGraphWrapperPass>().getCallGraph();


	find_recursive_functions(M);

	// basic statistics gathering
	// also populates the functionMap
	visit(M);

	// For each function
	for (auto F = M.begin(), FE = M.end(); F != FE; F++) {
		run_on_function(F);
	}

	*outputLog << "On to printing statistics\n";
	// pre-instrumentation statistics
	print_statistics();

/*
	*outputLog << "On to instrumentation\n";
	// instrumentation stage
	for (auto F = M.begin(), FE = M.end(); F != FE; F++) {
		instrument_function(F);
		F->print(*outputLog);
	}
*/

	return true;
}

void AdvisorInst::visitFunction(Function &F) {
	*outputLog << "visit Function: " << F.getName() << "\n";
	FunctionCounter++;

	// create and initialize a node for this function
	FunctionInfo *newFuncInfo = new FunctionInfo();
	newFuncInfo->function = &F;
	newFuncInfo->bbList.clear();
	newFuncInfo->instList.clear();
	newFuncInfo->loopList.clear();
	
	/* Loop stuff -- ignore for now
	if (! F.isDeclaration()) {
		// only get the loop info for functions with a body, else will get assertion error
		newFuncInfo->loopInfo = &getAnalysis<LoopInfo>(F);
		dbgs() << "PRINTOUT THE LOOPINFO\n";
		newFuncInfo->loopInfo->print(dbgs());
		dbgs() << "\n";
		// find all the loops in this function
		for (LoopInfo::reverse_iterator li = newFuncInfo->loopInfo->rbegin(), le = newFuncInfo->loopInfo->rend(); li != le; li++) {
			*outputLog << "Encountered a loop!\n";
			(*li)->print(dbgs());
			dbgs() << "\n" << (*li)->isAnnotatedParallel() << "\n";
			// append to the loopList
			newFuncInfo->loopList.push_back(*li);
		}
	}
	*/

	// insert into the map
	functionMap.insert( {&F, newFuncInfo} );
}

void AdvisorInst::visitBasicBlock(BasicBlock &BB) {
	//*outputLog << "visit BasicBlock: " << BB.getName() << "\n";
	BasicBlockCounter++;

	// make sure function is in functionMap
	assert(functionMap.find(BB.getParent()) != functionMap.end());
	FunctionInfo *FI = functionMap.find(BB.getParent())->second;
	FI->bbList.push_back(&BB);
}

void AdvisorInst::visitInstruction(Instruction &I) {
	//*outputLog << "visit Instruction: " << I << "\n";
	InstructionCounter++;

	// make sure function is in functionMap
	assert(functionMap.find(I.getParent()->getParent()) != functionMap.end());
	FunctionInfo *FI = functionMap.find(I.getParent()->getParent())->second;
	FI->instList.push_back(&I);
	
	// eliminate instructions which are not synthesizable
}

// visit callsites -- count function calls
// visit memory related instructions

// Function: print_statistics
// Return: nothing
void AdvisorInst::print_statistics() {
	errs() << "Number of Functions : " << functionMap.size() << "\n";
	// iterate through each function info block
	for (auto it = functionMap.begin(), et = functionMap.end(); it != et; it++) {
		errs() << it->first->getName() << ":\n";
		errs() << "\t" << "Number of BasicBlocks : " << it->second->bbList.size() << "\n";
		errs() << "\t" << "Number of Instructions : " << it->second->instList.size() << "\n";
		errs() << "\t" << "Number of Loops : " << it->second->loopList.size() << "\n";
	}
}


// Function: find_recursive_functions
// Return: nothing
void AdvisorInst::find_recursive_functions(Module &M) {
	*outputLog << __func__ << "\n";
	// look at call graph for loops
	//CallGraph &CG = getAnalysis<CallGraphWrapperPass>().getCallGraph();
	//DEBUG(CG.print(dbgs()); dbgs() << "\n");
	DEBUG(callGraph->print(dbgs()); dbgs() << "\n");
	//CallGraph &CG = CallGraphAnalysis::run(&M);

	// do a depth first search to find the recursive functions
	// a function is recursive if any of its called functions is
	// either itself or contains a call to itself
	// (ironically), use recursion for this...
	// store onto the recursiveFunctionList
	for (auto F = M.begin(), FE = M.end(); F != FE; F++) {
		if (!F->isDeclaration()) {
			*outputLog << "Calling does_function_recurse on function: " << F->getName() << "\n";
			std::vector<Function *> fStack;
			// function will modify recursiveFunctionList directly
			does_function_recurse(F, callGraph->getOrInsertFunction(F), fStack); 
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
void AdvisorInst::does_function_recurse(Function *func, CallGraphNode *CGN, std::vector<Function *> &stack) {
	*outputLog << "does_function_recurse: " << CGN->getFunction()->getName() << "\n";
	*outputLog << "stack size: " << stack.size() << "\n";
	// if this function exists within the stack, function recurses and add to list
	if ((stack.size() > 0) && (std::find(stack.begin(), stack.end(), CGN->getFunction()) != stack.end())) {
		*outputLog << "Function recurses: " << CGN->getFunction()->getName() << "\n";
		
		// delete functions off "stack"
		//while (stack[stack.size()-1] != CGN->getFunction()) {
			// pop off stack
		//	stack.pop_back();
		//	*outputLog << "stack size: " << stack.size() << "\n";
		//}
		//stack.pop_back();
		//*outputLog << "stack size: " << stack.size() << "\n";
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
		*outputLog << "Found a call to function: " << calledGraphNode->getFunction()->getName() << "\n";
		//stack.push_back(calledGraphNode->getFunction());
		// ignore this function if its primary definition is outside current module
		if (! calledGraphNode->getFunction()->isDeclaration()) {
			does_function_recurse(func, calledGraphNode, stack);
		} else { // print a warning
			errs() << __func__ << " is being ignored, it is declared outside of this translational unit.\n";
		}
		*outputLog << "Returned from call to function: " 
					<< calledGraphNode->getFunction()->getName() << "\n";
	}
	// pop off the stack
	stack.pop_back();
	*outputLog << "stack size: " << stack.size() << "\n";
	return;
}


void AdvisorInst::print_recursive_functions() {
	dbgs() << "Found recursive functions: \n";
	for (auto r = recursiveFunctionList.begin(), re = recursiveFunctionList.end(); r != re; r++) {
		Function *F = *r;
		dbgs() << "  " << F->getName() << "\n";
	}
}

// Function: run_on_function
// Return: false if function cannot be synthesized
// Function looks at the loops within the function
bool AdvisorInst::run_on_function(Function *F) {
	*outputLog << "Examine function: " << F->getName() << "\n";
	// Find constructs that are not supported by HLS
	if (has_unsynthesizable_construct(F)) {
		*outputLog << "Function contains unsynthesizable constructs, moving on.\n";
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
bool AdvisorInst::has_unsynthesizable_construct(Function *F) {
	// no recursion
	if (has_recursive_call(F)) {
		*outputLog << "Function has recursive call.\n";
		return true;
	}

	// no external function calls
	if (has_external_call(F)) {
		*outputLog << "Function has external function call.\n";
		return true;
	}

	// examine memory accesses

	return false;
}

// Function: is_recursive_function
// Return: true if function is contained in recursiveFunctionList
// A function recurses if it or any of the functions it calls calls itself
// TODO?? I do not handle function pointers by the way
bool AdvisorInst::is_recursive_function(Function *F) {
	return (std::find(recursiveFunctionList.begin(), recursiveFunctionList.end(), F) 
			!= recursiveFunctionList.end());
}

// Function: has_recursive_call
// Return: true if function is recursive or contains a call to a recursive 
// function on the recursiveFunctionList
bool AdvisorInst::has_recursive_call(Function *F) {
	if (is_recursive_function(F)) {
		return true;
	}

	bool result = false;

	// look through the CallGraph for this function to see if this function makes calls to 
	// recursive functions either directly or indirectly
	if (! F->isDeclaration()) {
		result = does_function_call_recursive_function(callGraph->getOrInsertFunction(F));
	} else {
		//errs() << __func__ << " is being ignored, it is declared outside of this translational unit.\n";
	}

	return result;
}

// Function: does_function_call_recursive_function
// Return: true if function contain a call to a function which is recursive
// This function should not recurse infinitely since it will stop at a recursive function
// and therefore not get stuck in a loop in the call graph
bool AdvisorInst::does_function_call_recursive_function(CallGraphNode *CGN) {
	if (is_recursive_function(CGN->getFunction())) {
		return true;
	}

	bool result = false;

	for (auto it = CGN->begin(), et = CGN->end(); it != et; it++) {
		CallGraphNode *calledGraphNode = it->second;
		*outputLog << "Found a call to function: " << calledGraphNode->getFunction()->getName() << "\n";
		if (! calledGraphNode->getFunction()->isDeclaration()) {
			result |= does_function_call_recursive_function(calledGraphNode);
		} else {
			//errs() << __func__ << " is being ignored, it is declared outside of this translational unit.\n";
		}
	}
	return result;
}

// Function: has_external_call
// Return: true if function is or contains a call to an external function
// external functions are not declared within the current module => library function
bool AdvisorInst::has_external_call(Function *F) {
	if (F->isDeclaration()) {
		return true;
	}

	bool result = does_function_call_external_function(callGraph->getOrInsertFunction(F));

	return result;
}

// Function: does_function_call_external_function
// Return: true if function contain a call to a function which is extenal to the module
// Always beware of recursive functions when dealing with the call graph
bool AdvisorInst::does_function_call_external_function(CallGraphNode *CGN) {
	if (CGN->getFunction()->isDeclaration()) {
		return true;
	}

	bool result = false;
	
	for (auto it = CGN->begin(), et = CGN->end(); it != et; it++) {
		CallGraphNode *calledGraphNode = it->second;
		*outputLog << "Found a call to function: " << calledGraphNode->getFunction()->getName() << "\n";
		if (std::find(recursiveFunctionList.begin(), recursiveFunctionList.end(), calledGraphNode->getFunction()) 
			== recursiveFunctionList.end()) {
			result |= does_function_call_external_function(calledGraphNode);
		} else {
			//errs() << __func__ << " is being ignored, it is recursive.\n";
		}
	}
	return result;
}

/*
// Function: instrument_function
// Instruments each function and the basic blocks contained in the function
// such that the insrumented IR will print each function execution as well
// as each basic block that is executed in the function
// e.g.) Entering Function: func
void AdvisorInst::instrument_function(Function *F) {
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
void AdvisorInst::instrument_basicblock(BasicBlock *BB) {
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
*/














