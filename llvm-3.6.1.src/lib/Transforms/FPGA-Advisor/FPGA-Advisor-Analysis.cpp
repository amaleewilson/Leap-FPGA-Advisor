//===- FPGA-Advisor-Analysis.cpp ---------------------------------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the FPGA-Advisor Analysis pass
// The analysis pass is intended to be run after the instrumentation pass. It
// also assumes that a program trace has been produced by the instrumented code.
// The analysis will reconstruct the trace from the file and perform analysis on
// the loops within the module.
//
// This file implements the FPGA-Advisor Analysis and pass
// This pass is used in the second stage of FPGA-Advisor tool and will provide
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
// Describe the analysis details here ----- I am still not sure how it will be
// implemented !!!!!!!!
//
//===----------------------------------------------------------------------===//
//
// Author: chenyuti
//
//===----------------------------------------------------------------------===//

#include "FPGA-Advisor-Analysis.h"
#include <algorithm>
#include <fstream>
#include <regex>

#define DEBUG_TYPE "fpga-advisor-analysis"

using namespace llvm;

using std::ifstream;

std::error_code AEC;

static cl::opt<std::string> TraceFileName("trace-file", cl::desc("Name of the trace file"), 
		cl::Hidden, cl::init("trace.log"));
static cl::opt<bool> IgnoreSanity("ignore-sanity", cl::desc("Enable to ignore trace sanity check"),
		cl::Hidden, cl::init(false));

// List of statistics -- not necessarily the statistics listed above, this is
// at a module level
STATISTIC(FunctionCounter, "Number of functions in module");
STATISTIC(BasicBlockCounter, "Number of basic blocks in all functions in module");
STATISTIC(LoopCounter, "Number of loops in all functions in module");
STATISTIC(ParallelizableLoopCounter, "Number of parallelizable loops in all functions in module");
STATISTIC(InstructionCounter, "Number of instructions in all functions in module");
STATISTIC(LoopInstructionCounter, "Number of instructions in all loops in all functions in module");
STATISTIC(ParallelizableLoopInstructionCounter, "Number of instructions in all parallelizable loops in all functions in module");


// Function: runOnModule
// This is the main analysis pass
bool AdvisorAnalysis::runOnModule(Module &M) {
	//=------------------------------------------------------=//
	// [1] Initialization
	//=------------------------------------------------------=//
	raw_fd_ostream OL("fpga-advisor-analysis.log", AEC, sys::fs::F_RW);
	outputLog = &OL;
	DEBUG(outputLog = &dbgs());

	*outputLog << "FPGA-Advisor Analysis Pass Starting.\n";

	mod = &M;

	//=------------------------------------------------------=//
	// [2] Static analyses and setup
	//=------------------------------------------------------=//
	callGraph = &getAnalysis<CallGraphWrapperPass>().getCallGraph();
	find_recursive_functions(M);

	// basic statistics gathering
	// also populates the functionMap
	visit(M);
	
	//=------------------------------------------------------=//
	// [3] Read trace from file into memory
	//=------------------------------------------------------=//
	if (! get_program_trace(TraceFileName)) {
		errs() << "Could not find trace file: " << TraceFileName << "!\n";
		return false;
	}

	// should also contain a sanity check to follow the trace and make sure
	// the paths are valid
	//if (! IgnoreSanity && ! check_trace_sanity()) {
	//	errs() << "Trace from file is broken, path does not follow control flow graph!\n";
	//	return false;
	//}

	//=------------------------------------------------------=//
	// [4] Analysis after dynamic feedback for each function
	//=------------------------------------------------------=//
	for (auto F = M.begin(), FE = M.end(); F != FE; F++) {
		run_on_function(F);
	}

	//=------------------------------------------------------=//
	// [5] Printout statistics
	//=------------------------------------------------------=//
	*outputLog << "Print static information\n";
	// pre-instrumentation statistics => work with uninstrumented code
	print_statistics();

	return true;
}


void AdvisorAnalysis::visitFunction(Function &F) {
	*outputLog << "visit Function: " << F.getName() << "\n";
	FunctionCounter++;

	// create and initialize a node for this function
	FunctionInfo *newFuncInfo = new FunctionInfo();
	newFuncInfo->function = &F;
	newFuncInfo->bbList.clear();
	newFuncInfo->instList.clear();
	newFuncInfo->loopList.clear();
	
	if (! F.isDeclaration()) {
		// only get the loop info for functions with a body, else will get assertion error
		newFuncInfo->loopInfo = &getAnalysis<LoopInfo>(F);
		*outputLog << "PRINTOUT THE LOOPINFO\n";
		newFuncInfo->loopInfo->print(*outputLog);
		*outputLog << "\n";
		// find all the loops in this function
		for (LoopInfo::reverse_iterator li = newFuncInfo->loopInfo->rbegin(), le = newFuncInfo->loopInfo->rend(); li != le; li++) {
			*outputLog << "Encountered a loop!\n";
			(*li)->print(*outputLog);
			*outputLog << "\n" << (*li)->isAnnotatedParallel() << "\n";
			// append to the loopList
			//newFuncInfo->loopList.push_back(*li);
			LoopIterInfo newLoop;
			//newLoop.loopInfo = *li;
			// how many subloops are contained within the loop
			*outputLog << "This natural loop contains " << (*li)->getSubLoops().size() << " subloops\n";
			newLoop.subloops = (*li)->getSubLoopsVector();
			*outputLog << "Copied subloops " << newLoop.subloops.size() << "\n";
			newLoop.maxIter = 0;
			newLoop.parIter = 0;
			newFuncInfo->loopList.push_back(newLoop);
		}
	}

	// insert into the map
	functionMap.insert( {&F, newFuncInfo} );
}

void AdvisorAnalysis::visitBasicBlock(BasicBlock &BB) {
	//*outputLog << "visit BasicBlock: " << BB.getName() << "\n";
	BasicBlockCounter++;

	// make sure function is in functionMap
	assert(functionMap.find(BB.getParent()) != functionMap.end());
	FunctionInfo *FI = functionMap.find(BB.getParent())->second;
	FI->bbList.push_back(&BB);
}

void AdvisorAnalysis::visitInstruction(Instruction &I) {
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
void AdvisorAnalysis::print_statistics() {
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
void AdvisorAnalysis::find_recursive_functions(Module &M) {
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
void AdvisorAnalysis::does_function_recurse(Function *func, CallGraphNode *CGN, std::vector<Function *> &stack) {
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


void AdvisorAnalysis::print_recursive_functions() {
	dbgs() << "Found recursive functions: \n";
	for (auto r = recursiveFunctionList.begin(), re = recursiveFunctionList.end(); r != re; r++) {
		Function *F = *r;
		dbgs() << "  " << F->getName() << "\n";
	}
}

// Function: run_on_function
// Return: false if function cannot be synthesized
// Function looks at the loops within the function
bool AdvisorAnalysis::run_on_function(Function *F) {
	*outputLog << "Examine function: " << F->getName() << "\n";
	// Find constructs that are not supported by HLS
	if (has_unsynthesizable_construct(F)) {
		*outputLog << "Function contains unsynthesizable constructs, moving on.\n";
		return false;
	}
	// now that we have the trace imported into memory, let's mark
	// each loop with a max iteration count
	// This is the algorithm:
	// 	For each loop:
	//		iterate through the trace
	//			if (currBB = loop.Header)
	//				maxIt++
	//			else if (currBB ! belongs in loop)
	//				loop.maxIt = MAX(loop.maxIt, maxIt)
	//				maxIt = 0
	//			else // currBB belongs in loop
	//				// do nothing

	// for each loop
	/*
	assert(functionMap.find(F) != functionMap.end());
	FunctionInfo *fInfo = (functionMap.find(F))->second;
	for (auto loopIt = fInfo->loopList.begin(); loopIt != fInfo->loopList.end(); loopIt++) {
		// iterate through the trace
		LoopIterInfo loopIterInfo = *loopIt;
		Loop *loop = loopIterInfo.loopInfo;
		BasicBlock *header = loop->getHeader();

		*outputLog << "Annotating loop: " << header->getName() << "\n";
		uint64_t maxIterations = 0;
		for (std::list<BasicBlock *>::iterator traceBB = executionTrace.begin(); traceBB != executionTrace.end(); traceBB++) {
			BasicBlock *currBB = *traceBB;
			if ((currBB)->getParent() != header->getParent()) {
				continue;
			}
			if (currBB == header) {
				maxIterations++;
			} else if (!loop->contains(currBB)) {
				loopIterInfo.maxIter = std::max(maxIterations, loopIterInfo.maxIter);
				maxIterations = 0;
			} else {
				// block belongs in loop
			}
		}
		*outputLog << "Max iteration is: " << loopIterInfo.maxIter << "\n";
	}
	*/

	// for each execution of the function found in the trace
	// we want to find the optimal tiling for the basicblocks
	// the starting point of the algorithm is the MOST parallel
	// configuration, which can be found by scheduling independent
	// sic blocks in the earliest cycle that it is allowed to be executed
	find_maximal_configuration_for_all_calls(F);

	return true;
}

// Function: has_unsynthesizable_construct
// Return: true if module contains code which is not able to be run through HLS tools
// These contain:
// 	- Recursion
// 	- Dynamic memory allocation
//	- Arbitrary pointer accesses
// 	- Some tools do not support pthread/openmp but LegUp does (so we will ignore it)
bool AdvisorAnalysis::has_unsynthesizable_construct(Function *F) {
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
bool AdvisorAnalysis::is_recursive_function(Function *F) {
	return (std::find(recursiveFunctionList.begin(), recursiveFunctionList.end(), F) 
			!= recursiveFunctionList.end());
}

// Function: has_recursive_call
// Return: true if function is recursive or contains a call to a recursive 
// function on the recursiveFunctionList
bool AdvisorAnalysis::has_recursive_call(Function *F) {
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
bool AdvisorAnalysis::does_function_call_recursive_function(CallGraphNode *CGN) {
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
bool AdvisorAnalysis::has_external_call(Function *F) {
	if (F->isDeclaration()) {
		return true;
	}

	bool result = does_function_call_external_function(callGraph->getOrInsertFunction(F));

	return result;
}

// Function: does_function_call_external_function
// Return: true if function contain a call to a function which is extenal to the module
// Always beware of recursive functions when dealing with the call graph
bool AdvisorAnalysis::does_function_call_external_function(CallGraphNode *CGN) {
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

// Function: get_program_trace
// Return: false if unsuccessful
// Reads input trace file, parses and stores trace into executionTrace map
bool AdvisorAnalysis::get_program_trace(std::string fileIn) {
	// clear the hash
	executionTrace.clear();

	// read file
	ifstream fin;
	fin.open(fileIn.c_str());
	if (!fin.good()) {
		return false; // file not found
	}
	
	std::string line;

	// unique ID for each basic block executed
	int ID = 0;

	while (std::getline(fin, line)) {
		// There are 3 types of messages:
		//	1. Enter Function: <func name>
		//	2. Basic Block: <basic block name> Function: <func name>
		//	3. Return from: <func name>
		if (std::regex_match(line, std::regex("(Entering Function: )(.*)"))) {
			// enter function into execution trace, if not already present
			const char *delimiter = " ";
			
			// make a non-const copy of line
			std::vector<char> lineCopy(line.begin(), line.end());
			lineCopy.push_back(0);

			//=----------------------------=//
			char *pch = std::strtok(&lineCopy[0], delimiter);
			// Entering
			pch = strtok(NULL, delimiter);
			// Function:
			pch = strtok(NULL, delimiter);
			// funcName
			std::string funcString(pch);
			//=----------------------------=//

			Function *F = find_function_by_name(funcString);
			if (!F) {
				// could not find the function by name
				errs() << "Could not find the function from trace in program!\n";
				return false;
			}
			
			//==----------------------------------------------------------------==//
			ExecTrace_iterator fTrace = executionTrace.find(F);
			if (fTrace == executionTrace.end()) {
				FuncExecTrace emptyList;
				executionTrace.insert(std::make_pair(F, emptyList));
				Trace newList;
				executionTrace[F].push_back(newList);
			} else {
				// function exists
				Trace newList;
				fTrace->second.push_back(newList);
			}
			//==----------------------------------------------------------------==//
			ExecGraph_iterator fGraph = executionGraph.find(F);
			if (fGraph == executionGraph.end()) {
				TraceGraphList emptyList;
				executionGraph.insert(std::make_pair(F, emptyList));
				TraceGraph newGraph;
				executionGraph[F].push_back(newGraph);
			} else {
				// function exists
				TraceGraph newGraph;
				fGraph->second.push_back(newGraph);
			}
			//==----------------------------------------------------------------==//
		} else if (std::regex_match(line, std::regex("(BasicBlock: )(.*)( Function: )(.*)"))) {
			// record this information
			const char *delimiter = " ";

			// make a non-const copy of line
			std::vector<char> lineCopy(line.begin(), line.end());
			lineCopy.push_back(0);

			//=----------------------------=//
			// BasicBlock:<space>bbName<space>Function:<space>funcName\n
			// separate out string by tokens
			char *pch = std::strtok(&lineCopy[0], delimiter);
			// BasicBlock:
			pch = strtok(NULL, delimiter);
			// bbName
			std::string bbString(pch);

			pch = strtok(NULL, delimiter);
			// Function:
			pch = strtok(NULL, delimiter);
			// funcName
			std::string funcString(pch);
			//=----------------------------=//

			BasicBlock *BB = find_basicblock_by_name(funcString, bbString);
			if (!BB) {
				// could not find the basicblock by name
				errs() << "Could not find the basicblock from trace in program!\n";
				return false;
			}
			// TODO We can do sanity checks here to make sure the path taken by the
			// trace is valid
			//executionTrace.push_back(BB);
			BBSchedElem newBB;
			newBB.basicblock = BB;
			newBB.ID = ID++;
			// mark the start and end cycles of unscheduled basic blocks as -ve
			// mark the end cycles as 'earlier' than start cycles
			newBB.cycStart = -1;
			newBB.cycEnd = -2;
			//==----------------------------------------------------------------==//
			executionTrace[BB->getParent()].back().push_back(newBB);
			//==----------------------------------------------------------------==//
			boost::add_vertex(newBB, executionGraph[BB->getParent()].back());
			boost::write_graphviz(std::cerr, executionGraph[BB->getParent()].back());
			//==----------------------------------------------------------------==//

			*outputLog << funcString << "(" << executionTrace[BB->getParent()].size() << ") " << bbString << "\n";
		} else if (std::regex_match(line, std::regex("(Return from: )(.*)"))) {
			// nothing to do really...
		} else {
			errs() << "Unexpected trace input!\n" << line << "\n";
			return false;
		}
	}
	
	// print some debug output

	return true;
}

/*
// TODO TODO TODO TODO TODO remember to check for external functions, I know
// you're going to forget this!!!!!!!!
bool AdvisorAnalysis::check_trace_sanity() {
	// run through the trace to see if these are valid paths
	// Here are the rules:
	//	1. The first basic block executed in the trace does not necessarily
	//		have to be the entry block in the main program, we are allowing
	//		traces to start somewhere arbitrarily
	//	2. For basicblock B executing after A within the same function, B
	//		must be one of the successors of A
	//	3. For basicblock B executing after A across function boundaries,
	//		we will use a stack to trace the function execution order and
	//		the function A belongs to musts have been the caller of the
	//		function B belongs to
	//std::stack<Function *> funcStack;
	std::string funcStr = executionTrace.front().first;
	std::string bbStr = executionTrace.front().second;
	
	BasicBlock *currBB = find_basicblock_by_name(funcStr, bbStr);
	if (!currBB) {
		// did not find basic block
		return false;
	}

	return trace_cfg(currBB);
}

// Function: trace_cfg
// Return: false if the dcfg trace does not follow the cfg
bool AdvisorAnalysis::trace_cfg(BasicBlock *startBB) {
	// TODO TODO TODO FIXME FIXME FIXME
	return true; // for now, do this function later
	// TODO TODO TODO FIXME FIXME FIXME
	// keep track of a function stack
	std::stack<Function *> funcStack;
	BasicBlock *currBB = startBB;

	// push the starting function onto stack
	funcStack.push_back(currBB->getParent());

	// iterate through trace
	for (auto it = executionTrace.begin(), et = executionTrace.end(); it != et; it++) {
		// check with stack that it is consistent
		if (strcmp(it->first, funcStack.top()->getName().str().c_str()) != 0) {
			// did we call another function? or did we return from a function?!!
		}
	}
}
*/

// TODO: there's probably a much better way than this...
// TODO: but I need to show results!!!
// Function: find_basicblock_by_name
// Return: Pointer to the basic block belonging to function and basic block, NULL if
// not found
BasicBlock *AdvisorAnalysis::find_basicblock_by_name(std::string funcName, std::string bbName) {
	for (auto F = mod->begin(), FE = mod->end(); F != FE; F++) {
		if (strcmp(funcName.c_str(), F->getName().str().c_str()) != 0) {
			continue;
		}
		for (auto BB = F->begin(), BE = F->end(); BB != BE; BB++) {
			if (strcmp(bbName.c_str(), BB->getName().str().c_str()) == 0) {
				return BB;
			}
		}
	}
	return NULL;
}

// Function: find_function_by_name
// Return: Pointer to the function belonging to function, NULL if not found
Function *AdvisorAnalysis::find_function_by_name(std::string funcName) {
	for (auto F = mod->begin(), FE = mod->end(); F != FE; F++) {
		if (strcmp(funcName.c_str(), F->getName().str().c_str()) != 0) {
			continue;
		}
		return F;
	}
	return NULL;
}

// Function: find_maximal_configuration_for_all_calls
// Return: true if successful, else false
// This function will find the maximum needed tiling for a given function
// across all individual calls within the trace
// Does not look across function boundaries
// The parallelization factor will be stored in metadata for each basicblock
bool AdvisorAnalysis::find_maximal_configuration_for_all_calls(Function *F) {
	assert(executionTrace.find(F) != executionTrace.end());
	bool scheduled = false;
	// iterate over all calls
	for (FuncExecTrace_iterator fIt = executionTrace[F].begin(); fIt != executionTrace[F].end(); fIt++) {
		//scheduled |= find_maximal_configuration_for_call(F, fIt);
	}
	return scheduled;
}

// Function: find_maximal_configuration_for_call
// Return: true if successful, else false
// This function will find the maximum needed tiling for a given function
// for one call/run of the function
// The parallelization factor will be stored in metadata for each basicblock
/*
bool AdvisorAnalysis::find_maximal_configuration_for_call(Function *F, FuncExecTrace_iterator trace) {

		BasicBlock *entryBB = &(F->getEntryBlock());
		// build some data structure for anti-chain

		// for each separate call, iterate across the trace for that call instance
		for (Trace_iterator bbIt = trace->begin(); bbIt != trace->end(); trace++) {
			BasicBlock *currBB = bbIt->basicblock;
			// the scheduling of basicblocks will occur through the marking of
			// basic block scheduling elements with start and end cycles corresponding
			// to when the basic block can start executing.
			// TODO FIXME: For now, let's just assume each basic block takes 1 cycle

			//=-----------------------------------------------------------------=//
			// How to schedule the basic blocks:
			// 1) Basic blocks whose only dependencies are outside of the function
			//		can all be scheduled right away
			// 2) Basic blocks whose preceding basic block schedule elements in
			//		the function call trace has been scheduled
			// 3) Basic blocks whose dependent basic blocks have all been scheduled
			//=-----------------------------------------------------------------=//
			
			//=-----------------------------------------------------------------=//
			// When is a basic block ready to be scheduled:
			// The trace should start at the entry block, the first entry block
			// should be ready to be scheduled.
			// Blocks whose parent has been scheduled are ready to be scheduled
			//=-----------------------------------------------------------------=//
			
			// each function can only have one entry point, can it be part of a loop? I should think not..
			if (currBB ) {
			}

		}

		return true;
}
*/


