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

// FIXME Need to change the direction of the trace graph.... sighh

#include "fpga_common.h"

#include <fstream>
#include <fstream>
#include <regex>
#include <time.h>

#define DEBUG_TYPE "fpga-advisor-analysis"

using namespace llvm;
using namespace fpga;
using std::ifstream;

//===----------------------------------------------------------------------===//
// Some globals ... is that bad? :/
// I can move this into the class and clean up every time a different function
// is being analyzed. The problem is that I have written a module pass.
//===----------------------------------------------------------------------===//

std::error_code AEC;
MemoryDependenceAnalysis *MDA;
DominatorTree *DT;
DepGraph *depGraph;
// latency table
std::map<BasicBlock *, int> *LT;
// area table
std::map<BasicBlock *, int> *AT;
int cpuCycle;

//===----------------------------------------------------------------------===//
// Advisor Analysis Pass options
//===----------------------------------------------------------------------===//

static cl::opt<std::string> TraceFileName("trace-file", cl::desc("Name of the trace file"), 
		cl::Hidden, cl::init("trace.log"));
static cl::opt<bool> IgnoreSanity("ignore-sanity", cl::desc("Enable to ignore trace sanity check"),
		cl::Hidden, cl::init(false));
static cl::opt<bool> HideGraph("hide-graph", cl::desc("If enabled, disables printing of dot graphs"),
		cl::Hidden, cl::init(false));
static cl::opt<bool> NoMessage("no-message", cl::desc("If enabled, disables printing of messages for debug"),
		cl::Hidden, cl::init(false));

//===----------------------------------------------------------------------===//
// List of statistics -- not necessarily the statistics listed above,
// this is at a module level
//===----------------------------------------------------------------------===//

STATISTIC(FunctionCounter, "Number of functions in module");
STATISTIC(BasicBlockCounter, "Number of basic blocks in all functions in module");
STATISTIC(InstructionCounter, "Number of instructions in all functions in module");
//STATISTIC(LoopCounter, "Number of loops in all functions in module");
//STATISTIC(ParallelizableLoopCounter, "Number of parallelizable loops in all functions in module");
//STATISTIC(LoopInstructionCounter, "Number of instructions in all loops in all functions in module");
//STATISTIC(ParallelizableLoopInstructionCounter, "Number of instructions in all parallelizable loops in all functions in module");
STATISTIC(ConvergenceCounter, "Number of steps taken to converge in gradient descent optimization");

//===----------------------------------------------------------------------===//
// Helper functions
//===----------------------------------------------------------------------===//
//template <typename T> void output_dot_graph(std::ostream stream, T const &g) {
//	boost::write_graphviz(stream, g);
//}


//===----------------------------------------------------------------------===//
// AdvisorAnalysis Class functions
//===----------------------------------------------------------------------===//

// Function: runOnModule
// This is the main analysis pass
bool AdvisorAnalysis::runOnModule(Module &M) {
	std::cerr << "Starting FPGA Advisor Analysis Phase...\n";
	// take some run time stats
	clock_t start, end;
	start = clock();
	//=------------------------------------------------------=//
	// [1] Initialization
	//=------------------------------------------------------=//
	raw_fd_ostream OL("fpga-advisor-analysis.log", AEC, sys::fs::F_RW);
	outputLog = &OL;
	if (NoMessage) {
		outputLog = &nulls();
	} else {
		DEBUG(outputLog = &dbgs());
	}
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
	//*outputLog << "Print static information\n"; // there is no need to announce this..
	// pre-instrumentation statistics => work with uninstrumented code
	print_statistics();

	end = clock();
	float timeElapsed(((float) end - (float) start) / CLOCKS_PER_SEC);
	std::cerr << "TOTAL ANALYSIS RUNTIME: " << timeElapsed << " seconds\n";

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

	LT = &getAnalysis<FunctionScheduler>(*F).getLatencyTable();
	AT = &getAnalysis<FunctionAreaEstimator>(*F).getAreaTable();

	// get the dependence graph for the function
	depGraph = &getAnalysis<DependenceGraph>(*F).getDepGraph();

	// for each execution of the function found in the trace
	// we want to find the optimal tiling for the basicblocks
	// the starting point of the algorithm is the MOST parallel
	// configuration, which can be found by scheduling independent
	// sic blocks in the earliest cycle that it is allowed to be executed
	find_maximal_configuration_for_all_calls(F);

	*outputLog << "Maximal basic block configuration.\n";
	print_basic_block_configuration(F);

	std::cerr << "Finished computing maximal configuration\n";

	// by this point, the basic blocks have been annotated by the maximal
	// replication factor
	// build a framework that is able to methodically perturb the basic block
	// to follow the gradient descent method of restricting basic block
	// replication to achieve most optimal area-latency result
	// Description of gradient descent method:
	//	- determine differential in performance/area for each basic block
	//		i.e. reduce the basic block resource by 1 to determine the 
	//		impact on performance
	//	- move in the direction of maximum performance/area
	//		i.e. reduce the basic block which provides the least performance/area
	//	- for now, we will finish iterating when we find a local maximum of performance/area
	find_optimal_configuration_for_all_calls(F);

	*outputLog << "===-------------------------------------===";
	*outputLog << "Final optimal basic block configuration.\n";
	print_basic_block_configuration(F);
	*outputLog << "===-------------------------------------===";

	if (!HideGraph) {
		print_optimal_configuration_for_all_calls(F);
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
// TODO: do not add to trace the basic blocks which have only a branch instruction
bool AdvisorAnalysis::get_program_trace(std::string fileIn) {
	// clear the hash
	//executionTrace.clear();

	// read file
	ifstream fin;
	fin.open(fileIn.c_str());
	if (!fin.good()) {
		return false; // file not found
	}
	
	std::string line;

	// unique ID for each basic block executed
	int ID = 0;

	TraceGraph::vertex_descriptor prevVertex = 0;

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
			//ExecTrace_iterator fTrace = executionTrace.find(F);
			//if (fTrace == executionTrace.end()) {
			//	FuncExecTrace emptyList;
			//	executionTrace.insert(std::make_pair(F, emptyList));
			//	Trace newList;
			//	executionTrace[F].push_back(newList);
			//} else {
			//	// function exists
			//	Trace newList;
			//	fTrace->second.push_back(newList);
			//}
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

			// FIXME BOOKMARK
			if (isa<TerminatorInst>(BB->getFirstNonPHI())) {
				// if the basic block only contains a branch/control flow and no computation
				// then skip it, do not add to graph
				// TODO if this is what I end up doing, need to remove looking at these
				// basic blocks when considering transitions ?? I think that already happens.
				continue;
			}

			// TODO We can do sanity checks here to make sure the path taken by the
			// trace is valid
			//executionTrace.push_back(BB);
			//==----------------------------------------------------------------==//
			//BBSchedElem newBB;
			//newBB.basicblock = BB;
			//newBB.ID = ID;
			// mark the start and end cycles of unscheduled basic blocks as -ve
			// mark the end cycles as 'earlier' than start cycles
			//newBB.minCycStart = -1;
			//newBB.minCycEnd = -2;
			//executionTrace[BB->getParent()].back().push_back(newBB);
			//*outputLog << funcString << "(" << executionTrace[BB->getParent()].size() << ") " << bbString << "\n";
			//==----------------------------------------------------------------==//
			TraceGraph::vertex_descriptor currVertex = boost::add_vertex(executionGraph[BB->getParent()].back());
			TraceGraph &currGraph = executionGraph[BB->getParent()].back();
			currGraph[currVertex].basicblock = BB;
			currGraph[currVertex].ID = ID;
			currGraph[currVertex].minCycStart = -1;
			currGraph[currVertex].minCycEnd = -1;
			currGraph[currVertex].name = BB->getName().str();
			if (currVertex != prevVertex) {
				// A -> B means that A depends on the completion of B
				// initial edge weight is 0, assume all performed on fpga
				boost::add_edge(prevVertex, currVertex, 0, currGraph);
				//boost::add_edge(currVertex, prevVertex, currGraph);
			}
			prevVertex = currVertex;
			//boost::write_graphviz(std::cerr, executionGraph[BB->getParent()].back());
			//==----------------------------------------------------------------==//

			// increment the node ID
			ID++;
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
	*outputLog << __func__ << " for function " << F->getName() << "\n";;
	//assert(executionTrace.find(F) != executionTrace.end());
	assert(executionGraph.find(F) != executionGraph.end());
	bool scheduled = false;

	initialize_basic_block_instance_count(F);

	// The ending condition should be determined by the user input of acceptable
	// area and latency constraints
	//while (1) {
		// iterate over all calls
		*outputLog << "There are " << executionGraph[F].size() << " calls to " << F->getName() << "\n";
		for (TraceGraphList_iterator fIt = executionGraph[F].begin(); 
				fIt != executionGraph[F].end(); fIt++) {
			std::vector<TraceGraph_vertex_descriptor> rootVertices;
			rootVertices.clear();
			scheduled |= find_maximal_configuration_for_call(F, fIt, rootVertices);
			// after creating trace graphs representing maximal parallelism
			// compute maximal tiling
			//find_maximal_tiling_for_call(F, fIt);

			TraceGraph graph = *fIt;
			*outputLog << "root vertices are: ";
			for (auto rV = rootVertices.begin(); rV != rootVertices.end(); rV++) {
				*outputLog << "root: [" << *rV << "]->" << graph[*rV].name << "\n";
			}
	
			int lastCycle = -1;
	
			// annotate each node with the start and end cycles
			scheduled |= annotate_schedule_for_call(F, fIt, rootVertices, lastCycle);
	
			*outputLog << "Last Cycle: " << lastCycle << "\n";

			// after creating trace graphs, find maximal resources needed
			// to satisfy longest antichain
			scheduled |= find_maximal_resource_requirement(F, fIt, rootVertices, lastCycle);
	
			// use gradient descent method
			//modify_resource_requirement(F, fIt);

		}
	//}

	return scheduled;
}


// Function: find_maximal_configuration_for_call
// Return: true if successful, else false
// This function will find the maximum needed tiling for a given function
// for one call/run of the function
// The parallelization factor will be stored in metadata for each basicblock
bool AdvisorAnalysis::find_maximal_configuration_for_call(Function *F, TraceGraphList_iterator graph_it, std::vector<TraceGraph_vertex_descriptor> &rootVertices) {
	*outputLog << __func__ << " for function " << F->getName() << "\n";
	// for each node N in trace graph G
		// for each parent node P of N
			// if N can execute in parallel to P, set P.parent as N.parent
	//TraceGraph graph = *graph_it;
	TraceGraphList_iterator graph = graph_it;
	TraceGraph_iterator vi, ve;

	// print the schedule graph before transformation
	if (!HideGraph) {
		/*
		// for printing labels in graph output
		boost::dynamic_properties dpTG;
		dpTG.property("label", get(&BBSchedElem::name, *graph));
		dpTG.property("id", get(&BBSchedElem::ID, *graph));
		dpTG.property("start", get(&BBSchedElem::minCycStart, *graph));
		dpTG.property("end", get(&BBSchedElem::minCycEnd, *graph));
		boost::write_graphviz_dp(std::cerr, *graph, dpTG, std::string("id"));
		*/
		TraceGraphVertexWriter<TraceGraph> vpw(*graph);
		TraceGraphEdgeWriter<TraceGraph> epw(*graph);
		std::ofstream outfile("initial.dot");
		boost::write_graphviz(outfile, *graph, vpw, epw);
	}

	for (boost::tie(vi, ve) = boost::vertices(*graph); vi != ve; vi++) {
		*outputLog << "*** working on vertex " << (*graph)[*vi].basicblock->getName() << " (" << *vi << ")\n";
		std::cerr << "*** working on vertex " << (*graph)[*vi].basicblock->getName().str() << " (" << *vi << ")\n";

		TraceGraph_in_edge_iterator ii, ie;
		//TraceGraph_out_edge_iterator oi, oe;
		std::vector<TraceGraph_vertex_descriptor> newParents;
		std::vector<TraceGraph_vertex_descriptor> oldParents;
		for (boost::tie(ii, ie) = boost::in_edges(*vi, *graph); ii != ie; ii++) {
			//std::cerr << "111\n";
			TraceGraph_vertex_descriptor parent = boost::source(*ii, *graph);
			*outputLog << "=== examine edge === " << parent << "->" << *vi << " / " << (*graph)[parent].basicblock->getName() << "->" << (*graph)[*vi].name << "\n";
			find_new_parents(newParents, *vi, parent, *graph);
			//*outputLog << "=== examined edge " << *vi << " -> " << parent << "\n";
			oldParents.push_back(parent);
		}
		//std::cerr << "222\n";

		// remove edges -> they will be replaced later
		// remember to also add edges from child to parents that are no longer my parents
		// so that we keep that dependence
		boost::clear_in_edges(*vi, *graph);

		// if vertex now has no parents, it is another root
		if (newParents.size() == 0) {
			rootVertices.push_back(*vi);
		}

		//std::cerr << "333\n";
		// sort the two lists for easier comparison
		std::sort(newParents.begin(), newParents.end());
		std::sort(oldParents.begin(), oldParents.end());


		for (unsigned i = 0; i < newParents.size(); i++) {
			//std::cerr << "444\n";
	*outputLog << "+++ new parent add edge +++ " << newParents[i] << "->" << *vi << " / " << (*graph)[newParents[i]].basicblock->getName() << "->" << (*graph)[*vi].name << "\n";
			// add edges
			// initial edge weight is 0, no fpga<->cpu transitions
			boost::add_edge(newParents[i], *vi, 0, *graph);
			//boost::add_edge(*vi, newParents[i], *graph);
			//*outputLog << "+++ added edge " << newParents[i] << "->" << *vi << "\n";
			auto search = std::find(oldParents.begin(), oldParents.end(), newParents[i]);
			if (search != oldParents.end()) {
				oldParents.erase(search);
			}
		}

		*outputLog << "Abandoned old parents: " << oldParents.size() << "\n";
		// if any oldParents still exist, they have been abandoned, so connect my children to them
		//std::cerr << "555\n";
		for (unsigned i = 0; i < oldParents.size(); i++) {
			//std::cerr << "666\n";
			// add edge from child to old parents
			TraceGraph_out_edge_iterator oi, oe;
			for (boost::tie(oi, oe) = boost::out_edges(*vi, *graph); oi != oe; oi++) {
				TraceGraph_vertex_descriptor child = boost::target(*oi, *graph);
				*outputLog << "+++ old parent add edge +++ " << oldParents[i] << "->" << child << " / " << (*graph)[oldParents[i]].basicblock->getName() << "->" << (*graph)[child].name << "\n";
				// initial edge weight is 0, no fpga<->cpu transitions
				boost::add_edge(oldParents[i], child, 0, *graph);
				//*outputLog << "+++ added edge " << oldParents[i] << " -> " << child << "\n";
			}
		}
		//std::cerr << "777\n";
	}
	//std::cerr << "888\n";

	// print graph after modification
	if (!HideGraph) {
		TraceGraphVertexWriter<TraceGraph> vpw(*graph);
		TraceGraphEdgeWriter<TraceGraph> epw(*graph);
		std::ofstream outfile("maximal.dot");
		boost::write_graphviz(outfile, *graph, vpw, epw);
		//boost::write_graphviz_dp(std::cerr, *graph, dpTG, std::string("id"));
	}

	//std::cerr << "999\n";
	return false;
}


// Function: initialize_basic_block_instance_count
// Initializes the replication factor metadata for each basic block in function to zero
void AdvisorAnalysis::initialize_basic_block_instance_count(Function *F) {
	for (auto BB = F->begin(); BB != F->end(); BB++) {
		set_basic_block_instance_count(BB, 0);
	}
}

#if 0
// Function: find_maximal_configuration_for_call
// Return: true if successful, else false
// This function will find the maximum needed tiling for a given function
// for one call/run of the function
// The parallelization factor will be stored in metadata for each basicblock
bool AdvisorAnalysis::find_maximal_configuration_for_call(Function *F, TraceGraphList_iterator graph_it) {

		BasicBlock *entryBB = &(F->getEntryBlock());
		// build some data structure for anti-chain

		/*
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
		*/

		// Algorithm description:
		// 	for each node N in the graph G
		// 		for each parent node P of N
		//			recursively: if N does not depend on P:
		//				P->N->children, P->parent->N

		// get the memory dependence analysis for function
		MDA = &getAnalysis<MemoryDependenceAnalysis>(*F);
		assert(MDA);
		
		// get the dominator tree for function
		DT = &getAnalysis<DominatorTreeWrapperPass>(*F).getDomTree();
		assert(DT);
		
		std::pair<TraceGraph_iterator, TraceGraph_iterator> p;
		TraceGraph graph = *graph_it;
		// print out what the schedule graph looks like before
		//boost::write_graphviz(std::cerr, graph);
		p = vertices(graph);
		bool changed = true;
		while (changed) {
			changed = false;
			*outputLog << "Another one\n";
			for (TraceGraph_iterator gIt = p.first; gIt != p.second; gIt++) {
				*outputLog << "***\n";
				TraceGraph_vertex_descriptor self = *gIt;
				*outputLog << "Vertex " << self << ": " << graph[self].basicblock->getName() << "\n";

				// A -> B means A is dependent on B
				// get all the out edges
				TraceGraph_out_edge_iterator oi, oe;
				for (boost::tie(oi, oe) = boost::out_edges(self, graph); oi != oe; oi++) {
					TraceGraph_vertex_descriptor parent = boost::target(*oi, graph);
					*outputLog << "Out edge of " << self << " points to " << parent << "\n";
					if (basicblock_is_dependent(graph[self].basicblock, graph[parent].basicblock, graph)) {
						continue;
					} else {
						*outputLog << "No data dependencies between basicblocks " << self << "\n";
					}

					if (basicblock_control_flow_dependent(graph[self].basicblock, graph[parent].basicblock, graph)) {
						//continue;
					}

					//*outputLog << "No control flow dependencies between basicblocks " << self << "\n";



					// inherit the parents of the parent
					// add edge from self to parents of parent
					TraceGraph_out_edge_iterator pi, pe;
					for (boost::tie(pi, pe) = boost::out_edges(parent, graph); pi != pe; pi++) {
						TraceGraph_vertex_descriptor grandparent = boost::target(*pi, graph);
						boost::add_edge(self, grandparent, graph);
					}

					// connect parent to my children if children depend on parents
					TraceGraph_in_edge_iterator ci, ce;
					for (boost::tie(ci, ce) = boost::in_edges(self, graph); ci != ce; ci++) {
						TraceGraph_vertex_descriptor children = boost::source(*ci, graph);
						if (basicblock_is_dependent(graph[children].basicblock, graph[parent].basicblock, graph)) {
							boost::add_edge(children, parent, graph);
						}
					}
					
					// remove edge from self to parent
					boost::remove_edge(self, parent, graph);

					changed = true; // FIXME FIXME FIXME only executing 1 cycle of this now
				}
			}
			// print out what the schedule graph looks like after each transformation
			//boost::write_graphviz(std::cerr, graph);
		}

		*outputLog << "Final form: ";

		// print out what the schedule graph looks like after
		//boost::write_graphviz(std::cerr, graph);

		return true;
}
#endif

// Function: basicblock_is_dependent
// Return: true if child is dependent on parent and must execute after parent
bool AdvisorAnalysis::basicblock_is_dependent(BasicBlock *child, BasicBlock *parent, TraceGraph &graph) {
	// use dependence analysis to determine if basic block is dependent on another
	// we care about true dependencies and control flow dependencies only
	// true dependencies include any of these situations:
	// 	- if any instruction in the child block depends on an output produced from the
	//	parent block
	//	- how do we account for loop dependencies??
	// compare each instruction in the child to the parent
	bool dependent = false;
	for (auto cI = child->begin(); cI != child->end(); cI++) {
		for (auto pI = parent->begin(); pI != parent->end(); pI++) {
			dependent |= instruction_is_dependent(cI, pI);
		}
	}

	return dependent;
}


// Function: instruction_is_dependent
// Return: true if instruction inst1 is dependent on inst2 and must be executed after inst2
bool AdvisorAnalysis::instruction_is_dependent(Instruction *inst1, Instruction *inst2) {
	bool dependent = false;
	// handle different instruction types differently
	// namely, for stores and loads we need to consider memory dependence analysis stufffff
	// flow dependence exists at two levels:
	//	1) inst1 directly consumes the output of inst2
	//		E.g.
	//			a = x + y
	//			b = load(a)
	//	Memory data dependence analysis:
	//	2) inst2 modifies memory which inst1 requires
	//		E.g.
	//			store(addr1, x)
	//			a = load(addr1)
	//	3) inst2 modifies a memory location which inst1 also modifies!?!?
	//		E.g.
	//			store(addr1, x)
	//			...
	//			store(addr2, y)
	//		Although, you could argue we would just get rid of the first store in this case
	//	4) inst1 modifies memory which inst2 first reads
	//		E.g.
	//			a = load(addr1)
	//			...
	//			store(addr1, x)
	// 1)
	if (true_dependence_exists(inst1, inst2)) {
		return true;
	}

	// only look at memory instructions
	// but don't care if both are loads
	if (inst1->mayReadOrWriteMemory() && inst2->mayReadOrWriteMemory()
		&& !(inst1->mayReadFromMemory() && inst2->mayReadFromMemory())) {
		*outputLog << "Looking at memory instructions: ";
		inst1->print(*outputLog);
		*outputLog << " & ";
		inst2->print(*outputLog);
		*outputLog << "\n";
		MemDepResult MDR = MDA->getDependency(inst1);
		if (Instruction *srcInst = MDR.getInst()) {
			if (srcInst == inst2) {
				*outputLog << "There is a memory dependence: ";
				inst1->print(*outputLog);
				*outputLog << " is dependent on ";
				srcInst->print(*outputLog);
				*outputLog << "\n";
				dependent |= true;
			} else {
				// inst1 is not dependent on inst2
			}
		} else {
			// Other:
			// Could be non-local to basic block => it is
			// Could be non-local to function
			// Could just be unknown
			if (MDR.isNonLocal()) {
				// this is what we expect...
				*outputLog << "Non-local dependency\n";
				/*
				MemDepResult nonLocalMDR = MDA->getDependency(inst1).getNonLocal();
				*outputLog << "---" << nonLocalMDR.isNonLocal() <<  " " << MDR.isNonLocal() << "\n";
				if (Instruction *srcInst = nonLocalMDR.getInst()) {
					*outputLog << "Source of dependency: ";
					srcInst->print(*outputLog);
					*outputLog << "\n";
					if (srcInst == inst2) {
						*outputLog << "There is a memory dependence: ";
						inst1->print(*outputLog);
						*outputLog << " is dependent on ";
						srcInst->print(*outputLog);
						*outputLog << "\n";
						dependent |= true;
					}
				}*/

				SmallVector<NonLocalDepResult, 0> queryResult;
				MDA->getNonLocalPointerDependency(inst1, queryResult);
				// scan the query results to see if inst2 is in this set
				//*outputLog << "query result size " << queryResult.size() < "\n";
				for (SmallVectorImpl<NonLocalDepResult>::const_iterator qi = queryResult.begin(); qi != queryResult.end(); qi++) {
					// which basic block is this dependency originating from
					NonLocalDepResult NLDR = *qi;
					const MemDepResult nonLocalMDR = NLDR.getResult();

					*outputLog << "entry ";
					if (Instruction *srcInst = nonLocalMDR.getInst()) {
						srcInst->print(*outputLog);
						if (srcInst == inst2) {
							dependent |= true;
						}
					}
					*outputLog << "\n";
				}
			} else if (MDR.isNonFuncLocal()) {
				*outputLog << "Non-func-local dependency\n";
				// nothing.. this is fine
				// this is beyond our scope
			} else {
				*outputLog << "UNKNOWN\n";
				// unknown, so we will mark as dependent
				dependent |= true;
			}
		}
	}

	return dependent;
}

// Function: true_dependence_exists
// Returns: true if there is a flow dependence flowing from inst2 to inst1
// i.e. inst1 must execute after inst2
bool AdvisorAnalysis::true_dependence_exists(Instruction *inst1, Instruction *inst2) {
	// look at each operand of inst1
	User *user = dyn_cast<User>(inst1);
	assert(user);

	Value *val2 = dyn_cast<Value>(inst2);
	for (auto op = user->op_begin(); op != user->op_end(); op++) {
		Value *val1 = op->get();
		if (val1 == val2) {
			*outputLog << "True dependency exists: ";
			inst1->print(*outputLog);
			*outputLog << ", ";
			inst2->print(*outputLog);
			*outputLog << "\n";
			return true;
		}
	}

	return false;
}

// Function: basicblock_control_flow_dependent
// Return: true if child must execute after parent
// A child basic block must execute after the parent basic block if either
// 	1) parent does not unconditionally branch to child
//	2) child is not a dominator of parent
bool AdvisorAnalysis::basicblock_control_flow_dependent(BasicBlock *child, BasicBlock *parent, TraceGraph &graph) {
	
	TerminatorInst *TI = parent->getTerminator();
	BranchInst *BI = dyn_cast<BranchInst>(TI);
	if (BI && BI->isUnconditional() && (BI->getSuccessor(0) == child)) {
		*outputLog << "no control flow dependence " << parent->getName() << " uncond branch to " << child->getName() << "\n";
		return false;
	}

	// dominates -- do not use properlyDominates because it may be the same basic block
	// check if child dominates parent
	if (DT->dominates(DT->getNode(child), DT->getNode(parent))) {
		*outputLog << "no control flow dependence " << child->getName() << " dominates " << parent->getName() << "\n";
		return false;
	}

	*outputLog << "control flow dependency exists. " << child->getName() << " & " << parent->getName() << "\n";
	
	return true;
}


void AdvisorAnalysis::find_new_parents(std::vector<TraceGraph_vertex_descriptor> &newParents, TraceGraph_vertex_descriptor child, TraceGraph_vertex_descriptor parent, TraceGraph &graph) {
	//std::cerr << __func__ << " parent: " << graph[parent].name << "\n";
	if (parent == child) {
		assert(0);
	}

	// find the corresponding vertices on the DG
	BasicBlock *childBB = graph[child].basicblock;
	BasicBlock *parentBB = graph[parent].basicblock;

	*outputLog << "Tracing through the execution graph -- child: " << childBB->getName() 
				<< " parent: " << parentBB->getName() << "\n";

	// if the child basic block only has one instruction and it is a branch/control flow
	// instruction, it needs to remain in place FIXME???
	//if (isa<TerminatorInst>(childBB->getFirstNonPHI())) {
	//	newParents.push_back(parent);
	//	return;
	//}

	// if childBB can execute in parallel with parentBB i.e. childBB does not depend on parentBB
	// then childBB can be moved up in the graph to inherit the parents of the parentBB
	// this is done recursively until we find the final parents of the childBB whose execution
	// the childBB *must* follow
	if (DependenceGraph::is_basic_block_dependent(childBB, parentBB, *depGraph)) {
		*outputLog << "Must come after parent: " << parentBB->getName() << "\n";
		if (std::find(newParents.begin(), newParents.end(), parent) == newParents.end()) {
			newParents.push_back(parent);
		}
		return;
	} else {
		TraceGraph_in_edge_iterator ii, ie;
		for (boost::tie(ii, ie) = boost::in_edges(parent, graph); ii != ie; ii++) {
			TraceGraph_vertex_descriptor grandparent = boost::source(*ii, graph);
			find_new_parents(newParents, child, grandparent, graph);
		}
	}
}


// Function: find_maximal_tiling_for_call
// This function finds the maximal tiling needed to get 
//void AdvisorAnalysis::find_maximal_tiling_for_call(Function *F, TraceGraphList_iterator graph_it) {
	//TraceGraph graph = *graph_it;
//}


// Function: annotate_schedule_for_call
// Return: true if successful, false otherwise
bool AdvisorAnalysis::annotate_schedule_for_call(Function *F, TraceGraphList_iterator graph_it, std::vector<TraceGraph_vertex_descriptor> &rootVertices, int &lastCycle) {
	// get the graph
	TraceGraphList_iterator graph = graph_it;
	//TraceGraph graph = *graph_it;

	// get the vertices which have no parents
	// these are the vertices that can be scheduled right away
	/*
	for (std::vector<TraceGraph_vertex_descriptor>::iterator rV = rootVertices.begin();
			rV != rootVertices.end(); rV++) {
		ScheduleVisitor vis(graph, *LT, lastCycle);
		*outputLog << "bfs on root " << (*graph)[*rV].name << "\n";
		//boost::breadth_first_search(*graph, vertex(0, *graph), boost::visitor(vis).root_vertex(*rV));
		boost::depth_first_search(*graph, boost::visitor(vis).root_vertex(*rV));
	}*/

	// use depth first visit to discover all the vertices in the graph
	// do not need to give the root node of each disconnected subgraph
	// use dfs instead of bfs because bfs only traverses nodes through a graph
	// that is *reachable* from a starting node
	// also, since there are no resource constraints, each basic block
	// will be scheduled as early as possible, so no need for bfs here
	ScheduleVisitor vis(graph, *LT, lastCycle);
	boost::depth_first_search(*graph, boost::visitor(vis));


	// for printing labels in graph output
	if (!HideGraph) {
		/*
		boost::dynamic_properties dpTG;
		//dpTG.property("label", boost::make_label_writer(boost::get(&BBSchedElem::name, *graph), boost::get(&BBSchedElem::minCycStart, *graph)));
		//dpTG.property("label", get(&BBSchedElem::name, *graph));
		dpTG.property("id", get(&BBSchedElem::ID, *graph));
		dpTG.property("start", get(&BBSchedElem::minCycStart, *graph));
		dpTG.property("end", get(&BBSchedElem::minCycEnd, *graph));
		boost::write_graphviz_dp(std::cerr, *graph, dpTG, std::string("id"));
		*/
		TraceGraphVertexWriter<TraceGraph> vpw(*graph);
		TraceGraphEdgeWriter<TraceGraph> epw(*graph);
		std::ofstream outfile("maximal_schedule.dot");
		boost::write_graphviz(outfile, *graph, vpw, epw);
	}

	return true;
}


// Function: find_maximal_resource_requirement
// Return: true if successful, false otherwise
bool AdvisorAnalysis::find_maximal_resource_requirement(Function *F, TraceGraphList_iterator graph_it, std::vector<TraceGraph_vertex_descriptor> &rootVertices, int lastCycle) {
	*outputLog << __func__ << "\n";

	// get the graph
	TraceGraphList_iterator graph = graph_it;
	//TraceGraph graph = *graph_it;

	// keep a chain of active basic blocks
	// at first, the active blocks are the roots (which start execution at cycle 0)
	std::vector<TraceGraph_vertex_descriptor> antichain = rootVertices;

	// keep track of timestamp
	for (int timestamp = 0; timestamp < lastCycle; timestamp++) {
		*outputLog << "Examine Cycle: " << timestamp << "\n";
		std::cerr << "Examine Cycle: " << timestamp << "\n";
		// activeBBs keeps track of the number of a particular 
		// basic block resource that is needed to execute all
		// the basic blocks within the anti-chain for each given
		// cycle
		// it stores a pair of basic block ptr and an int representing
		// the number of that basic block needed
		std::map<BasicBlock *, int> activeBBs;
		activeBBs.clear();

		*outputLog << "anti-chain in cycle " << timestamp << ":\n";
		// look at all active basic blocks and annotate the IR
		// annotate annotate annotate
		for (auto it = antichain.begin(); it != antichain.end(); it++) {
			BasicBlock *BB = (*graph)[*it].basicblock;
			auto search = activeBBs.find(BB);
			if (search != activeBBs.end()) {
				// BB exists in activeBBs, increment count
				search->second++;
			} else {
				// else add it to activeBBs list
				activeBBs.insert(std::make_pair(BB, 1));
			}
			*outputLog << BB->getName() << "\n";
		}

		*outputLog << "activeBBs:\n";
		// update the IR
		// will store the replication factor of each basic block as metadata
		// could not find a way to directly attach metadata to each basic block
		// will instead attach to the terminator instruction of each basic block
		// this will be an issue if the basic block is merged/split...
		for (auto it = activeBBs.begin(); it != activeBBs.end(); it++) {
			*outputLog << it->first->getName() << " repfactor " << it->second << "\n";
			Instruction *inst = dyn_cast<Instruction>(it->first->getTerminator());
			// look at pre-existing replication factor
			LLVMContext &C = inst->getContext();
			//MDNode *N = MDNode::get(C, MDString::get(C, "FPGA_ADVISOR_REPLICATION_FACTOR"));
			// inst->setMetadata(repFactorStr, N);
			int repFactor = 0; // zero indicates CPU execution

			std::string MDName = "FPGA_ADVISOR_REPLICATION_FACTOR_";
			MDName += inst->getParent()->getName().str();
			MDNode *M = inst->getMetadata(MDName);

			if (M && M->getOperand(0)) {
				std::string repFactorStr = cast<MDString>(M->getOperand(0))->getString().str();
				repFactor = stoi(repFactorStr);
			} // else metadata was not set, repFactor is 0

			repFactor = std::max(repFactor, it->second);
			std::string repFactorStr = std::to_string(repFactor);

			MDNode *N = MDNode::get(C, MDString::get(C, repFactorStr));

			inst->setMetadata(MDName, N);
		}

		*outputLog << ".\n";
		
		// retire blocks which end this cycle and add their children
		#if 0
		for (auto it = antichain.begin(); /*done in loop body*/; /*done in loop body*/) {
			*outputLog << "1!!!\n";
			if (it == antichain.end()) {
				break;
			}
			*outputLog << *it << " 2!!!\n";
			if ((*graph)[*it].cycEnd == timestamp) {
			*outputLog << "3!!!\n";
				TraceGraph_out_edge_iterator oi, oe;
				TraceGraph_vertex_descriptor removed = *it;
				auto remove = it;
				it = antichain.erase(remove);
			*outputLog << "4!!!\n";
				for (boost::tie(oi, oe) = boost::out_edges(removed, *graph); oi != oe; oi++) {
			*outputLog << "5!!!\n";
					antichain.push_back(boost::target(*oi, *graph));
				}
				continue;
			}
			*outputLog << "6!!!\n";
			it++;
		}
		#endif

		
		*outputLog << "antichain size: " << antichain.size() << "\n";
		std::vector<TraceGraph_vertex_descriptor> newantichain;
		newantichain.clear();
		for (auto it = antichain.begin(); it != antichain.end(); ) {
			*outputLog << *it << " s: " << (*graph)[*it].cycStart << " e: " << (*graph)[*it].cycEnd << "\n";
			if ((*graph)[*it].cycEnd == timestamp) {
				// keep track of the children to add
				TraceGraph_out_edge_iterator oi, oe;
				for (boost::tie(oi, oe) = boost::out_edges(*it, *graph); oi != oe; oi++) {
					// designate the latest finishing parent to add child to antichain
					if (latest_parent(oi, graph)) {
						*outputLog << "new elements to add " << boost::target(*oi, *graph);
						newantichain.push_back(boost::target(*oi, *graph));
					}
				}
				*outputLog << "erasing from antichain " << *it << "\n";
				it = antichain.erase(it);
			} else {
				it++;
			}
		}
		
		for (auto it = newantichain.begin(); it != newantichain.end(); it++) {
			*outputLog << "adding to antichain " << *it << "\n";
			antichain.push_back(*it);
		}

		*outputLog << "-\n";

	}
	return true;
}

// return true if this edge connects the latest finishing parent to the child
bool AdvisorAnalysis::latest_parent(TraceGraph_out_edge_iterator edge, TraceGraphList_iterator graph) {
	TraceGraph_vertex_descriptor thisParent = boost::source(*edge, *graph);
	TraceGraph_vertex_descriptor child = boost::target(*edge, *graph);
	TraceGraph_in_edge_iterator ii, ie;
	for (boost::tie(ii, ie) = boost::in_edges(child, *graph); ii != ie; ii++) {
		TraceGraph_vertex_descriptor otherParent = boost::source(*ii, *graph);
		if (otherParent == thisParent) {
			continue;
		}
		// designate to latest parent and also to parent whose vertex id is larger
		if ( (*graph)[thisParent].cycEnd < (*graph)[otherParent].cycEnd ) {
			return false;
		} else if ( ((*graph)[thisParent].cycEnd == (*graph)[otherParent].cycEnd) && thisParent < otherParent ) {
			return false;
		}
	}
	return true;
}


// Function: find_optimal_configuration_for_all_calls
// Performs the gradient descent method for function F until it
// arrives at a local minima for area-delay product
void AdvisorAnalysis::find_optimal_configuration_for_all_calls(Function *F) {
	*outputLog << __func__ << "\n";
	assert(executionGraph.find(F) != executionGraph.end());
	
	bool done = false;

	// the constraint we care about is area delay product
	// we want to find the (local) minimum by the gradient
	// descent method
	unsigned minAreaDelay = UINT_MAX;
	int areaDelay = INT_MAX;

	std::cerr << "Progress bar <";
	while (!done) {
		ConvergenceCounter++; // stats
		*outputLog << "while looping\n";
		BasicBlock *removeBB;
		removeBB = NULL;
		areaDelay = incremental_gradient_descent(F, removeBB);
		std::cerr << "="; // progress bar
		*outputLog << "Current basic block configuration.\n";
		print_basic_block_configuration(F);
		if (areaDelay < 0) {
			// there are no more basic blocks that can
			// be reduced in count
			done = true;
			continue;
		} else if ((unsigned) areaDelay < minAreaDelay) {
			minAreaDelay = (unsigned) areaDelay;
			*outputLog << "Incremental Gradient Descent - minimum area delay: " << minAreaDelay << "\n";
			continue;
		}
		std::cerr << ">\n"; // end progress bar
		std::cerr << "Took " << ConvergenceCounter << " steps for convergence.\n";
		// since this incremental gradient descent step caused
		// area delay product to increase, undo this move
		increment_basic_block_instance_count(removeBB);
		std::cerr << "goldfish\n";
		done = true;
	}
}


// Function: incremental_gradient_descent
// Performs one incremental step of gradient descent for function F
// i.e. it will reduce the # of basic block instances for each basic block
// whose instance is greater than 1 and chose the block which makes the
// least contribution to performance/area
// Always return the best area delay product after removal, if no removals exist
// i.e. all of program is on CPU, return -1
int AdvisorAnalysis::incremental_gradient_descent(Function *F, BasicBlock *&removeBB) {
	unsigned minAreaDelay = UINT_MAX;
	//BasicBlock *removeBB = NULL;
	removeBB = NULL;

	for (auto BB = F->begin(); BB != F->end(); BB++) {
		bool modify = false;
		if (decrement_basic_block_instance_count(BB)) {
			*outputLog << "Performing removal of basic block " << BB->getName() << "\n";
			//*outputLog << "decremented successfully. Do analysis.\n";
			// iterate through all calls to this function in the trace
			// keep a sum of how long the program takes to finish, will
			// be multiplied by area for an area delay product estimate
			unsigned latency = 0;
			for (TraceGraphList_iterator fIt = executionGraph[F].begin();
					fIt != executionGraph[F].end(); fIt++) {
				std::vector<TraceGraph_vertex_descriptor> rootVertices;
				rootVertices.clear();
				find_root_vertices(rootVertices, fIt);

				// need to update edge weights before scheduling
				update_transition_delay(fIt);
				
				latency += schedule_with_resource_constraints(rootVertices, fIt, F);
			}

			*outputLog << "New Latency: " << latency << "\n";

			// if the entire design executes on the CPU, we will use unit
			// area since no extra area is needed
			// otherwise each basic block resource is approximated by the
			// FunctionAreaEstimator class, the *area* incurred only applies
			// to designs that use limited resources on the FPGA or resources
			// which may lead to worse performance
			// i.e. if a basic block requires simple LUT logic to implement,
			// no extra cost will be incurred as we can assume these resources
			// are abundant...
			unsigned area = get_area_requirement(F);
			*outputLog << "New Area: " << area << "\n";
			unsigned areaDelay = latency * area;
			if (areaDelay < minAreaDelay) {
				// record the basic block whose removal leads
				// to minimizing area-delay product
				minAreaDelay = areaDelay;
				removeBB = BB;
				*outputLog << "New Minimum Area Delay Product: " << minAreaDelay << "\n";
			}

			// restore the basic block count after removal
			increment_basic_block_instance_count(BB);
		} // else continue
	}

	// make the best move
	if (removeBB == NULL) {
		return -1;
	} else {
		*outputLog << "Take incremental step - remove BB " << removeBB->getName() << "\n";
		decrement_basic_block_instance_count(removeBB);
		return minAreaDelay;
	}
}


// Function: schedule_with_resource_constraints
// Return: latency of execution of trace
// This function will use the execution trace graph generated previously 
// and the resource constraints embedded in the IR as metadata to determine
// the latency of the particular function call instance represented by this
// execution trace
unsigned AdvisorAnalysis::schedule_with_resource_constraints(std::vector<TraceGraph_vertex_descriptor> &roots, TraceGraphList_iterator graph_it, Function *F) {
	*outputLog << __func__ << "\n";

	TraceGraph graph = *graph_it;
	// perform the scheduling with resource considerations
	
	// use hash table to keep track of resources available
	// the key is the basicblock resource	
	// each key indexes into a vector of unsigned integers
	// the number of elements in the vector correspond to the
	// number of available resources of that basic block
	// the vector contains unsigned ints which represent the cycle
	// at which the resource next becomes available
	// the bool of the pair in the value is the CPU resource flag
	// if set to true, no additional hardware is required
	// however, a global value is used to keep track of the cpu
	// idleness
	std::map<BasicBlock *, std::pair<bool, std::vector<unsigned> > > resourceTable;
	resourceTable.clear();

	//===----------------------------------------------------===//
	// Populate resource table
	//===----------------------------------------------------===//
	initialize_resource_table(F, resourceTable);

	// reset the cpu free cycle global!
	cpuCycle = -1;

	int lastCycle = -1;

	//===----------------------------------------------------===//
	// Use breadth first search to perform the scheduling
	// with resource constraints
	//===----------------------------------------------------===//
	for (std::vector<TraceGraph_vertex_descriptor>::iterator rV = roots.begin();
			rV != roots.end(); rV++) {
		ConstrainedScheduleVisitor vis(graph, *LT/*latency of BBs*/, lastCycle, cpuCycle, resourceTable);
		boost::breadth_first_search(graph, vertex(0, graph), boost::visitor(vis).root_vertex(*rV));
		//boost::depth_first_search(graph, boost::visitor(vis).root_vertex(*rV));
	}

	return lastCycle;
}


// Function: find_root_vertices
// Finds all vertices with in degree 0 -- root of subgraph/tree
void AdvisorAnalysis::find_root_vertices(std::vector<TraceGraph_vertex_descriptor> &roots, TraceGraphList_iterator graph_it) {
	TraceGraph graph = *graph_it;
	TraceGraph_iterator vi, ve;
	for (boost::tie(vi, ve) = vertices(graph); vi != ve; vi++) {
		if (boost::in_degree(*vi, graph) == 0) {
			roots.push_back(*vi);
		}
	}
}

// Function: set_basic_block_instance_count
// Set the basic block metadata to denote the number of basic block instances
// needed
void AdvisorAnalysis::set_basic_block_instance_count(BasicBlock *BB, int value) {
	std::string MDName = "FPGA_ADVISOR_REPLICATION_FACTOR_";
	MDName += BB->getName().str();
	LLVMContext &C = BB->getContext();
	MDNode *N = MDNode::get(C, MDString::get(C, std::to_string(value)));
	BB->getTerminator()->setMetadata(MDName, N);
}

// Function: decrement_basic_block_instance_count
// Return: false if decrement not successful
// Modify basic block metadata to denote the number of basic block instances
// needed
bool AdvisorAnalysis::decrement_basic_block_instance_count(BasicBlock *BB) {
	int repFactor = get_basic_block_instance_count(BB);
	if (repFactor <= 0) { // 0 represents CPU execution, anything above 0 means HW accel
		return false;
	}

	/*
	std::string MDName = "FPGA_ADVISOR_REPLICATION_FACTOR_";
	MDName += BB->getName().str();
	LLVMContext &C = BB->getContext();
	MDNode *N = MDNode::get(C, MDString::get(C, std::to_string(repFactor-1)));
	BB->getTerminator()->setMetadata(MDName, N);
	*/
	set_basic_block_instance_count(BB, repFactor - 1);
	return true;
}

// Function: increment_basic_block_instance_count
// Return: false if decrement not successful
// Modify basic block metadata to denote the number of basic block instances
// needed
bool AdvisorAnalysis::increment_basic_block_instance_count(BasicBlock *BB) {
	int repFactor = get_basic_block_instance_count(BB);

	/*
	std::string MDName = "FPGA_ADVISOR_REPLICATION_FACTOR_";
	MDName += BB->getName().str();
	LLVMContext &C = BB->getContext();
	MDNode *N = MDNode::get(C, MDString::get(C, std::to_string(repFactor+1)));
	BB->getTerminator()->setMetadata(MDName, N);
	*/
	set_basic_block_instance_count(BB, repFactor + 1);

	return true;
}

// Function: get_basic_block_instance_count
// Return: the number of instances of this basic block from metadata
int AdvisorAnalysis::get_basic_block_instance_count(BasicBlock *BB) {
	assert(BB);
	std::string MDName = "FPGA_ADVISOR_REPLICATION_FACTOR_";
	MDName += BB->getName().str();
	MDNode *M = BB->getTerminator()->getMetadata(MDName);

	int repFactor = -1;

	assert(M);
	assert(M->getOperand(0));

	std::string repFactorStr = cast<MDString>(M->getOperand(0))->getString().str();
	repFactor = stoi(repFactorStr);

	//*outputLog << __func__ << " metadata: ";
	//BB->getTerminator()->print(*outputLog);
	//*outputLog << " replication factor: " << repFactor << "\n";

	return repFactor;
}


// Function: initialize_resource_table
// The resource table represents the resources needed for this program
// the resources we need to consider are:
// HW logic: represented by individual basic blocks
// CPU: represented by a flag
// other??
// FIXME: integrate the cpu
void AdvisorAnalysis::initialize_resource_table(Function *F, std::map<BasicBlock *, std::pair<bool, std::vector<unsigned> > > &resourceTable) {
	for (auto BB = F->begin(); BB != F->end(); BB++) {
		int repFactor = get_basic_block_instance_count(BB);
		if (repFactor < 0) {
			continue;
		}

		if (repFactor == 0) {
			// cpu
			std::vector<unsigned> resourceVector(0);
			resourceTable.insert(std::make_pair(BB, std::make_pair(true, resourceVector)));
			*outputLog << "Created entry in resource table for basic block: " << BB->getName()
						<< " using cpu resources.\n";
		} else {
			// fpga
			// create a vector
			std::vector<unsigned> resourceVector(repFactor, 0);
			resourceTable.insert(std::make_pair(BB, std::make_pair(false, resourceVector)));

			*outputLog << "Created entry in resource table for basic block: " << BB->getName()
						<< " with " << repFactor << " entries.\n";
		}
	}
}


// Function: get_area_requirement
// Return: a unitless value representing the area 'cost' of a design
// I'm sure this will need a lot of calibration...
unsigned AdvisorAnalysis::get_area_requirement(Function *F) {
	// baseline area required for cpu
	int area = 1000;
	for (auto BB = F->begin(); BB != F->end(); BB++) {
		int areaBB = FunctionAreaEstimator::get_basic_block_area(*AT, BB);
		int repFactor = get_basic_block_instance_count(BB);
		area += areaBB * repFactor;
	}
	return area;
}


// Function: update_transition_delay
// updates the trace execution graph edge weights
void AdvisorAnalysis::update_transition_delay(TraceGraphList_iterator graph) {
	TraceGraph_edge_iterator ei, ee;
	for (boost::tie(ei, ee) = edges(*graph); ei != ee; ei++) {
		TraceGraph_vertex_descriptor s = boost::source(*ei, *graph);
		TraceGraph_vertex_descriptor t = boost::target(*ei, *graph);
		bool sHwExec = (0 < get_basic_block_instance_count((*graph)[s].basicblock));
		bool tHwExec = (0 < get_basic_block_instance_count((*graph)[t].basicblock));
		// add edge weight <=> transition delay when crossing a hw/cpu boundary
		unsigned delay = 0;
		if (sHwExec ^ tHwExec) {
			bool CPUToHW = true;
			if (sHwExec == true) {
				// fpga -> cpu
				CPUToHW = false;
			}
			delay = get_transition_delay((*graph)[s].basicblock, (*graph)[t].basicblock, CPUToHW);
		} else {
			// should have no transition penalty, double make sure
			delay = 0;
		}
		boost::put(boost::edge_weight_t(), *graph, *ei, delay);
	}
}


// Function: get_transition_delay
// Return: an unsigned int representing the transitional delay between switching from either
// fpga to cpu, or cpu to fpga
unsigned AdvisorAnalysis::get_transition_delay(BasicBlock *source, BasicBlock *target, bool CPUToHW) {
	unsigned delay = 100; // some baseline delay
	// need to do something here...
	// the delay shouldn't be constant?
	return delay;
}



void AdvisorAnalysis::print_basic_block_configuration(Function *F) {
	*outputLog << "Basic Block Configuration:\n";
	for (auto BB = F->begin(); BB != F->end(); BB++) {
		int repFactor = get_basic_block_instance_count(BB);
		*outputLog << BB->getName() << "\t[" << repFactor << "]\n";
	}
}


void AdvisorAnalysis::print_optimal_configuration_for_all_calls(Function *F) {
	int callNum = 0;
	for (TraceGraphList_iterator fIt = executionGraph[F].begin();
			fIt != executionGraph[F].end(); fIt++) {

		callNum++;
		std::string outfileName(F->getName().str() + "." + std::to_string(callNum) + ".final.dot");
		TraceGraphVertexWriter<TraceGraph> vpw(*fIt);
		TraceGraphEdgeWriter<TraceGraph> epw(*fIt);
		std::ofstream outfile(outfileName);
		boost::write_graphviz(outfile, *fIt, vpw, epw);
	}
}




// Function: modify_resource_requirement
// This function will use the gradient descent method to reduce the resource requirements
// for the program
void AdvisorAnalysis::modify_resource_requirement(Function *F, TraceGraphList_iterator graph_it) {
	// add code here...
}





char AdvisorAnalysis::ID = 0;
static RegisterPass<AdvisorAnalysis> X("fpga-advisor-analysis", "FPGA-Advisor Analysis Pass -- to be executed after instrumentation and program run", false, false);

char FunctionScheduler::ID = 0;
static RegisterPass<FunctionScheduler> Z("func-scheduler", "FPGA-Advisor Analysis Function Scheduler Pass", false, false);

char FunctionAreaEstimator::ID = 0;
static RegisterPass<FunctionAreaEstimator> Y("func-area-estimator", "FPGA-Advisor Analysis Function Area Estimator Pass", false, false);
