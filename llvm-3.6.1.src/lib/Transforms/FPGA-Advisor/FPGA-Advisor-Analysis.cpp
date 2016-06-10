//===- FPGA-Advisor-Analysis.cpp ---------------------------------------------------===//
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
#include "stack_trace.h"

#include <fstream>
#include <fstream>
#include <regex>
#include <time.h>
#include <exception>

//#define DEBUG_TYPE "fpga-advisor-analysis"
#define DEBUG_TYPE "fpga-advisor"

using namespace llvm;
using namespace fpga;
using std::ifstream;


//===----------------------------------------------------------------------===//
// Having some fun with colors
//===----------------------------------------------------------------------===//
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */


//===----------------------------------------------------------------------===//
// Some globals ... is that bad? :/
// I can move this into the class and clean up every time a different function
// is being analyzed. The problem is that I have written a module pass.
//===----------------------------------------------------------------------===//

std::error_code AEC;
MemoryDependenceAnalysis *MDA;
DominatorTree *DT;
DepGraph *depGraph;
// latency tables
std::map<BasicBlock *, LatencyStruct> *LT; // filled in by FunctionScheduler - simple visitation of instructions
//std::map<BasicBlock *, int> *LTCPU; // filled in after getting dynamic trace
// area table
std::map<BasicBlock *, int> *AT;
int cpuCycle;
std::vector<unsigned long long> startTime;

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
static cl::opt<bool> StaticDepsOnly("static-deps-only", 
		cl::desc("If enabled, program is analyzed only with dependence information that is statically avaiable"), 
		cl::Hidden, cl::init(false));
static cl::opt<unsigned int> TraceThreshold("trace-threshold", cl::desc("Maximum lines of input trace to read"),
		cl::Hidden, cl::init(UINT_MAX)
);
static cl::opt<unsigned int> AreaConstraint("area-constraint", cl::desc("Set the area constraint"),
		cl::Hidden, cl::init(0));
static cl::opt<unsigned> UserTransitionDelay("transition-delay", cl::desc("Set the fpga to cpu transition delay baseline"),
		cl::Hidden, cl::init(0));

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
	// log file
	raw_fd_ostream OL("fpga-advisor-analysis.log", AEC, sys::fs::F_RW);
	outputLog = &OL;
	if (NoMessage) {
		outputLog = &nulls();
	} else {
		DEBUG(outputLog = &dbgs());
	}
	*outputLog << "FPGA-Advisor Analysis Pass Starting.\n";

	// output results
	raw_fd_ostream OF("fpga-advisor-analysis-result.log", AEC, sys::fs::F_RW);
	outputFile = &OF;

	mod = &M;


	//=------------------------------------------------------=//
	// [2] Static analyses and setup
	//=------------------------------------------------------=//
	callGraph = &getAnalysis<CallGraphWrapperPass>().getCallGraph();
	find_recursive_functions(M);

	// basic statistics gathering
	// also populates the functionMap
	// disable this statistic for now
	//visit(M);

	*outputLog << "Finished visit.\n";
	
	//=------------------------------------------------------=//
	// [3] Read trace from file into memory
	//=------------------------------------------------------=//
	if (! get_program_trace(TraceFileName)) {
		errs() << "Could not process trace file: " << TraceFileName << "!\n";
		return false;
	}

	*outputLog << "Finished importing program trace.\n";

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
	// [5] Printout statistics [turned off, this isn't even useful]
	//=------------------------------------------------------=//
	//*outputLog << "Print static information\n"; // there is no need to announce this..
	// pre-instrumentation statistics => work with uninstrumented code
	//print_statistics();

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
		for (LoopInfo::reverse_iterator li = newFuncInfo->loopInfo->rbegin(),
										le = newFuncInfo->loopInfo->rend(); li != le; li++) {
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
        DEBUG(*outputLog << __func__ << "\n");
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
                  DEBUG(*outputLog << "Calling does_function_recurse on function: " << F->getName() << "\n");
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
        DEBUG(*outputLog << "does_function_recurse: " << CGN->getFunction()->getName() << "\n");
        DEBUG(*outputLog << "stack size: " << stack.size() << "\n");
	// if this function exists within the stack, function recurses and add to list
	if ((stack.size() > 0) && (std::find(stack.begin(), stack.end(), CGN->getFunction()) != stack.end())) {
          DEBUG(*outputLog << "Function recurses: " << CGN->getFunction()->getName() << "\n");
		
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
		DEBUG(*outputLog << "Found a call to function: " << calledGraphNode->getFunction()->getName() << "\n");
		//stack.push_back(calledGraphNode->getFunction());
		// ignore this function if its primary definition is outside current module
		if (! calledGraphNode->getFunction()->isDeclaration()) {
			does_function_recurse(func, calledGraphNode, stack);
		} else { // print a warning
			errs() << __func__ << " is being ignored, it is declared outside of this translational unit.\n";
		}
		DEBUG(*outputLog << "Returned from call to function: " << calledGraphNode->getFunction()->getName() << "\n");
	}
	// pop off the stack
	stack.pop_back();
	DEBUG(*outputLog << "stack size: " << stack.size() << "\n");
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
	unsigned cpuOnlyLatency = UINT_MAX;
	unsigned fpgaOnlyLatency = UINT_MAX;
	unsigned fpgaOnlyArea = 0;

	*outputLog << "Examine function: " << F->getName() << "\n";
	// Find constructs that are not supported by HLS
	if (has_unsynthesizable_construct(F)) {
		*outputLog << "Function contains unsynthesizable constructs, moving on.\n";
		return false;
	}

	// was this function even executed in run
	if (executionGraph.find(F) == executionGraph.end()) {
		*outputLog << "Did not find execution of function in program trace. Skipping.\n";
		return false;
	}
	
	// make sure execution was recorded in execution order
	if (executionOrderListMap.find(F) == executionOrderListMap.end()) {
		*outputLog << "Did not find execution of function in execution order. Error.\n";
		assert(0);
	}

	LT = &getAnalysis<FunctionScheduler>(*F).getFPGALatencyTable();
	AT = &getAnalysis<FunctionAreaEstimator>(*F).getAreaTable();
	// fill in latency table for cpu by traversing execution graph
	getCPULatencyTable(F, LT, executionOrderListMap[F], executionGraph[F]);

	// get the dependence graph for the function
	//depGraph = &getAnalysis<DependenceGraph>(*F).getDepGraph();
	std::string dgFileName = "dg." + F->getName().str() + ".log";
	if (!get_dependence_graph_from_file(dgFileName, &depGraph, F->getName().str())) {
		std::cerr << "Could not get the dependence graph! Error opening file " << dgFileName << "\n";
		assert(0);
	}

	// for each execution of the function found in the trace
	// we want to find the optimal tiling for the basicblocks
	// the starting point of the algorithm is the MOST parallel
	// configuration, which can be found by scheduling independent
	// sic blocks in the earliest cycle that it is allowed to be executed
	find_maximal_configuration_for_all_calls(F, fpgaOnlyLatency, fpgaOnlyArea);

	*outputLog << "Maximal basic block configuration for function: " << F->getName() << "\n";
	print_basic_block_configuration(F, outputLog);

	// print this to output file
	*outputFile << "Maximal basic block configuration for function: " << F->getName() << "\n";
	print_basic_block_configuration(F, outputFile);

	std::cerr << "Finished computing maximal configuration\n";

        // Now that we have a replication factor, we will prune it to honor the area
        // constraints of the device. 
        std::cerr << "Maximal basic blocks: " << get_total_basic_block_instances(F) << "\n";
        std::cerr << "Accelerator-only latency: " << fpgaOnlyLatency << "\n";
        *outputFile << "Maximal basic blocks: " << get_total_basic_block_instances(F) << "\n";
        prune_basic_block_configuration_to_device_area(F);
        std::cerr << "Pruned basic blocks: " << get_total_basic_block_instances(F) << "\n";
        *outputFile << "Pruned basic blocks: " << get_total_basic_block_instances(F) << "\n";
	// by this point, the basic blocks have been annotated by the maximal
	// legal replication factor
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
	find_optimal_configuration_for_all_calls(F, cpuOnlyLatency, fpgaOnlyLatency, fpgaOnlyArea);

	*outputLog << "===-------------------------------------===\n";
	*outputLog << "Final optimal basic block configuration for function: " << F->getName() << "\n";
	print_basic_block_configuration(F, outputLog);
	*outputLog << "===-------------------------------------===\n";

	// print this to output file
	*outputFile << "===-------------------------------------===\n";
	*outputFile << "Final optimal basic block configuration for function: " << F->getName() << "\n";
	print_basic_block_configuration(F, outputFile);
	*outputFile << "===-------------------------------------===\n";

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
		// return true;
		return false; // we ignore these for now!!?!? FIXME
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


/*#if 0
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

	//TraceGraph::vertex_descriptor prevVertex = 0;
	TraceGraph::vertex_descriptor latestVertex;
	//TraceGraphList_iterator latestGraph;
	Function *latestFunction = NULL;

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
			try {
				lineCopy.push_back(0);
			} catch (std::exception &e) {
				std::cerr << "An error occured." << e.what() << "\n";
			}

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
			latestFunction = F;
			if (!F) {
				// could not find the function by name
				errs() << "Could not find the function from trace in program!\n";
				return false;
			}
			
			//==----------------------------------------------------------------==//
			ExecGraph_iterator fGraph = executionGraph.find(F);
			ExecutionOrderListMap_iterator fOrder = executionOrderListMap.find(F);
			if (fGraph == executionGraph.end() && fOrder == executionOrderListMap.end()) {
				// function does not exist as entry in execGraph
				TraceGraphList emptyList;
				executionGraph.insert(std::make_pair(F, emptyList));
				TraceGraph newGraph;
				try {
					executionGraph[F].push_back(newGraph);
				} catch (std::exception &e) {
					std::cerr << "An error occured." << e.what() << "\n";
				}
				// set latest seen graph
				//latestGraph = (executionGraph[F].end()) - 1;

				ExecutionOrderList emptyOrderList;
				executionOrderListMap.insert(std::make_pair(F, emptyOrderList));
				ExecutionOrder newOrder;
				newOrder.clear();
				try {
					executionOrderListMap[F].push_back(newOrder);
				} catch (std::exception &e) {
					std::cerr << "An error occured." << e.what() << "\n";
				}
			} else if (fGraph != executionGraph.end() && fOrder != executionOrderListMap.end()) {
				// function exists
				TraceGraph newGraph;
				try {
					fGraph->second.push_back(newGraph);
				} catch (std::exception &e) {
					std::cerr << "An error occured." << e.what() << "\n";
				}

				// set latest seen graph
				//latestGraph = (fGraph->second.end()) - 1;

				ExecutionOrder newOrder;
				newOrder.clear();
				try {
					fOrder->second.push_back(newOrder);
				} catch (std::exception &e) {
					std::cerr << "An error occured." << e.what() << "\n";
				}
			} else {
				assert(0);
			}
			//==----------------------------------------------------------------==//
		} else if (std::regex_match(line, std::regex("(BasicBlock: )(.*)( Function: )(.*)"))) {
			// record this information
			const char *delimiter = " ";

			// make a non-const copy of line
			std::vector<char> lineCopy(line.begin(), line.end());
			try {
				lineCopy.push_back(0);
			} catch (std::exception &e) {
				std::cerr << "An error occured." << e.what() << "\n";
			}

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
			currGraph[currVertex].cpuCycles = 0;
			currGraph[currVertex].memoryWriteTuples.clear();
			currGraph[currVertex].memoryReadTuples.clear();

			//boost::write_graphviz(std::cerr, executionGraph[BB->getParent()].back());
			//==----------------------------------------------------------------==//

			// add to execution order
			// check if BB exists
			ExecutionOrder &currOrder = executionOrderListMap[BB->getParent()].back();
			ExecutionOrder_iterator search = currOrder.find(BB);
			if (search == currOrder.end()) {
				// insert BB into order
				std::vector<TraceGraph_vertex_descriptor> newVector;
				newVector.clear();
				try {
					newVector.push_back(currVertex);
					currOrder.insert(std::make_pair(BB, std::make_pair(-1, newVector)));
				} catch (std::exception &e) {
					std::cerr << "An error occured." << e.what() << "\n";
				}

			} else {
				// append to order
				try {
					search->second.second.push_back(currVertex);
				} catch (std::exception &e) {
					std::cerr << "An error occured." << e.what() << "\n";
				}
			}

			// increment the node ID
			ID++;

			// set the latest added vertex
			latestVertex = currVertex;

		} else if (std::regex_match(line, std::regex("(Store at address: )(.*)( size in bytes: )(.*)") )) {
			const char *delimiter = " ";

			// make a non-const copy of line
			std::vector<char> lineCopy(line.begin(), line.end());
			try {
				lineCopy.push_back(0);
			} catch (std::exception &e) {
				std::cerr << "An error occured." << e.what() << "\n";
			}

			//=---------------------------------=//
			// Store<space>at<space>address:<space>addr<space>size<space>in<space>bytes:<space>size\n
			// separate out string by tokens
			char *pch = std::strtok(&lineCopy[18], delimiter);
			std::string addrString(pch);

			pch = strtok(NULL, delimiter);
			// size
			pch = strtok(NULL, delimiter);
			// in
			pch = strtok(NULL, delimiter);
			// bytes:
			pch = strtok(NULL, delimiter);
			std::string bytesString(pch);
			//=---------------------------------=//

			// convert the string to uint64_t
			uint64_t addrStart = std::strtoul(addrString.c_str(), NULL, 0);
			uint64_t width = std::strtoul(bytesString.c_str(), NULL, 0);
			DEBUG(*outputLog << "Discovered a store with starting address : " << addrStart << "\n");
                        DEBUG(*outputLog << "Store width in bytes : " << width << "\n");

			TraceGraph &latestGraph = executionGraph[latestFunction].back();
			try {
				latestGraph[latestVertex].memoryWriteTuples.push_back(std::make_pair(addrStart, width));
			} catch (std::exception &e) {
				std::cerr << "An error occured." << e.what() << "\n";
			}

		} else if (std::regex_match(line, std::regex("(Load from address: )(.*)( size in bytes: )(.*)") )) {
			const char *delimiter = " ";

			// make a non-const copy of line
			std::vector<char> lineCopy(line.begin(), line.end());
			try {
				lineCopy.push_back(0);
			} catch (std::exception &e) {
				std::cerr << "An error occured." << e.what() << "\n";
			}

			//=---------------------------------=//
			// Load<space>from<space>address:<space>addr<space>size<space>in<space>bytes:<space>size\n
			// separate out string by tokens
			char *pch = std::strtok(&lineCopy[19], delimiter);
			std::string addrString(pch);

			pch = strtok(NULL, delimiter);
			// size
			pch = strtok(NULL, delimiter);
			// in
			pch = strtok(NULL, delimiter);
			// bytes:
			pch = strtok(NULL, delimiter);
			std::string byteString(pch);
			//=---------------------------------=//

			// convert the string to uint64_t
			uint64_t addrStart = std::strtoul(addrString.c_str(), NULL, 0);
			uint64_t width = std::strtoul(byteString.c_str(), NULL, 0);
			DEBUG(*outputLog << "Discovered a load with starting address : " << addrStart << "\n");
                        DEBUG(*outputLog << "Load width in bytes : " << width << "\n");

			TraceGraph &latestGraph = executionGraph[latestFunction].back();
			std::pair<uint64_t, uint64_t> addrWidthTuple = std::make_pair(addrStart, width);
			DEBUG(*outputLog << "after pair\n");
			try {
				//latestGraph[latestVertex].memoryReadTuples.push_back(std::make_pair(addrStart, width));
                                DEBUG(*outputLog << "before push_back read tuples " << latestGraph[latestVertex].memoryReadTuples.size() << "\n");
				//latestGraph[latestVertex].memoryReadTuples.push_back(addrWidthTuple);
				latestGraph[latestVertex].memoryReadTuples.emplace_back(addrStart, width);
				DEBUG(*outputLog << "after push_back read tuples\n");
			} catch (std::exception &e) {
				std::cerr << "An error occured." << e.what() << "\n";
			}
                      
			DEBUG(*outputLog << "after load\n");

		} else if (std::regex_match(line, std::regex("(Return from: )(.*)"))) {
			// nothing to do really...
		} else {
			//errs() << "Unexpected trace input!\n" << line << "\n";
			//return false;
			// ignore, probably program output
			// I'm going to have to deal with the issue of if previous program
			// output doesn't append \n .. okay, deal with this later, should be easy
		}
	}
	
	// print some debug output

	return true;
}
#endif
*/

/*#if 0
// Function: get_program_trace
// Return: false if unsuccessful
// Reads input trace file, parses and stores trace into executionTrace map
// TODO: do not add to trace the basic blocks which have only a branch instruction
bool AdvisorAnalysis::get_program_trace(std::string fileIn) {
	// the instrumentation phase will instrument all functions as long as
	// they are not external to the module (this will include recursive functions)
	// when recording the trace, create the trace for each function encountered,
	// however, simply ignore them later

	// read file
	ifstream fin;
	fin.open(fileIn.c_str());
	if (!fin.good()) {
		return false; // file not found
	}

	std::string line;

	// unique ID for each basic block executed
	int ID = 0;

	// for keeping track of which function and execution graph to insert into
	TraceGraph_vertex_descriptor lastVertex;
	TraceGraphList_iterator latestTraceGraph;
	Function *latestFunction = NULL;
	ExecutionOrderList_iterator latestExecutionOrder;

	// use a stack to keep track of where we should return to
	std::stack<FunctionExecutionRecord> funcStack;

	bool showProgressBar = true;
	// get total line number from file using wc command
	FILE *in;
	char buf[256];
	
	unsigned int fileLineNum;
	unsigned int traceThreshold = TraceThreshold;

	std::string cmd = "wc " + fileIn;
	if (!(in = popen(cmd.c_str(), "r"))) {
		// if cannot execute command, don't show progress bar
		showProgressBar = false;
		fileLineNum = UINT_MAX;
	} else {
		assert(fgets(buf, sizeof(buf), in) != NULL);
		*outputLog << "WC " << buf << "\n";
		char *pch = std::strtok(&buf[0], " ");
		fileLineNum = atoi(pch);
		*outputLog << "Total lines from " << fileIn << ": " << fileLineNum << "\n";
		std::cerr << "Total lines " << fileLineNum << "\n";
	}

	std::cerr << "Processing program trace.\n";

	unsigned int lineNum = 0;
	unsigned int totalLineNum = std::min(traceThreshold, fileLineNum);

	unsigned int times = 0;
	while (std::getline(fin, line)) {
		if (lineNum > traceThreshold) {
			break;
		}

		if (showProgressBar) {
			// print a processing progress bar
			// 20 points, print progress every 5% processed
			unsigned int fivePercent = totalLineNum / 20;
			if ((lineNum % fivePercent) == 0) {
				std::cerr << BOLDGREEN << " [ " << 5 * times << "% ] " << RESET << lineNum << "/" << totalLineNum << "\n";
				times++;
			}
			std::cerr << RESET;
		}

		DEBUG(*outputLog << "PROCESSING LINE: " << line << " (" << lineNum << ")\n");
                lineNum++;
 		//*outputLog << "latestTraceGraph iterator: " << latestTraceGraph << "\n";
		DEBUG(*outputLog << "lastVertex: " << lastVertex << "\n");
		// There are 5 types of messages:
		// 1. Enter Function: <func name>
		// 2. Basic Block: <basic block name> Function: <func name>
		// 3. Return from: <func name>
		// 4. Store at address: <addr start> size in bytes: <size>
		// 5. Load from address: <addr start> size in bytes: <size>
		if (std::regex_match(line, std::regex("(Entering Function: )(.*)"))) {
			if (!process_function_entry(line, &latestFunction, latestTraceGraph, lastVertex, latestExecutionOrder, funcStack)) {
				*outputLog << "process function entry: FAILED.\n";
				return false;
			}
		} else if (std::regex_match(line, std::regex("(BasicBlock: )(.*)( Function: )(.*)"))) {
			if (!process_basic_block_entry(line, ID, latestTraceGraph, lastVertex, latestExecutionOrder)) {
				*outputLog << "process basic block entry: FAILED.\n";
				return false;
			}
		} else if (std::regex_match(line, std::regex("(Store at address: )(.*)( size in bytes: )(.*)") )) {
			if (!process_store(line, latestFunction, latestTraceGraph, lastVertex)) {
				*outputLog << "process store: FAILED.\n";
				return false;
			}
		} else if (std::regex_match(line, std::regex("(Load from address: )(.*)( size in bytes: )(.*)") )) {
			if (!process_load(line, latestFunction, latestTraceGraph, lastVertex)) {
				*outputLog << "process load: FAILED.\n";
				return false;
			}
		} else if (std::regex_match(line, std::regex("(BasicBlock Clock get time start: )(.*)"))) {
			if (!process_time(line, latestTraceGraph, lastVertex, true)) {
				*outputLog << "process time start: FAILED.\n";
				return false;
			}
		} else if (std::regex_match(line, std::regex("(BasicBlock Clock get time stop: )(.*)"))) {
			if (!process_time(line, latestTraceGraph, lastVertex, false)) {
				*outputLog << "process time stop: FAILED.\n";
				return false;
			}
		} else if (std::regex_match(line, std::regex("(Return from: )(.*)"))) {
			if (!process_function_return(line, &latestFunction, funcStack, latestTraceGraph, lastVertex, latestExecutionOrder)) {
				*outputLog << "process function return: FAILED.\n";
				return false;
			}
		} else {
			// ignore, probably program output
		}
	}
	return true;
}
#endif
*/

// Function: get_program_trace
// Return: false if unsuccessful
// Reads input trace file, parses and stores trace into executionTrace map
bool AdvisorAnalysis::get_program_trace(std::string fileIn) {
	// the instrumentation phase will instrument all functions as long as
	// they are not external to the module (this will include recursive functions)
	// when recording the trace, create the trace for each function encountered,
	// however, simply ignore them later

	// read file
	ifstream fin;
	fin.open(fileIn.c_str());
	if (!fin.good()) {
		return false; // file not found
	}

	std::string line;

	// unique ID for each basic block executed
	int ID = 0;

	// for keeping track of which function and execution graph to insert into
	TraceGraph_vertex_descriptor lastVertex;
	TraceGraphList_iterator latestTraceGraph;
	Function *latestFunction = NULL;
	ExecutionOrderList_iterator latestExecutionOrder;

	// use a stack to keep track of where we should return to
	std::stack<FunctionExecutionRecord> funcStack;

	bool showProgressBar = true;
	// get total line number from file using wc command
	FILE *in;
	char buf[256];
	
	unsigned int fileLineNum;
	unsigned int traceThreshold = TraceThreshold;

	std::string cmd = "wc " + fileIn;
	if (!(in = popen(cmd.c_str(), "r"))) {
		// if cannot execute command, don't show progress bar
		showProgressBar = false;
		fileLineNum = UINT_MAX;
	} else {
		assert(fgets(buf, sizeof(buf), in) != NULL);
		DEBUG(*outputLog << "WC " << buf << "\n");
		char *pch = std::strtok(&buf[0], " ");
		fileLineNum = atoi(pch);
		*outputLog << "Total lines from " << fileIn << ": " << fileLineNum << "\n";
		std::cerr << "Total lines " << fileLineNum << "\n";
	}

	std::cerr << "Processing program trace.\n";

	unsigned int lineNum = 0;
	unsigned int totalLineNum = std::min(traceThreshold, fileLineNum);

	unsigned int times = 0;
	while (std::getline(fin, line)) {
		if (lineNum > traceThreshold) {
			break;
		}

		if (showProgressBar) {
			// print a processing progress bar
			// 20 points, print progress every 5% processed
			unsigned int fivePercent = totalLineNum / 20;
			if ((lineNum % fivePercent) == 0) {
				std::cerr << BOLDGREEN << " [ " << 5 * times << "% ] " << RESET << lineNum << "/" << totalLineNum << "\n";
				times++;
			}
			std::cerr << RESET;
		}

		DEBUG(*outputLog << "PROCESSING LINE: " << line << " (" << lineNum << ")\n");
                lineNum++;
		//*outputLog << "latestTraceGraph iterator: " << latestTraceGraph << "\n";
                DEBUG(*outputLog << "lastVertex: " << lastVertex << "\n");
		// There are 5 types of messages:
		// 1. Enter Function: <func name>
		// 2. Basic Block: <basic block name> Function: <func name>
		// 3. Return from: <func name>
		// 4. Store at address: <addr start> size in bytes: <size>
		// 5. Load from address: <addr start> size in bytes: <size>
               if (std::regex_match(line, std::regex("(BasicBlock: )(.*)( Function: )(.*)"))) {
			if (!process_basic_block_entry(line, ID, latestTraceGraph, lastVertex, latestExecutionOrder)) {
				*outputLog << "process basic block entry: FAILED.\n";
				return false;
			}
		} else if (std::regex_match(line, std::regex("(BSTR: )(.*)"))) {
			if (!process_time(line, latestTraceGraph, lastVertex, true)) {
				*outputLog << "process time start: FAILED.\n";
				return false;
			}
		} else if (std::regex_match(line, std::regex("(BSTP: )(.*)"))) {
			if (!process_time(line, latestTraceGraph, lastVertex, false)) {
				*outputLog << "process time stop: FAILED.\n";
				return false;
			}
                } else if (std::regex_match(line, std::regex("(ST: )(.*)( B: )(.*)") )) {
			if (!process_store(line, latestFunction, latestTraceGraph, lastVertex)) {
				*outputLog << "process store: FAILED.\n";
				return false;
			}
		} else if (std::regex_match(line, std::regex("(LD: )(.*)( B: )(.*)") )) {
			if (!process_load(line, latestFunction, latestTraceGraph, lastVertex)) {
				*outputLog << "process load: FAILED.\n";
				return false;
			}
                } else if (std::regex_match(line, std::regex("(Entering Function: )(.*)"))) {
			if (!process_function_entry(line, &latestFunction, latestTraceGraph, lastVertex, latestExecutionOrder, funcStack)) {
				*outputLog << "process function entry: FAILED.\n";
				return false;
			}	        
		} else if (std::regex_match(line, std::regex("(Return from: )(.*)"))) {
			if (!process_function_return(line, &latestFunction, funcStack, latestTraceGraph, lastVertex, latestExecutionOrder)) {
				*outputLog << "process function return: FAILED.\n";
				return false;
			}
		} else {
			// ignore, probably program output
		}
	}
	return true;
}



// process one line of trace containing a time start or stop
bool AdvisorAnalysis::process_time(const std::string &line, TraceGraphList_iterator latestTraceGraph, TraceGraph_vertex_descriptor lastVertex, bool start) {
        DEBUG(*outputLog << __func__ << " " << line << "\n");
	const char *delimiter = " ";

	// get the time value
	// make a non-const copy of the line
	std::vector<char> lineCopy(line.begin(), line.end());
	lineCopy.push_back(0);

	// separate line by space to get keywords
	//=---------------------------------=//
	char *pch;
        pch = std::strtok(&lineCopy[6], delimiter);

	std::string cycleString(pch);
	//=---------------------------------=//

	// convert the string to long int
	unsigned long long cycle = std::strtol(cycleString.c_str(), NULL, 0);

	if (start) {
          DEBUG(*outputLog << "Start time : " << cycle << " cycles\n");
		// store the starts in stack, pop stack when stop is encountered
		assert(startTime.empty());
		startTime.push_back(cycle);
	} else {
                DEBUG(*outputLog << "Stop time : " << cycle << " cycles\n");
		// update the timer
		unsigned long long start = startTime.back();
		assert(startTime.size() == 1); // size must be one!!
		startTime.pop_back();
		
		// update the graph
		(*latestTraceGraph)[lastVertex].cpuCycles = (cycle - start);
	}

	DEBUG(*outputLog << (*latestTraceGraph)[lastVertex].name << "\n");

	return true;
}


// process one line of trace containing return
bool AdvisorAnalysis::process_function_return(const std::string &line, Function **function, std::stack<FunctionExecutionRecord> &stack, TraceGraphList_iterator &lastTraceGraph, TraceGraph_vertex_descriptor &lastVertex, ExecutionOrderList_iterator &lastExecutionOrder) {
        DEBUG(*outputLog << __func__ << " " << line << "\n");
	
	const char *delimiter = " ";

	// make non-const copy of line
	std::vector<char> lineCopy(line.begin(), line.end());
	lineCopy.push_back(0);

	//=---------------------------------=//
	// Return<space>from:<space>function
	char *pch = std::strtok(&lineCopy[13], delimiter);
	std::string funcString(pch);
	//=---------------------------------=//

	// make sure that this is the last function on stack
	Function *F = find_function_by_name(funcString);
	assert(F);


	// update current function after returning
	if (*function == NULL) {
		return false;
	} else if (*function != NULL && stack.size() == 0) {
		*function = NULL;
		return true;
	} else {
		*function = stack.top().function;
		lastTraceGraph = stack.top().graph;
		lastVertex = stack.top().vertex;
		lastExecutionOrder = stack.top().executionOrder;
		DEBUG(*outputLog << "<<<< Return to function " << (*function)->getName() << "\n");
	}

	if (stack.size() > 0) {
		stack.pop();
	}

	return true;
}


// process one line of trace containing load
bool AdvisorAnalysis::process_load(const std::string &line, Function *function, TraceGraphList_iterator lastTraceGraph, TraceGraph_vertex_descriptor lastVertex) {
        DEBUG(*outputLog << __func__ << " " << line << "\n");
	const char *delimiter = " ";

	// make a non-const copy of line
	std::vector<char> lineCopy(line.begin(), line.end());
	lineCopy.push_back(0);

	// separate line by space to get keywords
	//=---------------------------------=//
	// Load<space>from<space>address:<space>addr<space>size<space>in<space>bytes:<space>size\n
	// separate out string by tokens
	char *pch = std::strtok(&lineCopy[4], delimiter);
	std::string addrString(pch);
 
	pch = strtok(NULL, delimiter);
	// bytes:
	pch = strtok(NULL, delimiter);
	std::string byteString(pch);
	//=---------------------------------=//

	// convert the string to uint64_t
	uint64_t addrStart = std::strtoul(addrString.c_str(), NULL, 0);
	uint64_t width = std::strtoul(byteString.c_str(), NULL, 0);
	DEBUG(*outputLog << "Discovered a load with starting address : " << addrStart << "\n");
	DEBUG(*outputLog << "Load width in bytes : " << width << "\n");

	//TraceGraph &latestGraph = executionGraph[function].back();
	TraceGraph &latestGraph = *lastTraceGraph;
	//std::pair<uint64_t, uint64_t> addrWidthTuple = std::make_pair(addrStart, width);
	DEBUG(*outputLog << "after pair\n");
	try {
		//latestGraph[latestVertex].memoryReadTuples.push_back(std::make_pair(addrStart, width));
                DEBUG(*outputLog << "before push_back read tuples " << latestGraph[lastVertex].memoryReadTuples.size() << "\n");
		//latestGraph[latestVertex].memoryReadTuples.push_back(addrWidthTuple);
		latestGraph[lastVertex].memoryReadTuples.emplace_back(addrStart, width);
		DEBUG(*outputLog << "after push_back read tuples\n");
	} catch (std::exception &e) {
		std::cerr << "An error occured." << e.what() << "\n";
	}
	DEBUG(*outputLog << "after load\n");

	return true;
}


// process one line of trace containing store
bool AdvisorAnalysis::process_store(const std::string &line, Function *function, TraceGraphList_iterator lastTraceGraph, TraceGraph_vertex_descriptor lastVertex) {
        DEBUG(*outputLog << __func__ << " " << line << "\n");
	const char *delimiter = " ";

	// make a non-const copy of line
	std::vector<char> lineCopy(line.begin(), line.end());
	lineCopy.push_back(0);

	// separate line by space to get keywords
	//=---------------------------------=//
	// Store<space>at<space>address:<space>addr<space>size<space>in<space>bytes:<space>size\n
	// separate out string by tokens
	char *pch = std::strtok(&lineCopy[4], delimiter);
	std::string addrString(pch);

	// bytes:
	pch = strtok(NULL, delimiter);
	pch = strtok(NULL, delimiter);
	std::string bytesString(pch);
	//=---------------------------------=//

        
        //std::cerr << "Discovered a store with starting address : " << addrString << "\n";
        //std::cerr << "Store width in bytes : " << bytesString << "\n";

	// convert the string to uint64_t
	uint64_t addrStart = std::strtoul(addrString.c_str(), NULL, 0);
	uint64_t width = std::strtoul(bytesString.c_str(), NULL, 0);
	DEBUG(*outputLog << "Discovered a store with starting address : " << addrStart << "\n");
        DEBUG(*outputLog << "Store width in bytes : " << width << "\n");

	//TraceGraph &latestGraph = executionGraph[function].back();
	TraceGraph &latestGraph = *lastTraceGraph;
	try {
		latestGraph[lastVertex].memoryWriteTuples.push_back(std::make_pair(addrStart, width));
	} catch (std::exception &e) {
		std::cerr << "An error occured." << e.what() << "\n";
	}

	return true;
}



// process one line of trace containing basic block entry
bool AdvisorAnalysis::process_basic_block_entry(const std::string &line, int &ID, TraceGraphList_iterator lastTraceGraph, TraceGraph_vertex_descriptor &lastVertex, ExecutionOrderList_iterator lastExecutionOrder) {
        DEBUG(*outputLog << __func__ << " " << line << "\n");
	const char *delimiter = " ";

	// make a non-const copy of line
	std::vector<char> lineCopy(line.begin(), line.end());
	lineCopy.push_back(0);

	// separate line by space to get keywords
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
		// could not find the basic block by name
		errs() << "Could not find the basic block from trace in program! " << bbString << "\n";
		return false;
	}

	DEBUG(*outputLog << "SOMETHING\n");

	if (isa<TerminatorInst>(BB->getFirstNonPHI())) {
		// if the basic block only contains a branch/control flow and no computation
		// then skip it, do not add to graph
		// TODO if this is what I end up doing, need to remove looking at these
		// basic blocks when considering transitions ?? I think that already happens.
		return true;
	}

	DEBUG(*outputLog << "~~~~~~~~~\n");

	//==----------------------------------------------------------------==//
	//TraceGraph::vertex_descriptor currVertex = boost::add_vertex(executionGraph[BB->getParent()].back());
	//TraceGraph &currGraph = executionGraph[BB->getParent()].back();
	TraceGraph::vertex_descriptor currVertex = boost::add_vertex(*lastTraceGraph);
	TraceGraph &currGraph = *lastTraceGraph;
	currGraph[currVertex].basicblock = BB;
	currGraph[currVertex].ID = ID;
	currGraph[currVertex].minCycStart = -1;
	currGraph[currVertex].minCycEnd = -1;
	currGraph[currVertex].cpuCycles = 0;
	currGraph[currVertex].name = BB->getName().str();
	currGraph[currVertex].memoryWriteTuples.clear();
	currGraph[currVertex].memoryReadTuples.clear();
	//==----------------------------------------------------------------==//

	// add to execution order
	// check if BB exists
	//ExecutionOrder &currOrder = executionOrderListMap[BB->getParent()].back();
	ExecutionOrderList_iterator currOrder = lastExecutionOrder;
	ExecutionOrder_iterator search = currOrder->find(BB);
	if (search == currOrder->end()) {
		// insert BB into order
		std::vector<TraceGraph_vertex_descriptor> newVector;
		newVector.clear();
		try {
			newVector.push_back(currVertex);
			currOrder->insert(std::make_pair(BB, std::make_pair(-1, newVector)));
		} catch (std::exception &e) {
			std::cerr << "An error occured." << e.what() << "\n";
		}

	} else {
		// append to order
		try {
			search->second.second.push_back(currVertex);
		} catch (std::exception &e) {
			std::cerr << "An error occured." << e.what() << "\n";
		}
	}

	// increment the node ID
	ID++;

	// set the latest added vertex
	lastVertex = currVertex;

	DEBUG(*outputLog << "lululululu\n");
        DEBUG(*outputLog << (*lastTraceGraph)[lastVertex].name << "\n");
        DEBUG(*outputLog << "huhuhuhuhu\n");

	return true;
}


// processes one line of input from trace of entering a function
bool AdvisorAnalysis::process_function_entry(const std::string &line, Function **function, TraceGraphList_iterator &latestTraceGraph, TraceGraph_vertex_descriptor &latestVertex, ExecutionOrderList_iterator &latestExecutionOrder, std::stack<FunctionExecutionRecord> &stack) {
        DEBUG(*outputLog << __func__ << " " << line << "\n");
	const char *delimiter = " ";

	// append to stack when entering a function from another calling function
	if (*function != NULL) {
		// keep track of caller
		FunctionExecutionRecord newRecord;
		newRecord.function = *function;
		newRecord.graph = latestTraceGraph;
		newRecord.vertex = latestVertex;
		newRecord.executionOrder = latestExecutionOrder;
		stack.emplace(newRecord);
	}

	// make a non-const copy of line
	std::vector<char> lineCopy(line.begin(), line.end());
	lineCopy.push_back(0);

	// separate by space to get keywords
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
		// could not find function by name
		errs() << "Could not find the function from trace in program! " << funcString << "\n";
		return false;
	}
	*function = F;

	// append to stack when entering function
	//stack.push(F);
	
	// add to execution graph
	//==----------------------------------------------------------------==//
	ExecGraph_iterator fGraph = executionGraph.find(F);
	ExecutionOrderListMap_iterator fOrder = executionOrderListMap.find(F);
	if (fGraph == executionGraph.end() && fOrder == executionOrderListMap.end()) {
		// function does not exist as entry in execGraph
		TraceGraphList emptyList;
		executionGraph.insert(std::make_pair(F, emptyList));
		TraceGraph newGraph;
		try {
			executionGraph[F].push_back(newGraph);
			DEBUG(*outputLog << __func__ << " size of list: " << executionGraph[F].size() << "\n");
			// update the latest trace graph created
			latestTraceGraph = executionGraph[F].end();
			latestTraceGraph--; // go to the last element in list
			assert(latestTraceGraph == executionGraph[F].begin());
		} catch (std::exception &e) {
			std::cerr << "An error occured." << e.what() << "\n";
		}

		ExecutionOrderList emptyOrderList;
		executionOrderListMap.insert(std::make_pair(F, emptyOrderList));
		ExecutionOrder newOrder;
		newOrder.clear();
		try {
			executionOrderListMap[F].push_back(newOrder);
			latestExecutionOrder = executionOrderListMap[F].end();
			latestExecutionOrder--;
			DEBUG(*outputLog << "11111\n");
                        DEBUG(*outputLog << latestExecutionOrder->size() << "\n");
		} catch (std::exception &e) {
			std::cerr << "An error occured." << e.what() << "\n";
		}
	} else if (fGraph != executionGraph.end() && fOrder != executionOrderListMap.end()) {
		// function exists
		TraceGraph newGraph;
		try {
			fGraph->second.push_back(newGraph);
			// update the latest trace graph created
			latestTraceGraph = fGraph->second.end();
			latestTraceGraph--; // go to the last element in list
		} catch (std::exception &e) {
			std::cerr << "An error occured." << e.what() << "\n";
		}

		ExecutionOrder newOrder;
		newOrder.clear();
		try {
			fOrder->second.push_back(newOrder);
			latestExecutionOrder = fOrder->second.end();
			latestExecutionOrder--;
		} catch (std::exception &e) {
			std::cerr << "An error occured." << e.what() << "\n";
		}
	} else {
		assert(0);
	}
	//==----------------------------------------------------------------==//
	return true;
}


void AdvisorAnalysis::getCPULatencyTable(Function *F, std::map<BasicBlock *, LatencyStruct> *LT, ExecutionOrderList &executionOrderList, TraceGraphList &executionGraphList) {
	*outputLog << __func__ << " for function: " << F->getName() << "\n";
	// traverse through each execution order
	ExecutionOrderList_iterator eol;
	TraceGraphList_iterator tgl;
	/*
	for (eol = executionOrderList.begin(), tgl = executionGraphList.begin();
			eol != executionOrderList.end() && tgl != executionOrderList.end(); eol++, tgl++) {
		// traverse through each basic block entry in the current execution order
		ExecutionOrder_iterator eo;
		for (eo = eol->begin(); eo != eol->end(); eo++) {
			BasicBlock *BB = eo->first;
			// look at each instance of execution of this basic block
			for (auto inst = eo->second.begin(); inst != eo->second.end(); inst++) {
				
			}
		}
	}
	*/

	// compute for each basic block across each execution
	for (auto BB = F->begin(); BB != F->end(); BB++) {
		int iterCount = 0;
		float avgLatency = 0;
		for (eol = (executionOrderList).begin(), tgl = (executionGraphList).begin();
			eol != (executionOrderList).end() && tgl != (executionGraphList).end(); eol++, tgl++) {
			auto search = (*eol).find(BB);
			if (search == (*eol).end()) {
				// this basic block did not execute in this function call
				continue;
			}
			// average of all cpu executions of this basic block
			for (unsigned int i = 0; i < search->second.second.size(); i++) {
				int newElem = (*tgl)[ search->second.second[i] ].cpuCycles;
				avgLatency = ((avgLatency * (float)iterCount) + (float)newElem)/(float)(iterCount + 1);
				iterCount++;
			}
		}
		// insert the entry

		// if basic block didn't exist, latency is just 0 ??
		// truncate to int
		int latency = (int) avgLatency;
		if (latency == 0) {
			latency++; // must be due to truncation
		}
		
		*outputLog << "Average Latency for basic block: " << BB->getName() << " " << latency << "\n";

                // std::cerr << "Average Latency for basic block: " << BB->getName() << " " << latency << "\n";

                auto search = LT->find(BB);
		assert(search != LT->end());

		search->second.cpuLatency = latency;

	}

	*outputLog << "done\n";
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
bool AdvisorAnalysis::find_maximal_configuration_for_all_calls(Function *F, unsigned &fpgaOnlyLatency, unsigned &fpgaOnlyArea) {
	*outputLog << __func__ << " for function " << F->getName() << "\n";;
	//assert(executionTrace.find(F) != executionTrace.end());
	assert(executionGraph.find(F) != executionGraph.end());
	assert(executionOrderListMap.find(F) != executionOrderListMap.end());
	bool scheduled = false;

	int unconstrainedLastCycle = -1;

	initialize_basic_block_instance_count(F);

	// The ending condition should be determined by the user input of acceptable
	// area and latency constraints
	//while (1) {
		// iterate over all calls
		*outputLog << "There are " << executionGraph[F].size() << " calls to " << F->getName() << "\n";
		TraceGraphList_iterator fIt;
		ExecutionOrderList_iterator eoIt;
		for (fIt = executionGraph[F].begin(),
				eoIt = executionOrderListMap[F].begin(); 
				fIt != executionGraph[F].end(),
				eoIt != executionOrderListMap[F].end(); fIt++, eoIt++) {
			std::vector<TraceGraph_vertex_descriptor> rootVertices;
			rootVertices.clear();
			scheduled |= find_maximal_configuration_for_call(F, fIt, eoIt, rootVertices);
			//scheduled |= find_maximal_configuration_for_call(F, fIt, rootVertices);
			// after creating trace graphs representing maximal parallelism
			// compute maximal tiling
			//find_maximal_tiling_for_call(F, fIt);

			// find root vertices
			find_root_vertices(rootVertices, fIt);

			TraceGraph graph = *fIt;
			*outputLog << "root vertices are: ";
			for (auto rV = rootVertices.begin(); rV != rootVertices.end(); rV++) {
				*outputLog << "root: [" << *rV << "]->" << graph[*rV].name << "\n";
			}
	
			int lastCycle = -1;
	
			// annotate each node with the start and end cycles
			scheduled |= annotate_schedule_for_call(F, fIt, rootVertices, lastCycle);
	
			DEBUG(*outputLog << "Last Cycle: " << lastCycle << "\n");

			// after creating trace graphs, find maximal resources needed
			// to satisfy longest antichain
			scheduled |= find_maximal_resource_requirement(F, fIt, rootVertices, lastCycle);
	
			// use gradient descent method
			//modify_resource_requirement(F, fIt);

			unconstrainedLastCycle = lastCycle;

		}
	//}


	// keep this value for determining when to stop pursuing fpga accelerator implementation
	fpgaOnlyLatency = unconstrainedLastCycle;
	fpgaOnlyArea = get_area_requirement(F);

	*outputFile << "Unconstrained schedule: " << unconstrainedLastCycle << "\n";
	*outputFile << "Area requirement: " << fpgaOnlyArea << "\n";

	return scheduled;
}

bool AdvisorAnalysis::find_maximal_configuration_for_call(Function *F, TraceGraphList_iterator graph,
		ExecutionOrderList_iterator execOrder, std::vector<TraceGraph_vertex_descriptor> &rootVertices) {
	*outputLog << __func__ << " for function " << F->getName() << "\n";
	//std::cerr << __func__ << " for function " << F->getName().str() << "\n";

	print_execution_order(execOrder);

	unsigned int totalNumVertices = boost::num_vertices(*graph);
	TraceGraph_iterator vi, ve;
	for (boost::tie(vi, ve) = boost::vertices(*graph); vi != ve; vi++) {
		TraceGraph_vertex_descriptor self = *vi;
		BasicBlock *selfBB = (*graph)[self].basicblock;
		DEBUG(*outputLog << "Inspecting vertex (" << self << "/" << totalNumVertices << ") " << selfBB->getName() << "\n");

		// staticDeps vector keeps track of basic blocks that this basic block is 
		// dependent on
		std::vector<BasicBlock *> staticDeps;
		staticDeps.clear();
		DependenceGraph::get_all_basic_block_dependencies(*depGraph, selfBB, staticDeps);

		// print out the static deps
		DEBUG(*outputLog << "Found number of static dependences: " << staticDeps.size() << "\n");
		for (auto sdi = staticDeps.begin(); sdi != staticDeps.end(); sdi++) {
                  DEBUG(*outputLog << "\tStatic dependence with: " << (*sdi)->getName() << "\n");
		}

		// dynamicDeps vector keeps track of vertices in dynamic execution trace
		std::vector<TraceGraph_vertex_descriptor> dynamicDeps;
		dynamicDeps.clear();

		// fill the dynamicDeps vector by finding the most recent past execution of the
		// dependent basic blocks in the dynamic trace
		for (auto sIt = staticDeps.begin(); sIt != staticDeps.end(); sIt++) {
			BasicBlock *depBB = *sIt;
			// find corresponding execution order vector
			auto search = (*execOrder).find(depBB);
			if (search == (*execOrder).end()) {
				// static dependence on basic block which was not executed in run
				continue;
			}
			//assert(search != (*execOrder).end());

			int currExec = search->second.first;
			std::vector<TraceGraph_vertex_descriptor> &execOrderVec = search->second.second;
			assert((int) currExec <= (int) execOrderVec.size());

			if (currExec < 0) {
                          DEBUG(*outputLog << "Dependent basic block hasn't been executed yet. " << depBB->getName() << "\n");
				// don't append dynamic dependence
			} else {
				// the dependent basic block has been executed before this basic block, so possibly
				// need to add a dependence edge
				TraceGraph_vertex_descriptor dynDep = execOrderVec[currExec];

				if (!StaticDepsOnly) {
					bool dynamicDepExists = dynamic_memory_dependence_exists(self, dynDep, graph);
					bool trueDepExists = DependenceGraph::is_basic_block_dependence_true((*graph)[self].basicblock, (*graph)[dynDep].basicblock, *depGraph);
					DEBUG(*outputLog << "dynamicDepExists: " << dynamicDepExists << "\n");
                                        DEBUG(*outputLog << "trueDepExists: " << trueDepExists << "\n");
					if (!dynamicDepExists && !trueDepExists) {
						// don't add edge to node for which there are no true dependences
						// nor any dynamic memory dependences
                                                DEBUG(*outputLog << "Dynamic execution determined no true or memory dependences between ");
                                                DEBUG(*outputLog << (*graph)[self].name << " (" << self << ") and " << (*graph)[dynDep].name << " (" << dynDep << ")\n");
						continue;
					}
				}

				dynamicDeps.push_back(dynDep);
			}
		}

		DEBUG(*outputLog << "Found number of dynamic dependences (before): " << dynamicDeps.size() << "\n");

		// remove redundant dynamic dependence entries
		// these are the dynamic dependences which another dynamic dependence is directly
		// or indirectly dependent on
		
		//===-----------------------------------------------------------------------===//
		//
		// I thought removal of these redundant dependencies would help performance
		// it *significantly* slowed down my analysis.. I am removing it for now.
		//
		//===-----------------------------------------------------------------------===//
		//remove_redundant_dynamic_dependencies(graph, dynamicDeps);

		DEBUG(*outputLog << "Found number of dynamic dependences (after): " << dynamicDeps.size() << "\n");
		
		// add dependency edges to graph
		for (auto it = dynamicDeps.begin(); it != dynamicDeps.end(); it++) {
			/*
			bool dynamicDepExists = dynamic_memory_dependence_exists(self, *it, graph);
			bool trueDepExists = DependenceGraph::is_basic_block_dependence_true((*graph)[self].basicblock, (*graph)[*it].basicblock, *depGraph);
			*outputLog << "dynamicDepExists: " << dynamicDepExists << "\n";
			*outputLog << "trueDepExists: " << trueDepExists << "\n";
			if (!StaticDepsOnly && !dynamicDepExists && !trueDepExists) {
				// don't add edge to node for which there are no true dependences
				// nor any dynamic memory dependences
				*outputLog << "Dynamic execution determined no true or memory dependences between ";
				*outputLog << (*graph)[self].name << " (" << self << ") and " << (*graph)[*it].name << " (" << *it << ")\n";
				continue;
			}
			*/

                        DEBUG(*outputLog << "Dynamic execution determined true or memory dependences EXIST between ");
                        DEBUG(*outputLog << (*graph)[self].name << " (" << self << ") and " << (*graph)[*it].name << " (" << *it << ")\n");
			boost::add_edge(*it, self, *graph);
		}

		// update the execution order index for current basic block after it has been processed
		auto search = (*execOrder).find(selfBB);
		assert(search != (*execOrder).end());
		search->second.first++;
	}
	return true;
}


/*
bool AdvisorAnalysis::true_dependence_exists(BasicBlock *BB1, BasicBlock *BB2) {
	// find corresponding edge in depGraph
	return DependenceGraph::is_basic_block_dependence_true(BB1, BB2, depGraph);
}
*/

bool AdvisorAnalysis::dynamic_memory_dependence_exists(TraceGraph_vertex_descriptor child, 
							TraceGraph_vertex_descriptor parent, TraceGraphList_iterator graph) {
	// examine each memory tuple between the two vertices
	// [1] compare parent store with child load RAW
	// [2] compare parent load with child store WAR
	// [3] compare parent store with child store WAW

        DEBUG(*outputLog << "determine if dynamic memory dependences exist between parent (" << parent << ") and child (" << child << ")\n");

	std::vector<std::pair<uint64_t, uint64_t> > &pWrite = (*graph)[parent].memoryWriteTuples;
	std::vector<std::pair<uint64_t, uint64_t> > &cWrite = (*graph)[child].memoryWriteTuples;
	std::vector<std::pair<uint64_t, uint64_t> > &pRead = (*graph)[parent].memoryReadTuples;
	std::vector<std::pair<uint64_t, uint64_t> > &cRead = (*graph)[child].memoryReadTuples;

	DEBUG(*outputLog << "Parent writes: " << pWrite.size() << "\n");
        DEBUG(*outputLog << "Parent reads: " << pRead.size() << "\n");
        DEBUG(*outputLog << "Child writes: " << cWrite.size() << "\n");
        DEBUG(*outputLog << "Child writes: " << cRead.size() << "\n");

	for (auto pwit = pWrite.begin(); pwit != pWrite.end(); pwit++) {
		for (auto cwit = cWrite.begin(); cwit != cWrite.end(); cwit++) {
			// [3]
			if (memory_accesses_conflict(*cwit, *pwit)) {
				*outputLog << "WAW conflict between : (" << pwit->first << ", " << pwit->second;
				*outputLog << ") and (" << cwit->first << ", " << cwit->second << ")\n";;
				return true;
			}
		}
		for (auto crit = cRead.begin(); crit != cRead.end(); crit++) {
			// [1]
			if (memory_accesses_conflict(*crit, *pwit)) {
				*outputLog << "RAW conflict between : (" << pwit->first << ", " << pwit->second;
				*outputLog << ") and (" << crit->first << ", " << crit->second << ")\n";;
				return true;
			}
		}
	}

	for (auto prit = pRead.begin(); prit != pRead.end(); prit++) {
		for (auto cwit = cWrite.begin(); cwit != cWrite.end(); cwit++) {
			// [2]
			if (memory_accesses_conflict(*cwit, *prit)) {
				*outputLog << "WAR conflict between : (" << prit->first << ", " << prit->second;
				*outputLog << ") and (" << cwit->first << ", " << cwit->second << ")\n";;
				return true;
			}
		}
	}

	return false;
}


bool AdvisorAnalysis::memory_accesses_conflict(std::pair<uint64_t, uint64_t> &access1, 
														std::pair<uint64_t, uint64_t> &access2) {
	assert(access1.second > 0 && access2.second > 0);
	if (access1.first > access2.first) {
		if (access1.first < (access2.first + access2.second)) {
			return true;
		}
	} else if (access1.first < access2.first) {
		if (access2.first < (access1.first + access1.second)) {
			return true;
		}
	} else {
		return true;
	}

	return false;
}

void AdvisorAnalysis::print_execution_order(ExecutionOrderList_iterator execOrder) {
	*outputLog << "Execution Order: \n";
	for (auto it = (*execOrder).begin(); it != (*execOrder).end(); it++) {
		*outputLog << it->first->getName() << " ";
		for (auto eit = it->second.second.begin(); eit != it->second.second.end(); eit++) {
			*outputLog << *eit << " ";
		}
		*outputLog << "\n";
	}
}


// sort trace graph vertex descriptor vector in reverse order
bool reverse_vertex_sort(TraceGraph_vertex_descriptor a, TraceGraph_vertex_descriptor b) {
	return b < a;
}

// Function: remove_redundant_dynamic_dependencies
// Given a dynamic trace graph and a vector of vertices for which a executed basic block is
// dependent, remove the dependent vertices which are redundant. A redundant vertices are 
// those which are depended on by other dependent vertices.
void AdvisorAnalysis::remove_redundant_dynamic_dependencies(TraceGraphList_iterator graph, 
									std::vector<TraceGraph_vertex_descriptor> &dynamicDeps) {
	// sort in reverse order, may have more chance to find and remove redundancies if
	// we start with vertices that executed later
	std::sort(dynamicDeps.begin(), dynamicDeps.end(), reverse_vertex_sort);

	for (auto it = dynamicDeps.begin(); it != dynamicDeps.end(); it++) {
		TraceGraph_vertex_descriptor v = *it;
		recursively_remove_redundant_dynamic_dependencies(graph, dynamicDeps, it, v);
	}
}


void AdvisorAnalysis::recursively_remove_redundant_dynamic_dependencies(TraceGraphList_iterator graph, 
				std::vector<TraceGraph_vertex_descriptor> &dynamicDeps, 
				std::vector<TraceGraph_vertex_descriptor>::iterator search, TraceGraph_vertex_descriptor v) {
	// if v already exists as a parent/ancestor, remove from list because it is redundant
	std::vector<TraceGraph_vertex_descriptor>::iterator found = std::find(search+1, dynamicDeps.end(), v);
	if (found != dynamicDeps.end()) {
		dynamicDeps.erase(found);
	}

	// for each of its predecessors, recurse
	TraceGraph_in_edge_iterator ii, ie;
	for (boost::tie(ii, ie) = boost::in_edges(v, *graph); ii != ie; ii++) {
		TraceGraph_vertex_descriptor parent = boost::source(*ii, *graph);
		recursively_remove_redundant_dynamic_dependencies(graph, dynamicDeps, search, parent);
	}
}

#if 0
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
#endif


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


void AdvisorAnalysis::find_new_parents(std::vector<TraceGraph_vertex_descriptor> &newParents, 
		TraceGraph_vertex_descriptor child, TraceGraph_vertex_descriptor parent, TraceGraph &graph) {
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
                DEBUG(*outputLog << "Must come after parent: " << parentBB->getName() << "\n");
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
bool AdvisorAnalysis::annotate_schedule_for_call(Function *F, TraceGraphList_iterator graph_it, 
					std::vector<TraceGraph_vertex_descriptor> &rootVertices, int &lastCycle) {
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
	ScheduleVisitor vis(graph, this, *LT, lastCycle);
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
          TraceGraphVertexWriter<TraceGraph> vpw(*graph, this);
		TraceGraphEdgeWriter<TraceGraph> epw(*graph);
		std::string outfileName = "maximal_schedule." + F->getName().str() + ".dot";
		std::ofstream outfile(outfileName);
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
                DEBUG(*outputLog << "Examine Cycle: " << timestamp << "\n");
		//std::cerr << "Examine Cycle: " << timestamp << "\n";
		// activeBBs keeps track of the number of a particular 
		// basic block resource that is needed to execute all
		// the basic blocks within the anti-chain for each given
		// cycle
		// it stores a pair of basic block ptr and an int representing
		// the number of that basic block needed
		std::map<BasicBlock *, int> activeBBs;
		activeBBs.clear();

		DEBUG(*outputLog << "anti-chain in cycle " << timestamp << ":\n");
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
			DEBUG(*outputLog << BB->getName() << "\n");
		}

		DEBUG(*outputLog << "activeBBs:\n");
		// update the IR
		// will store the replication factor of each basic block as metadata
		// could not find a way to directly attach metadata to each basic block
		// will instead attach to the terminator instruction of each basic block
		// this will be an issue if the basic block is merged/split...
		for (auto it = activeBBs.begin(); it != activeBBs.end(); it++) {
                        DEBUG(*outputLog << it->first->getName() << " repfactor " << it->second << "\n");
			Instruction *inst = dyn_cast<Instruction>(it->first->getTerminator());
			// look at pre-existing replication factor
			LLVMContext &C = inst->getContext();
			//MDNode *N = MDNode::get(C, MDString::get(C, "FPGA_ADVISOR_REPLICATION_FACTOR"));
			// inst->setMetadata(repFactorStr, N);
                        BasicBlock *currBB = (it->first); 

			int repFactor = get_basic_block_instance_count(currBB); // zero indicates CPU execution
                        repFactor = std::max(repFactor, it->second);

                        set_basic_block_instance_count(currBB, repFactor);

		}

		DEBUG(*outputLog << ".\n");
		
		// retire blocks which end this cycle and add their children
		#if 0
		for (auto it = antichain.begin(); /*done in loop body*/; /*done in loop body*/) {
                  DEBUG(*outputLog << "1!!!\n");
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

		
		DEBUG(*outputLog << "antichain size: " << antichain.size() << "\n");
		std::vector<TraceGraph_vertex_descriptor> newantichain;
		newantichain.clear();
		for (auto it = antichain.begin(); it != antichain.end(); ) {
                        DEBUG(*outputLog << *it << " s: " << (*graph)[*it].cycStart << " e: " << (*graph)[*it].cycEnd << "\n");
			if ((*graph)[*it].cycEnd == timestamp) {
				// keep track of the children to add
				TraceGraph_out_edge_iterator oi, oe;
				for (boost::tie(oi, oe) = boost::out_edges(*it, *graph); oi != oe; oi++) {
					// designate the latest finishing parent to add child to antichain
					if (latest_parent(oi, graph)) {
                                          DEBUG(*outputLog << "new elements to add " << boost::target(*oi, *graph));
						newantichain.push_back(boost::target(*oi, *graph));
					}
				}
				DEBUG(*outputLog << "erasing from antichain " << *it << "\n");
				it = antichain.erase(it);
			} else {
				it++;
			}
		}
		
		for (auto it = newantichain.begin(); it != newantichain.end(); it++) {
                        DEBUG(*outputLog << "adding to antichain " << *it << "\n");
			antichain.push_back(*it);
		}

		DEBUG(*outputLog << "-\n");

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
// Performs the gradient descent method for function F to find the optimal
// configuration of blocks on hardware vs. cpu
// Description of gradient descent method:
// With the gradient descent method we are trying to find the best configuration
// of basic blocks to be implemented on fpga and cpu such that we can achieve the
// best performance while satisfying the area constraints on an FPGA
// There are two goals of the optimization:
//	1) Fit the design on the hardware given some constraints
//	2) Maximize the performance
// We start from the maximal parallel configuration which implements the entire program
// on the fpga (as long as they can be implemented on hardware).
// If the design does not fit on the given resources, we find the basic block which
// contributes the least performance/area and remove it (remove an instance/push it
// onto cpu). We iterate this process until the design now "fits"
// If the design fits on the fpga, we now again use the gradient descent method
// to find blocks which contribute zero performance/area and remove them.
void AdvisorAnalysis::find_optimal_configuration_for_all_calls(Function *F, unsigned &cpuOnlyLatency, unsigned fpgaOnlyLatency, unsigned fpgaOnlyArea) {
        DEBUG(*outputLog << __func__ << "\n");
	assert(executionGraph.find(F) != executionGraph.end());

	// default hard-coded area constraint that means nothing

	if (AreaConstraint > 0) {
		areaConstraint = AreaConstraint;
	}

	bool done = false;

	// figure out the final latency when full cpu execution
	cpuOnlyLatency = get_cpu_only_latency(F);
        std::cerr << "CPU-only latency: " << cpuOnlyLatency << "\n";

	// we care about area and delay
	unsigned area = UINT_MAX;
	//unsigned delay = UINT_MAX;

	std::cerr << F->getName().str() << "\n";
	std::cerr << "Progress bar |";

        // Load up basic block counts from metadata
	//for (auto BB = F->begin(); BB != F->end(); BB++) {
        //  load_basic_block_instance_count(BB);
        //}

	while (!done) {
		ConvergenceCounter++; // for stats
		std::cerr << BOLDMAGENTA << "=" << RESET; // progress bar
                
		area = get_area_requirement(F);
		if (area > areaConstraint) {
			*outputLog << "Area constraint violated. Reduce area.\n";
			BasicBlock *removeBB;
			int deltaDelay = INT_MAX;
			bool cpuOnly = !incremental_gradient_descent(F, removeBB, deltaDelay, cpuOnlyLatency, fpgaOnlyLatency, fpgaOnlyArea);
			if (cpuOnly) {
				// decrement all basic blocks until cpu-only
				*outputLog << "[step] Remove all basic blocks\n";
				decrement_all_basic_block_instance_count_and_update_transition(F);
			} else {
				//decrement_basic_block_instance_count(removeBB);
				*outputLog << "[step] Remove basic block: " << removeBB->getName() << "\n";
				decrement_basic_block_instance_count_and_update_transition(removeBB);

				// printout
				*outputLog << "Current basic block configuration.\n";
				print_basic_block_configuration(F, outputLog);
			}
		} else {
			// terminate the process if:
			// 1. removal of block results in increase in delay
			// 2. there are no blocks to remove
			
			*outputLog << "Area constraint satisfied, remove non performing blocks.\n";
			BasicBlock *removeBB = NULL;
			int deltaDelay = INT_MIN;
			incremental_gradient_descent(F, removeBB, deltaDelay, cpuOnlyLatency, fpgaOnlyLatency, fpgaOnlyArea);

			// only remove block if it doesn't negatively impact delay
			if (deltaDelay >= 0 && removeBB != NULL) {
				//decrement_basic_block_instance_count(removeBB);
				*outputLog << "[step+] Remove basic block: " << removeBB->getName() << "\n";
				decrement_basic_block_instance_count_and_update_transition(removeBB);
			}

			// printout
			*outputLog << "Current basic block configuration.\n";
			print_basic_block_configuration(F, outputLog);

			// [1]
			if (deltaDelay < 0) {
				done = true;
			}

			if (removeBB == NULL) {
				done = true;
			}

		}
	}

	std::cerr << ">\n"; // terminate progress bar

        // Dump basic block counts to metadata
	//for (auto BB = F->begin(); BB != F->end(); BB++) {
        //  flush_basic_block_instance_count(BB);
        //}

	// print out final scheduling results and area
	unsigned finalLatency = 0;
	for (TraceGraphList_iterator fIt = executionGraph[F].begin();
		fIt != executionGraph[F].end(); fIt++) {
		std::vector<TraceGraph_vertex_descriptor> roots;
		roots.clear();
		find_root_vertices(roots, fIt);

                std::unordered_map<BasicBlock *, std::pair<bool, std::vector<unsigned> > > resourceTable;
    	        resourceTable.clear();
    	        initialize_resource_table(F, &resourceTable, false);

		finalLatency += schedule_with_resource_constraints(roots, fIt, F, false, &resourceTable);
	}
	
	unsigned finalArea = get_area_requirement(F);

	std::cerr << "Final Latency: " << finalLatency << "\n";
	std::cerr << "Final Area: " << finalArea << "\n";

}

uint64_t rdtsc(){
  unsigned int lo,hi;
  __asm__ __volatile__ ("rdtsc" : "=a" (lo), "=d" (hi));
  return ((uint64_t)hi << 32) | lo;
}

// Function: incremental_gradient_descent
// Function will iterate through each basic block which has a hardware instance of more than 0
// to determine the change in delay with the removal of that basic block and finds the basic block
// whose contribution of delay/area is the least (closest to zero or negative)
bool AdvisorAnalysis::incremental_gradient_descent(Function *F, BasicBlock *&removeBB, int &deltaDelay, unsigned cpuOnlyLatency, unsigned fpgaOnlyLatency, unsigned fpgaOnlyArea) {
	unsigned initialArea = get_area_requirement(F);
	*outputLog << "Initial area: " << initialArea << "\n";
	unsigned initialLatency = 0;

        uint64_t start = rdtsc();  

        unsigned finalArea = initialArea;
        unsigned finalLatency = initialLatency;

        unsigned finalDeltaLatency = 0;
        unsigned finalDeltaArea = 0;

        // this code must go away. 
	// need to loop through all calls to function to get total latency
	for (TraceGraphList_iterator fIt = executionGraph[F].begin();
		fIt != executionGraph[F].end(); fIt++) {
		std::vector<TraceGraph_vertex_descriptor> roots;
		roots.clear();
		find_root_vertices(roots, fIt);
 
                std::unordered_map<BasicBlock *, std::pair<bool, std::vector<unsigned> > > resourceTable;
                resourceTable.clear();
                initialize_resource_table(F, &resourceTable, false);

		initialLatency += schedule_with_resource_constraints(roots, fIt, F, false, &resourceTable);
	}

	// check to see if we should abandon search and opt for cpu only implementation
	// This is the attempt to solve the local minima problem
	// The intuition behind this is that, given a latency-area curve and given that
	// we know the solution for the accelerator only and cpu only implementations,
	// we have an idea of the projected performance that we should beat with the
	// accelerator-cpu implementation. If the performance of that is worse than 
	// the projection and the accelerator area usage is low, we should abandon
	// the search and opt for cpu-only implementation instead.
	//
	//	|
	//	| * *
	//	|*   *
	//	|     *
	//	|      *
	//	|        *
	//	|            *
	//	|                     * *
	//	|____________________________
    //  c       a               f
    //
	//   point a is the point at which the projected performance intersects with the
	//   actual performance, to the left of point a, the performance of a cpu-accelerator
	//   mix will always perform worse than the cpu only
	unsigned B = fpgaOnlyLatency;
	unsigned dA = fpgaOnlyArea - initialArea;

	float m = (cpuOnlyLatency - fpgaOnlyLatency) / fpgaOnlyArea;
	float projectedPerformance = (m * dA) + B;
	DEBUG(*outputLog << "Projected Performance at area is " << projectedPerformance << "\n");

	if ((initialLatency > (unsigned) projectedPerformance) && initialArea < 100 /*hard coded...*/) {
		return false; // go to cpu only solution
	}

	// we set an initial min marginal performance as the average performance/area
	//float minMarginalPerformance = (float) initialLatency / (float) initialArea;
	float minMarginalPerformance = FLT_MAX;

        // we will reuse resource table to avoid all those ugly calls to malloc. 
        // Really, the table could be hoisted even higher, and that will come at some point. 
        std::unordered_map<BasicBlock *, std::pair<bool, std::vector<unsigned> > > resourceTable;
        resourceTable.clear();
        initialize_resource_table(F, &resourceTable, false);

	// try removing each basic block
	for (auto BB = F->begin(); BB != F->end(); BB++) {
		//if (decrement_basic_block_instance_count(BB)) {
                auto search = resourceTable.find(BB);
                std::vector<unsigned> &resourceVector = search->second.second;                
                int count2 = get_basic_block_instance_count(BB);
                int count = resourceVector.size();
                assert(count == count2);

                // this is also pretty inefficient. Fix it too. 
		if (count > 0) {
                        // Provisionally remove block
                        decrement_basic_block_instance_count(BB);
                        // transition costs only happen if we go from accelerator impl. 
                        // to software impl. 
                        if(count == 1) {
                          update_transition(BB);
                        }
                        resourceVector.pop_back();
                          
                        DEBUG(*outputLog << "Performing removal of basic block " << BB->getName() << "\n");
			// need to iterate through all calls made to function
			unsigned latency = 0;

                        // Decrement by one. 


                        // reset all values to zero. 
                        // Really we should do our own maintainence here so as to reduce overhead. 
                        // One could even have a pool of these things reinitialized by a worker thread. 
                        for (auto itRT = resourceTable.begin(); itRT != resourceTable.end(); itRT++) {
                          for (auto itRV = itRT->second.second.begin(); itRV != itRT->second.second.end(); itRV++) {
                            *itRV = 0;
                          }
                        }

			for (TraceGraphList_iterator fIt = executionGraph[F].begin();
				fIt != executionGraph[F].end(); fIt++) {
				std::vector<TraceGraph_vertex_descriptor> roots;
				roots.clear();
				find_root_vertices(roots, fIt);

				// need to update edge weights before scheduling in case any blocks
				// become implemented on cpu
				//update_transition_delay(fIt);

				latency += schedule_with_resource_constraints(roots, fIt, F, false, &resourceTable);
			}

			DEBUG(*outputLog << "New latency: " << latency << "\n");
 
			unsigned area = initialArea - FunctionAreaEstimator::get_basic_block_area(*AT, BB);
			DEBUG(*outputLog << "New area: " << area << "\n");

			float deltaLatency = (float) (initialLatency - latency);
			float deltaArea = (float) (initialArea - area);
			float marginalPerformance;
			if (deltaArea < 0.1) {
				// this block contributes no area
				// never remove a block that contributes no area?? No harm.
				marginalPerformance = FLT_MAX;
			} else {
				marginalPerformance = deltaLatency / deltaArea;
			}

			assert(deltaArea >= 0);
			DEBUG(*outputLog << "marginal performance/area of block " << marginalPerformance << "\n");
			if (marginalPerformance < minMarginalPerformance) {
				minMarginalPerformance = marginalPerformance;
				removeBB = BB;
				DEBUG(*outputLog << "New marginal performing block detected: " << BB->getName() << "\n");
                                finalLatency = latency;
                                finalArea = area;
                                finalDeltaLatency = initialLatency - latency;
                                finalDeltaArea = initialArea - area;
			}

			// restore the basic block count after removal
			increment_basic_block_instance_count(BB);
                        // If we went ACC -> CPU, we need to fixup transition times. 
                        if(count == 1) {
                          update_transition(BB);
                        }
                        resourceVector.push_back(0);
		} // else continue
	}

        uint64_t finish = rdtsc();  

        std::cerr << "IGD Removing BB: " << removeBB << " area: " << finalArea << " ( " << finalDeltaArea << " ) " << " latency: " << finalLatency << " ( " << finalDeltaLatency << " ) in " << finish - start << " cycles" << std::endl;

	return true; // not going to cpu only solution
}
/*
#if 0
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
#endif
*/
/*
#if 0
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
                int count = get_basic_block_instance_count(BB);
		if (count) {
                        DEBUG(*outputLog << "Performing removal of basic block " << BB->getName() << "\n");
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

			DEBUG(*outputLog << "New Latency: " << latency << "\n");

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
			DEBUG(*outputLog << "New Area: " << area << "\n");
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
#endif
*/
unsigned AdvisorAnalysis::get_cpu_only_latency(Function *F) {
	*outputLog << "Calculating schedule for CPU only execution.\n";

	unsigned cpuOnlyLatency = 0;

	// loop through all calls to function, get total latency
	for (TraceGraphList_iterator fIt = executionGraph[F].begin();
		fIt != executionGraph[F].end(); fIt++) {
		std::vector<TraceGraph_vertex_descriptor> roots;
		roots.clear();
		find_root_vertices(roots, fIt);

                std::unordered_map<BasicBlock *, std::pair<bool, std::vector<unsigned> > > resourceTable;
    	        resourceTable.clear();
    	        initialize_resource_table(F, &resourceTable, false);

		cpuOnlyLatency += schedule_with_resource_constraints(roots, fIt, F, true, &resourceTable);
	}

	return cpuOnlyLatency;
}

// Function: schedule_with_resource_constraints
// Return: latency of execution of trace
// This function will use the execution trace graph generated previously 
// and the resource constraints embedded in the IR as metadata to determine
// the latency of the particular function call instance represented by this
// execution trace
unsigned AdvisorAnalysis::schedule_with_resource_constraints(std::vector<TraceGraph_vertex_descriptor> &roots, 
                                                             TraceGraphList_iterator graph_it, Function *F, bool cpuOnly, std::unordered_map<BasicBlock *, std::pair<bool, std::vector<unsigned> > > *resourceTable) {
        DEBUG(*outputLog << __func__ << "\n");

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

	// reset the cpu free cycle global!
	cpuCycle = -1;

	int lastCycle = -1;

	//===----------------------------------------------------===//
	// Use breadth first search to perform the scheduling
	// with resource constraints
	//===----------------------------------------------------===//
	for (std::vector<TraceGraph_vertex_descriptor>::iterator rV = roots.begin();
			rV != roots.end(); rV++) {
		ConstrainedScheduleVisitor vis(graph_it, *LT/*latency of BBs*/, lastCycle, cpuCycle, resourceTable);
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
  bbInstanceCounts[BB] = value;
}


int AdvisorAnalysis::get_basic_block_instance_count(BasicBlock *BB) {
  return bbInstanceCounts[BB];
}

void AdvisorAnalysis::flush_basic_block_instance_count(BasicBlock *BB) {
  //  set_basic_block_instance_count_meta(BB, bbInstanceCounts[BB]);
}

void AdvisorAnalysis::load_basic_block_instance_count(BasicBlock *BB) {
  //  bbInstanceCounts[BB] = get_basic_block_instance_count_meta(BB);
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

// Function: update_transition
void AdvisorAnalysis::update_transition(BasicBlock *BB) {

	Function *F = BB->getParent();

	// if successful, update the transition
	// this is dumb and inefficient, but just do this for now
	for (TraceGraphList_iterator fIt = executionGraph[F].begin(); fIt != executionGraph[F].end(); fIt++) {
		update_transition_delay(fIt);
	}

}


// Function: decrement_basic_block_instance_count_and_update_transition
// Return: false if decrement not successful
bool AdvisorAnalysis::decrement_basic_block_instance_count_and_update_transition(BasicBlock *BB) {
	// decrement
	if (!decrement_basic_block_instance_count(BB)) {
		return false;
	}

	Function *F = BB->getParent();

	// if successful, update the transition
	// this is dumb and inefficient, but just do this for now
	for (TraceGraphList_iterator fIt = executionGraph[F].begin(); fIt != executionGraph[F].end(); fIt++) {
		update_transition_delay(fIt);
	}

	return true;
}


// Function: increment_basic_block_instance_count_and_update_transition
// Return: false if increment not successful
bool AdvisorAnalysis::increment_basic_block_instance_count_and_update_transition(BasicBlock *BB) {
	// decrement
	if (!increment_basic_block_instance_count(BB)) {
		return false;
	}

	Function *F = BB->getParent();

	// if successful, update the transition
	// this is dumb and inefficient, but just do this for now
	for (TraceGraphList_iterator fIt = executionGraph[F].begin(); fIt != executionGraph[F].end(); fIt++) {
		update_transition_delay(fIt);
	}

	return true;
}

void AdvisorAnalysis::decrement_all_basic_block_instance_count_and_update_transition(Function *F) {
	for (auto BB = F->begin(); BB != F->end(); BB++) {
		while(decrement_basic_block_instance_count(BB));
	}

	// this is dumb and inefficient, but just do this for now
	for (TraceGraphList_iterator fIt = executionGraph[F].begin(); fIt != executionGraph[F].end(); fIt++) {
		update_transition_delay(fIt);
	}
}

/*
// Function: get_basic_block_instance_count
// Return: the number of instances of this basic block from metadata
static int AdvisorAnalysis::get_basic_block_instance_count(BasicBlock *BB) {
	assert(BB);
	std::string MDName = "FPGA_ADVISOR_REPLICATION_FACTOR_";
	MDName += BB->getName().str();
	MDNode *M = BB->getTerminator()->getMetadata(MDName);

	int repFactor = -1;

	assert(M);
	assert(M->getOperand(0));

	std::string repFactorStr = cast<MDString>(M->getOperand(0))->getString().str();
	repFactor = stoi(repFactorStr);

	// *outputLog << __func__ << " metadata: ";
	//BB->getTerminator()->print(*outputLog);
	// *outputLog << " replication factor: " << repFactor << "\n";

	return repFactor;
}
*/


// Function: initialize_resource_table
// The resource table represents the resources needed for this program
// the resources we need to consider are:
// HW logic: represented by individual basic blocks
// CPU: represented by a flag
// other??
// FIXME: integrate the cpu
void AdvisorAnalysis::initialize_resource_table(Function *F, std::unordered_map<BasicBlock *, std::pair<bool, std::vector<unsigned> > > *resourceTable, bool cpuOnly) {
	for (auto BB = F->begin(); BB != F->end(); BB++) {
		int repFactor = get_basic_block_instance_count(BB);
		if (repFactor < 0) {
			continue;
		}

		if (repFactor == 0 || cpuOnly) {
			// cpu 
			std::vector<unsigned> resourceVector(0);
			resourceTable->insert(std::make_pair(BB, std::make_pair(true, resourceVector)));
			DEBUG(*outputLog << "Created entry in resource table for basic block: " << BB->getName()
                              << " using cpu resources.\n");
		} else {
			// fpga
			// create a vector
			std::vector<unsigned> resourceVector(repFactor, 0);
			resourceTable->insert(std::make_pair(BB, std::make_pair(false, resourceVector)));

			DEBUG(*outputLog << "Created entry in resource table for basic block: " << BB->getName()
                              << " with " << repFactor << " entries.\n");
		}
	}
}


// Function: get_area_requirement
// Return: a unitless value representing the area 'cost' of a design
// I'm sure this will need a lot of calibration...
unsigned AdvisorAnalysis::get_area_requirement(Function *F) {
	// baseline area required for cpu
	//int area = 1000;
	int area = 0;
	for (auto BB = F->begin(); BB != F->end(); BB++) {
		int areaBB = FunctionAreaEstimator::get_basic_block_area(*AT, BB);
		int repFactor = get_basic_block_instance_count(BB);
		area += areaBB * repFactor;
	}
	return area;
}

// Function: update_transition_delay_basic_block
// updates the trace execution graph edge weights
/*void AdvisorAnalysis::update_transition_delay_basic_block(TraceGraphList_iterator graph) {
	//std::cerr << ">>>>>>>>>>=======================================================\n";
	TraceGraph_edge_iterator ei, ee;
	// look at each edge
	for (boost::tie(ei, ee) = edges(*graph); ei != ee; ei++) {
		TraceGraph_vertex_descriptor s = boost::source(*ei, *graph);
		TraceGraph_vertex_descriptor t = boost::target(*ei, *graph);
		bool sHwExec = (0 < get_basic_block_instance_count((*graph)[s].basicblock));
		bool tHwExec = (0 < get_basic_block_instance_count((*graph)[t].basicblock));
		//std::cerr << "TRANSITION DELAY [" << s << "] -> [" << t << "]\n";;
		//std::cerr << "Source CPU: " << sHwExec << " Target CPU: " << tHwExec << "\n";
		// add edge weight <=> transition delay when crossing a hw/cpu boundary
		unsigned delay = 0;
		if (sHwExec ^ tHwExec) {
			//std::cerr << "Transition cpu<->fpga\n";
			bool CPUToHW = true;
			if (sHwExec == true) {
				// fpga -> cpu
				CPUToHW = false;
			}
			// currently just returns 100
			delay = get_transition_delay((*graph)[s].basicblock, (*graph)[t].basicblock, CPUToHW);
		} else {
			// should have no transition penalty, double make sure
			delay = 0;
		}
		boost::put(boost::edge_weight_t(), *graph, *ei, delay);
	}
	//std::cerr << "<<<<<<<<<<=======================================================\n";
        }*/


// Function: update_transition_delay
// updates the trace execution graph edge weights
void AdvisorAnalysis::update_transition_delay(TraceGraphList_iterator graph) {
	//std::cerr << ">>>>>>>>>>=======================================================\n";
	TraceGraph_edge_iterator ei, ee;
	// look at each edge
	for (boost::tie(ei, ee) = edges(*graph); ei != ee; ei++) {
		TraceGraph_vertex_descriptor s = boost::source(*ei, *graph);
		TraceGraph_vertex_descriptor t = boost::target(*ei, *graph);
		bool sHwExec = (0 < get_basic_block_instance_count((*graph)[s].basicblock));
		bool tHwExec = (0 < get_basic_block_instance_count((*graph)[t].basicblock));
		//std::cerr << "TRANSITION DELAY [" << s << "] -> [" << t << "]\n";;
		//std::cerr << "Source CPU: " << sHwExec << " Target CPU: " << tHwExec << "\n";
		// add edge weight <=> transition delay when crossing a hw/cpu boundary
		unsigned delay = 0;
		if (sHwExec ^ tHwExec) {
			//std::cerr << "Transition cpu<->fpga\n";
			bool CPUToHW = true;
			if (sHwExec == true) {
				// fpga -> cpu
				CPUToHW = false;
			}
			// currently just returns 100
			delay = get_transition_delay((*graph)[s].basicblock, (*graph)[t].basicblock, CPUToHW);
		} else {
			// should have no transition penalty, double make sure
			delay = 0;
		}
		boost::put(boost::edge_weight_t(), *graph, *ei, delay);
	}
	//std::cerr << "<<<<<<<<<<=======================================================\n";
}


// Function: get_transition_delay
// Return: an unsigned int representing the transitional delay between switching from either
// fpga to cpu, or cpu to fpga
unsigned AdvisorAnalysis::get_transition_delay(BasicBlock *source, BasicBlock *target, bool CPUToHW) {
	unsigned delay = 100; // some baseline delay

	if (UserTransitionDelay > 0) {
		delay = UserTransitionDelay;
	}
	
	// need to do something here...
	// the delay shouldn't be constant?
	return delay;
}



void AdvisorAnalysis::print_basic_block_configuration(Function *F, raw_ostream *out) {
	*out << "Basic Block Configuration:\n";
	for (auto BB = F->begin(); BB != F->end(); BB++) {
		int repFactor = get_basic_block_instance_count(BB);
		*out << BB->getName() << "\t[" << repFactor << "]\n";
	}
}

int AdvisorAnalysis::get_total_basic_block_instances(Function *F) {
  int total = 0;
  for (auto BB = F->begin(); BB != F->end(); BB++) {
    total += get_basic_block_instance_count(BB);
  }
  return total;
}

bool AdvisorAnalysis::prune_basic_block_configuration_to_device_area(Function *F) {

  for (auto BB = F->begin(); BB != F->end(); BB++) {
    int areaBB = FunctionAreaEstimator::get_basic_block_area(*AT, BB);
    int repFactor = get_basic_block_instance_count(BB);
    int maxBBCount = areaConstraint/areaBB;
    // Lower repFactor to the maximum for the target FPGA. 
    repFactor = std::min(maxBBCount, repFactor);                
    set_basic_block_instance_count(BB, repFactor);
  }

  return true;
}


void AdvisorAnalysis::print_optimal_configuration_for_all_calls(Function *F) {
	//std::cerr << "PRINTOUT OPTIMAL CONFIGURATION\n";
	int callNum = 0;
	for (TraceGraphList_iterator fIt = executionGraph[F].begin();
			fIt != executionGraph[F].end(); fIt++) {

		callNum++;
		std::string outfileName(F->getName().str() + "." + std::to_string(callNum) + ".final.dot");
		TraceGraphVertexWriter<TraceGraph> vpw(*fIt, this);
		TraceGraphEdgeWriter<TraceGraph> epw(*fIt);
		std::ofstream outfile(outfileName);
		boost::write_graphviz(outfile, *fIt, vpw, epw);
	}
}


bool AdvisorAnalysis::get_dependence_graph_from_file(std::string fileName, DepGraph **DG, std::string funcName) {
	//std::cerr << "***************\n";
	// allocate space for dg
	DepGraph *depGraph = new DepGraph;

	// read file
	ifstream fin;
	fin.open(fileName.c_str());

	if (!fin.good()) {
		return false; // file not found
	}

	std::string line;

	std::cerr << "Getting Dependence Graph from log: " << fileName << "\n";

	while (std::getline(fin, line)) {
		if (std::regex_match(line, std::regex("(vertex )(.*)( )(.*)"))) {
			//===================================//
			// parse line - begin
			//===================================//
			const char *delimiter = " ";
			std::vector<char> lineCopy(line.begin(), line.end());
			lineCopy.push_back(0);

			char *pch = std::strtok(&lineCopy[7], delimiter);
			std::string bbString(pch);
			
			pch = strtok(NULL, delimiter);
			std::string vString(pch);
			//===================================//
			// parse line - end
			//===================================//

			BasicBlock *BB = find_basicblock_by_name(funcName, bbString);
			
			// add vertex
			DepGraph_descriptor currVertex = boost::add_vertex(*depGraph);
			(*depGraph)[currVertex] = BB;

			std::cerr << "found basic block for dep graph: " << BB->getName().str() << "\n";
		} else if (std::regex_match(line, std::regex("(edge )(.*)( )(.*)()(.*)"))) {
			//===================================//
			// parse line - begin
			//===================================//
			const char *delimiter = " ";
			std::vector<char> lineCopy(line.begin(), line.end());
			lineCopy.push_back(0);

			char *pch = std::strtok(&lineCopy[5], delimiter);
			std::string source(pch);

			pch = strtok(NULL, delimiter);
			std::string target(pch);

			pch = strtok(NULL, delimiter);
			std::string trueDep(pch);
			//===================================//
			// parse line - end
			//===================================//

			if (std::atoi(trueDep.c_str()) == 1) {
				boost::add_edge(std::atoi(source.c_str()), std::atoi(target.c_str()), true, *depGraph);
			} else if (std::atoi(trueDep.c_str()) == 0) {
				boost::add_edge(std::atoi(source.c_str()), std::atoi(target.c_str()), false, *depGraph);
			} else {
				assert(0);
			}
		
		} else {
			std::cerr << "Unknown line in " << fileName << "\n";
			assert(0);
		}
	}
	*DG = depGraph;
	return true;
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

void *FunctionAreaEstimator::analyzerLibHandle;
int (*FunctionAreaEstimator::getBlockArea)(BasicBlock *BB);
bool FunctionAreaEstimator::useDefault = true;

void *FunctionScheduler::analyzerLibHandle;
int (*FunctionScheduler::getBlockLatency)(BasicBlock *BB);
int (*FunctionScheduler::getBlockII)(BasicBlock *BB);
bool FunctionScheduler::useDefault = true;
