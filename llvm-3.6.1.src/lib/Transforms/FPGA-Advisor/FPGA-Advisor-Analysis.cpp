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
// Describe the analysis details here ----- I am still not sure how it will be
// implemented !!!!!!!!
//
//===----------------------------------------------------------------------===//
//
// Author: chenyuti
//
//===----------------------------------------------------------------------===//

#include "FPGA-Advisor-Analysis.h"
#include <fstream>
#include <regex>
using std::ifstream;
using std::getline;

#define DEBUG_TYPE "fpga-advisor-analysis"

using namespace llvm;

std::error_code AEC;

static cl::opt<std::string> TraceFileName("trace-file", cl::desc("Name of the trace file"), 
		cl::Hidden, cl::init("trace.log"));

// Function: runOnModule
// This is the main analysis pass
bool AdvisorAnalysis::runOnModule(Module &M) {
	raw_fd_ostream OL("fpga-advisor-analysis.log", AEC, sys::fs::F_RW);
	outputLog = &OL;
	DEBUG(outputLog = &dbgs());

	*outputLog << "FPGA-Advisor Analysis Pass Starting.\n";

	mod = &M;

	// read the trace from file into memory
	if (! get_program_trace(TraceFileName)) {
		errs() << "Could not find trace file: " << TraceFileName << "!\n";
		return false;
	}

	// should also contain a sanity check to follow the trace and make sure
	// the paths are valid
	if (! check_trace_sanity()) {
		errs() << "Trace from file is broken, path does not follow control flow graph!\n";
		return false;
	}

	return true;
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
	
	/*
	while(!fin.eof()) {
		// read line into memory
		char buf[1000];
		fin.getline(buf, 1000);

		// parse line
	}
	*/

	std::string line;
	while (std::getline(fin, line)) {
		// There are 3 types of messages:
		//	1. Enter Function: <func name>
		//	2. Basic Block: <basic block name> Function: <func name>
		//	3. Return from: <func name>
		if (std::regex_match(line, std::regex("(Entering Function: )(.*)"))) {
			// nothing to do really...
			errs() << "111\n";
		} else if (std::regex_match(line, std::regex("(BasicBlock: )(.*)( Function: )(.*)"))) {
			// record this information
			errs() << "222\n";

		} else if (std::regex_match(line, std::regex("(Return from: )(.*)"))) {
			// nothing to do really...
			errs() << "333\n";
		} else {
			errs() << "Unexpected trace input!\n" << line << "\n";
			return false;
		}
	}

	return true;
}

// TODO TODO TODO TODO TODO remember to check for external functions, I know
// you're going to forget this!!!!!!!!
bool AdvisorAnalysis::check_trace_sanity() {
	return false;
}





