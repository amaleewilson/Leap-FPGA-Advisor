#===-------------------------------------------------------------------------===#
#
# Copyright (c) 2016, Intel Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# Neither the name of the Intel Corporation nor the names of its contributors
# may be used to endorse or promote products derived from this software
# without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#===-------------------------------------------------------------------------===#

#===-------------------------------------------------------------------------===#
# Variables
#===-------------------------------------------------------------------------===#

# set paths
LLVM_HOME=$(LEVEL)../../../
LLVM_BUILD_HOME=$(LLVM_HOME)llvm-build/
LLVM_BUILD=$(LLVM_BUILD_HOME)bin/
LLVM_SRC_HOME=$(LLVM_HOME)llvm-3.6.1.src/

# set to -debug to output debug messages in opt
#DEBUG= -debug
DEBUG= 

# set tools
CLANG=clang
OPT=$(LLVM_BUILD)opt
DIS=$(LLVM_BUILD)llvm-dis
LLI=$(LLVM_BUILD)lli

# set opt related flags
OPT_LD_FLAG= -load=$(LLVM_BUILD_HOME)lib/LLVMFPGA-Advisor.so 
OPT_FLAGS= -O1 -custom-aa 
TRACE_LOG?= trace.log

#ifdef($(PRINT_DEPGRAPH))
#	OPT_FLAGS+= -print-dg 
#endif

#ifeq($(HIDE_GRAPH), 1)
#	OPT_FLAGS+= -hide-graph 
#endif

#ifeq($(NO_OUTPUT), 1)
#	OPT_FLAGS+= -no_message 
#endif

DOT_FILES=$(wildcard *.dot)
PDF_FILES=$(patsubst %.dot, %.pdf, $(DOT_FILES))

# full flow
all:
	make compile
	make instrument
	make analysis

# compile does the frontend compilation of the program and llvm optimization
compile: $(NAME).$(EXT)
	@echo "Running compilation stage..."
	$(CLANG) -S -emit-llvm $(NAME).$(EXT) -o $(NAME).ll
	$(OPT) $(OPT_FLAGS) $(DEBUG) < $(NAME).ll > $(NAME).opt.bc
	$(DIS) $(NAME).opt.bc

# instrumentation stage and run instrumented code
# we may need to deal with code that doesn't finish...
instrument: $(NAME).opt.bc
	@echo "Running instrumentation stage..."
	$(OPT) $(OPT_LD_FLAG) $(OPT_FLAGS) $(DEBUG) -fpga-advisor-instrument < $(NAME).opt.bc > $(NAME).opt.inst.bc
	$(DIS) $(NAME).opt.inst.bc
	$(LLI) $(NAME).opt.inst.bc > $(TRACE_LOG)

# fpga advisor analysis
analysis: $(NAME).opt.bc $(TRACE_LOG)
	@echo "Running analysis stage..."
	$(OPT) $(OPT_LD_FLAG) $(OPT_FLAGS) $(DEBUG) -fpga-advisor-analysis -trace-file=$(TRACE_LOG) < $(NAME).opt.bc > $(NAME).output.bc
	$(DIS) $(NAME).output.bc

# convert all .dot to .pdf
convert-graph: $(PDF_FILES)

# convert .dot to .pdf
%.pdf: %.dot
	dot -Tpdf $^ -o $@

# clean up
clean:
	@echo "Removing outputs"
	rm -rf *.ll *.bc *.dot *.log

# total clean up
cleanall:
	@echo "Removing all outputs"
	clean
	rm -rf *.pdf
