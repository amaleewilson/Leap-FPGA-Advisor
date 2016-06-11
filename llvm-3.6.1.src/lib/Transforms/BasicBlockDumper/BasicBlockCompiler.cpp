#include "BasicBlockCompiler.h"

std::map<BasicBlock*, BlockMetadata> *metaMap = NULL;

void compileBlock(BasicBlock *BB) {

    std::stringstream filename ;

    filename.str("");
    filename << "basic_block_" << (long long) BB << "_new.ll";
    
    dumpBasicBlock(filename.str(), BB);

    // invoke compiler. 
    // spawn a process
    //spawn a process                                                             
    int compiler_to_advisor[2];
                                                           
    if (pipe(compiler_to_advisor) < 0)
    {
        exit(1);
    }

    int pid_compiler;
    // fork off a process which will eventually invoke the target compiler. 
    pid_compiler = fork();
    if (pid_compiler == 0)
    {
      std::string baseLL(filename.str());
      filename << ".err";
      FILE* errfd = fopen(filename.str().c_str(), "w");

      // child                                                                
      close(compiler_to_advisor[0]);

      dup2(compiler_to_advisor[1], fileno(stdout));  // dump output to log             
      dup2(fileno(errfd), fileno(stderr));  // dump output to log             

      // make incoming fd non-blocking                                        

      fprintf(errfd, "launching basic block compiler %s\n", baseLL.c_str());
      fflush(errfd);
      // Each jtag channel will need some special sauce from                  
      // a particular vendor...                                               
      execlp("analyze-basic-block", "analyze-basic-block", baseLL.c_str(), NULL);
    }

    // parent                                                                     
    close(compiler_to_advisor[1]);
    
    BlockMetadata metadata;
    // put pipe in a buffer 
    FILE *advisor = fdopen(compiler_to_advisor[0],"r");

    fscanf(advisor, "%d,%d,%d", &metadata.area, &metadata.latency, &metadata.ii);

    // use at least one area and latency so as not to screw up the tools.
    if (metadata.area == 0) {
      metadata.area = 1;
    }

    if (metadata.latency == 0) {
      metadata.latency = 1;
    }
 
    fprintf(stderr, "Block %llx area: %d latency: %d ii: %d\n", BB, metadata.area, metadata.latency, metadata.ii);

    // now, wait for the child to emit something. 

    metaMap->insert(std::pair<BasicBlock*, BlockMetadata>(BB, metadata));

    fclose(advisor);
}



