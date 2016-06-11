#include <string>
#include <map>
#include "BasicBlockDumper.h"

using namespace llvm;

bool dumpBasicBlock(std::string outputLL, llvm::BasicBlock *BB) {

  std::stringstream filename;

  filename.str("");
  filename << "basic_block_" << (long long) BB;

  std::error_code errorInfo;
  llvm::sys::fs::OpenFlags flags = llvm::sys::fs::F_None;                        

  llvm::StringRef file(std::string(filename.str()));
  raw_fd_ostream bbDumpFile(file, errorInfo, flags);


  filename.str("");
  filename << "basic_block_" << (long long) BB << "_old";
  llvm::StringRef fileOld(filename.str());
  raw_fd_ostream bbOldFile(fileOld, errorInfo, flags);

  filename.str("");
  filename << "basic_block_" << (long long) BB << "_trans";
  llvm::StringRef fileTrans(filename.str());
  raw_fd_ostream bbTransFile(fileTrans, errorInfo, flags);

  llvm::StringRef fileNew(outputLL);

  raw_fd_ostream bbNewFile(fileNew, errorInfo, flags);

  BB->print(bbOldFile);

  // create a value map to enable the fixing of instructions by Remap instruction.
  llvm::ValueToValueMapTy VMap;
  
  llvm::Module *funcModule = llvm::CloneModule(BB->getParent()->getParent());
   
  // Remove existing functions. 
  std::vector<llvm::Function*> deadFuncs;                       
  for (Module::iterator F = funcModule->begin(), E = funcModule->end(); F != E; ++F) {
    deadFuncs.push_back(&(*F));
  }

  for (auto F = deadFuncs.begin(); F != deadFuncs.end(); ++F) {
    (*F)->removeFromParent();
  }
   
  // Find external arguments
  std::vector<llvm::Value*> args;
  std::vector<llvm::Type*> argTypes;

  std::vector<llvm::Value*> phiBlockArgs;                       
  std::vector<llvm::BasicBlock*> phiBlock;                        

  // Here, we find out about args.
  for (auto I = BB->begin(); I != BB->end(); I++) {
    bbTransFile << "Examining: "; 
    (*I).print(bbTransFile);
    bbTransFile << "\n"; 

    if(!isa<PHINode>(*I)) {                          
      for (auto Ops = (*I).value_op_begin(); Ops != (*I).value_op_end(); Ops++) {                                    
        if (Instruction *SourceI = dyn_cast<Instruction>(*Ops)) {
          if(SourceI->getParent() != BB) {
            args.push_back(*Ops);
            argTypes.push_back((*Ops)->getType()); 
            (*Ops)->getType()->print(bbTransFile);                               
          }
        }
        else if (isa<llvm::GlobalValue>(*Ops)) {
          // Assume that these global constants
          args.push_back(*Ops);
          argTypes.push_back((*Ops)->getType()); 
          (*Ops)->getType()->print(bbTransFile);                               
        }
        else {
          bbTransFile << "Warning args search found a non-instruction\n";
          (*I).getType()->print(bbTransFile);
          bbTransFile << "\n";
          Ops->print(bbTransFile);
          bbTransFile << "\n";
          Ops->getType()->print(bbTransFile);
          bbTransFile << "\n";
        }                                                    
      }
    } else {
      // We ditch phi nodes and replace them with arguments.
      // 
      llvm::PHINode *PN = dyn_cast<llvm::PHINode>(&(*I));
      for (unsigned val = 0; val < PN->getNumIncomingValues(); val++) {
        llvm::Value *inV = PN->getIncomingValue(val);
        llvm::BasicBlock *inBB = PN->getIncomingBlock(val);
        // for now we will ignore constants, etc.
        if (inBB != BB) {                                  
            args.push_back(inV);
            argTypes.push_back(inV->getType()); 
            phiBlockArgs.push_back(inV);
            phiBlock.push_back(inBB);
            inV->getType()->print(bbTransFile);                                                               
        } 
      }                                                                                  
    }                           
  }


  // find all results 
  std::vector<llvm::Instruction*> results;
  std::vector<llvm::Type*> resultTypes;

  llvm::ReturnInst *retInst = NULL;

  for (auto I = BB->begin(); I != BB->end(); I++) {
    bbTransFile << "Examining Return: "; 
    (*I).print(bbTransFile);
    bbTransFile << "\n"; 
    // Handle Terminators. 
    Instruction *instPtr = &(*I);
    retInst = dyn_cast<llvm::ReturnInst>(instPtr);

    if (retInst == NULL) {
      for (auto Users = (*I).user_begin(); Users != (*I).user_end(); Users++) {
        if (Instruction *UserI = dyn_cast<Instruction>(*Users)) {
          if(UserI->getParent() != BB) {
            results.push_back(& (*I));
            resultTypes.push_back((*I).getType());
          } 
        } else {
          bbTransFile << "Warning result search found a non-instruction\n";
        }
      }
    }                          
  }

  if ((retInst != NULL) && (resultTypes.size() > 0)) {
    bbTransFile << "ERROR Ret instruction found, but some values were still live.\n";
  }

  // create return type from escaped results.  Some blocks have no escaped values, 
  // and must return Null.
  llvm::Type * returnType = llvm::Type::getVoidTy(BB->getContext());
  
  if (retInst != NULL) {
    bbTransFile << "In Ret Inst\n";
    bbTransFile.flush();
    // Don't need a type for a void return. 
    if(retInst->getReturnValue() != NULL) {
      returnType = retInst->getReturnValue()->getType();
    }
  }
    bbTransFile << "Past Ret Inst\n";
    bbTransFile.flush();

  if (resultTypes.size()) {
    llvm::ArrayRef<llvm::Type*> resultTypesArray(resultTypes);
    llvm::StructType *returnStructType = llvm::StructType::create(resultTypesArray); 
    returnType = dyn_cast<Type>(returnStructType);
  }



  llvm::ArrayRef<llvm::Type*> argTypesArray(argTypes);


  //argTypesArray.print(bbTransFile);

  llvm::FunctionType *bbFTy = llvm::FunctionType::get(returnType, argTypesArray, false);

  bbFTy->print(bbTransFile);


  //  FunctionType *FTy = FunctionType::get(F->getFunctionType()->getReturnType(),
  //  ArgTypes, F->getFunctionType()->isVarArg());
  // Create the new function...

  //Function *NewF = Function::Create(FTy, F->getLinkage(), F->getName());
  llvm::Function *F = llvm::Function::Create(bbFTy, llvm::Function::ExternalLinkage, llvm::Twine(filename.str()), funcModule);

  llvm::BasicBlock *exitBB = NULL;
  llvm::BasicBlock *funcBB = NULL;
  // Create an exit basic block.

  // If block terminats in ret, no exit block is required.
  if (retInst == NULL) {
    exitBB = llvm::BasicBlock::Create(BB->getContext(), llvm::Twine(filename.str() + "exit_bb"), F, NULL);

    // create a clone of the basic block of interest. 
    funcBB = llvm::BasicBlock::Create(BB->getContext(), llvm::Twine(filename.str() + "_bb"), F, exitBB);
  } else {
    funcBB = llvm::BasicBlock::Create(BB->getContext(), llvm::Twine(filename.str() + "_bb"), F, NULL);
  }

  // We begin by creating an entry basic block, 
  // which will take care of the phi nodes in the
  // real basic block.
  llvm::BasicBlock *entryBB = llvm::BasicBlock::Create(BB->getContext(), llvm::Twine(filename.str() + "_entry_bb"), F, funcBB);


  // Fix up references to basic block
  VMap[BB] = funcBB;

  // our successor block is now exitBB->
  if(retInst == NULL) {
    if(BB->getTerminator() != NULL) {
      for ( unsigned termIdx = 0; termIdx < BB->getTerminator()->getNumSuccessors(); termIdx++) {     
        if(BB->getTerminator()->getSuccessor(termIdx) != BB) {
          VMap[BB->getTerminator()->getSuccessor(termIdx)] = exitBB;
        }
      }
    }
  }

  // It is only legal to clone a function if a block address within that
  // function is never referenced outside of the function.  Given that, we
  // want to map block addresses from the old function to block addresses in
  // the clone. (This is different from the generic ValueMapper
  // implementation, which generates an invalid blockaddress when
  // cloning a function.)
  if (BB->hasAddressTaken()) {
    Constant *OldBBAddr = BlockAddress::get(BB->getParent(), BB);
    VMap[OldBBAddr] = BlockAddress::get(F, funcBB);
  }
            
                        
  auto argIterator = F->arg_begin();
  int argIdx = 0;

  // create entry block.
  llvm::IRBuilder<> entryBuilder(entryBB);
  for (unsigned phiIdx = 0; phiIdx < phiBlockArgs.size(); phiIdx++) {
    llvm::Value *argVal = &(*argIterator);
    llvm::Value *storeAddr = entryBuilder.CreateAlloca(argVal->getType());
    entryBuilder.CreateStore(argVal, storeAddr);
    llvm::Value *argCpy = entryBuilder.CreateLoad(storeAddr);                                                
    VMap[phiBlockArgs[phiIdx]] = argCpy;
    VMap[phiBlock[phiIdx]] = entryBB;
    argIterator++;
    argIdx++;
  }

  entryBuilder.CreateBr(funcBB);

  // create entry block.
  if (retInst == NULL) {
    llvm::IRBuilder<> exitBuilder(exitBB);
    bbTransFile << "RetType";
    returnType->print(bbTransFile);
    bbTransFile << "\n";        
    bbTransFile.flush();
    if(resultTypes.size() > 0) {
      llvm::Value *retStructAddr = exitBuilder.CreateAlloca(returnType);
      // stuff return values into the return struct
      for (unsigned resIdx = 0; resIdx < resultTypes.size(); resIdx++) {
        //      llvm::Value *resMemberPtr = exitBuilder.CreateStructGEP(returnType, retStructAddr, resIdx);
        llvm::Value *resMemberPtr = exitBuilder.CreateStructGEP(retStructAddr, resIdx);
        exitBuilder.CreateStore(results[resIdx], resMemberPtr);
      }               
      llvm::Value *retVal = exitBuilder.CreateLoad(retStructAddr);
      exitBuilder.CreateRet(retVal);
    } else {
      exitBuilder.CreateRetVoid();
    } 
  }

  // Let's start stuffing the basic block with instructions
  for (auto I = BB->begin(); I != BB->end(); I++) {
    llvm::Instruction *funcI = (*I).clone();
    VMap[&(*I)] = funcI;
    //funcI->setParent(funcBB);
    // Check to see if operands are external. If so, grab 
    // from argument list.
    bbTransFile << " original " << &(*I) << " new " << funcI << "\n";

    if ((*I).hasName()) {
      funcI->setName((*I).getName()+"_bb"); 
    }

    bbTransFile << "Examining: "; 
    (*I).print(bbTransFile);
    bbTransFile << "\n"; 
    if (!isa<PHINode>(*I)) {
      for (unsigned opIdx = 0; opIdx < (*I).getNumOperands(); opIdx++) {
        Value *origOp = (*I).getOperand(opIdx);
        if (Instruction *SourceI = dyn_cast<Instruction>(origOp)) {
          if(SourceI->getParent() != BB) {
            // obtain op from argumentList
            Value *argVal = &(*argIterator);
            bbTransFile << "Checking op " << argIdx << "\n";
            argVal->print(bbTransFile);
            bbTransFile << "\n";
            argVal->getType()->print(bbTransFile); 
            bbTransFile << "\n";
            origOp->print(bbTransFile);
            bbTransFile << "\n";
            origOp->getType()->print(bbTransFile); 
            bbTransFile << "\n";
            argIterator++;
            argIdx++;
            funcI->setOperand(opIdx, argVal);
            bbTransFile << "setting inst op to arg\n";
            (*I).print(bbTransFile);
            bbTransFile << "\n";
            (funcI)->print(bbTransFile);
            bbTransFile << "\n";
          } else {
            bbTransFile << "not setting inst op to arg\n";
            (*I).print(bbTransFile);
            bbTransFile << "\n";
            (funcI)->print(bbTransFile);
            bbTransFile << "\n";
          }
        }
        else if (isa<llvm::GlobalValue>(origOp)) {
          // Assume that these global constants
          Value *argVal = &(*argIterator);
          bbTransFile << "Checking op " << argIdx << "\n";
          argVal->print(bbTransFile);
          bbTransFile << "\n";
          argVal->getType()->print(bbTransFile); 
          bbTransFile << "\n";
          origOp->print(bbTransFile);
          bbTransFile << "\n";
          origOp->getType()->print(bbTransFile); 
          bbTransFile << "\n";
          argIterator++;
          argIdx++;
          funcI->setOperand(opIdx, argVal);
          bbTransFile << "setting inst op to arg\n";
          (*I).print(bbTransFile);
          bbTransFile << "\n";
          (funcI)->print(bbTransFile);
          bbTransFile << "\n";
        }
        else {
            bbTransFile << "Warning result search found a non-instruction\n";
        }
      } 
    } else {
      // We may rescope the PHINode prior blocks.
      // Loop over incoming values
      llvm::PHINode *PN = dyn_cast<llvm::PHINode>(&(*I));
      for (unsigned val = 0; val < PN->getNumIncomingValues(); val++) {
        llvm::Value *V = PN->getIncomingValue(val);
        if (BasicBlock *phiBB = dyn_cast<BasicBlock>(V)) {
          if (phiBB != BB) {
            Value *argVal = &(*argIterator);
            PN->setIncomingValue(val, argVal);
            argIterator++;
            argIdx++;                                                              
          }
        } else {
          bbTransFile << "Non basic block incomign to phinode\n";                                
          V->print(bbTransFile);
          bbTransFile << "\n";
          V->getType()->print(bbTransFile);
          bbTransFile << "\n";
        }
      }                                                                                                              
    }                                                   

    funcBB->getInstList().push_back(funcI); 
  }

  // invoke translation function on basic block
  for (auto I = funcBB->begin(); I != funcBB->end(); I++) {
    bbTransFile << " translating " << &(*I) << "\n";
    (*I).print(bbTransFile);
    bbTransFile.flush();
    bbTransFile << "\n";
    RemapInstruction(&(*I), VMap, llvm:: RF_IgnoreMissingEntries); 
    (*I).print(bbTransFile);
    bbTransFile << "\n";
  }
 
  // Print our fully elaborated function. 
  F->print(bbTransFile);
  funcModule->print(bbNewFile, NULL);
 
  //F->removeFromParent();

  bbNewFile.close();

  return false;
}

bool BasicBlockDumper::runOnBasicBlock(llvm::BasicBlock &BB) {
    std::stringstream filename ;

    filename.str("");
    filename << "basic_block_" << (long long) &BB << "_new";
    
    return dumpBasicBlock(filename.str(), &BB);
}


//char BasicBlockDumper::ID = 0;
//static RegisterPass<BasicBlockDumper> X("basicblockdumper", "FPGA-Advisor basic block dumper", false, false);

