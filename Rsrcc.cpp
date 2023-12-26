// https://stackoverflow.com/questions/71318084/how-visitnameddecl-and-visitcxxrecorddecl-are-called-by-recursiveastvisitor

#include "Rsrcc.h"

using namespace clang;

const RsrccVisitor::Location RsrccVisitor::Location::ESP{
    RsrccVisitor::Location::esp, -1};
const RsrccVisitor::Location RsrccVisitor::Location::EBP{
    RsrccVisitor::Location::ebp, -1};
const RsrccVisitor::Location RsrccVisitor::Location::EAX{
    RsrccVisitor::Location::eax, -1};
const RsrccVisitor::Location RsrccVisitor::Location::RTEMP{
    RsrccVisitor::Location::rtemp, -1};

namespace {

std::string ESPs = RsrccVisitor::Location::ESP.toString();
std::string EBPs = RsrccVisitor::Location::EBP.toString();
std::string EAXs = RsrccVisitor::Location::EAX.toString();
std::string RTEMPs = RsrccVisitor::Location::RTEMP.toString();

void emit(std::string str) { llvm::outs() << str << "\n"; }

void push(int reg) {
  emit("addi " + ESPs + ", " + ESPs + ", -4 ; push0");
  emit("st r" + std::to_string(reg) + ", 0(" + ESPs + ") ; push1");
}

void pop(int reg) {
  emit("ld r" + std::to_string(reg) + ", 0(" + ESPs + ") ; pop0");
  emit("addi " + ESPs + ", " + ESPs + ", 4 ; pop1");
  // clear stack (doesn't work if reg is rtemp)
  // emit("la " + RTEMPs + ", 0 ; pop clear0");
  // emit("st " + RTEMPs + ", -4(" + ESPs + ") ; pop clear1");
}

void push(RsrccVisitor::Location reg) {
  if (!reg.isReg()) {
    llvm::errs() << "Error: push " << reg.toString() << " is not a register\n";
    return;
  }
  push(reg.regNum);
}

void pop(RsrccVisitor::Location reg) {
  if (!reg.isReg()) {
    llvm::errs() << "Error: pop " << reg.toString() << " is not a register\n";
    return;
  }
  pop(reg.regNum);
}

void ret() {
  auto reg = RsrccVisitor::Location::RTEMP;
  pop(reg);
  emit("br " + reg.toString() + " ; ret");
}

void move(RsrccVisitor::Location dest, RsrccVisitor::Location src) {
  if (dest.isReg()) {
    if (src.isReg()) {
      emit("addi " + dest.toString() + ", " + src.toString() + ", 0; mov");
    } else {
      emit("ld " + dest.toString() + ", " + src.toString());
    }
  } else { // storing to stack
    if (src.isReg()) {
      emit("st " + src.toString() + ", " + dest.toString());
    } else {
      emit("ld " + RsrccVisitor::Location::RTEMP.toString() + ", " +
           src.toString());
      emit("st " + RsrccVisitor::Location::RTEMP.toString() + ", " +
           dest.toString());
    }
  }
}
} // namespace

void RsrccVisitor::call(std::string funcName) {
  if (symTab.find(funcName) == symTab.end()) {
    llvm::errs() << "Error: function " << funcName << " is not found\n";
    return;
  }
  emit("brlnv " + RTEMPs);
  // Account for return address + upcoming instructions
  emit("addi " + RTEMPs + ", " + RTEMPs + ", 20");
  push(Location::RTEMP); // Push return address
  emit("la " + RTEMPs + ", " + symTab[funcName].label);
  emit("br " + RTEMPs + " ; call " + funcName);
}

std::string RsrccVisitor::Location::toString() const {
  if (isReg()) {
    return "r" + std::to_string(regNum);
  } else if (isStack()) {
    return std::to_string(stackOffset) + "(" +
           RsrccVisitor::Location::EBP.toString() + ")";
  } else {
    return "unallocated";
  }
}

void RsrccVisitor::debug(std::string str) {
  if (DEBUG)
    llvm::outs() << "\t;debug: " << str << "\n";
}

RsrccVisitor::Location RsrccVisitor::allocateLoc() {
  Location loc;
  if (currentlyAllocatedRegs < MAX_REGS) {
    loc.regNum = currentlyAllocatedRegs++;
  } else {
    // TODO allocate on stack
    llvm::errs() << "Error: out of registers\n";
  }
  return loc;
}

RsrccVisitor::Location RsrccVisitor::evaluateVarDecl(VarDecl *Declaration) {
  std::string name = Declaration->getNameAsString();

  if (symTab.find(name) != symTab.end()) {
    llvm::errs() << "Error: " << name << " is already declared\n";
    return Location();
  }

  SymTabEntry entry;
  entry.name = name;
  entry.location = allocateLoc();
  symTab[name] = entry;

  debug("VarDecl: " + name + " at " + entry.location.toString());

  return entry.location;
}

RsrccVisitor::Location RsrccVisitor::evaluateParmVarDecl(ParmVarDecl *decl) {
  std::string name = decl->getNameAsString();

  if (symTab.find(name) != symTab.end()) {
    llvm::errs() << "Error: " << name << " is already declared\n";
    return Location();
  }

  SymTabEntry entry;
  entry.name = name;
  entry.location = allocateLoc();
  symTab[name] = entry;

  debug("ParmVarDecl: " + name + " at " + entry.location.toString());

  return entry.location;
}

RsrccVisitor::Location RsrccVisitor::evaluateAssign(BinaryOperator *op) {
  Expr *lhs = op->getLHS()->IgnoreParenImpCasts();
  Expr *rhs = op->getRHS()->IgnoreParenImpCasts();

  debug("visitAssign");
  Location lhsLoc = evaluateExpression(lhs);
  Location rhsLoc = evaluateExpression(rhs);

  move(lhsLoc, rhsLoc);

  return lhsLoc;
}

RsrccVisitor::Location RsrccVisitor::evaluateDeclRefExpr(DeclRefExpr *expr) {
  ValueDecl *decl = expr->getDecl();
  std::string name = decl->getNameAsString();

  SymTabEntry entry = symTab[name];
  debug("evaluateDeclRefExpr: " + name + " at " + entry.location.toString());
  if (entry.location.isAllocated()) {
    return entry.location;
  } else {
    llvm::errs() << "Error DeclRefExpr " << name << " is not found\n";
  }

  return entry.location;
}

RsrccVisitor::Location
RsrccVisitor::evaluateIntegerLiteral(IntegerLiteral *expr) {
  Location loc = allocateLoc();
  debug("evaluateIntegerLiteral: " +
        std::to_string(expr->getValue().getSExtValue()) + " at " +
        loc.toString());
  if (loc.isReg()) {
    emit("lar " + loc.toString() + ", " +
         std::to_string(expr->getValue().getSExtValue()));
  } else {
    emit("lar " + Location::RTEMP.toString() + ", " +
         std::to_string(expr->getValue().getSExtValue()));
    emit("st " + Location::RTEMP.toString() + ", " + loc.toString());
  }
  return loc;
}

RsrccVisitor::Location RsrccVisitor::evaluateStmt(Stmt *stmt) {
  if (Expr* expr = dyn_cast<Expr>(stmt)) {
    return evaluateExpression(expr);
  } if(ReturnStmt* stmt2 = dyn_cast<ReturnStmt>(stmt)) {
    return evaluateReturnStmt(stmt2);
  } if(IfStmt* stmt2 = dyn_cast<IfStmt>(stmt)) {
    return evaluateIfStmt(stmt2);
  } if(DeclStmt* stmt2 = dyn_cast<DeclStmt>(stmt)) {
    for (DeclStmt::decl_iterator DI = stmt2->decl_begin();
           DI != stmt2->decl_end(); ++DI) {
        if (isa<VarDecl>(*DI)) {
          evaluateVarDecl(cast<VarDecl>(*DI));
        }
      }
      return Location();
  } if(BinaryOperator* stmt2 = dyn_cast<BinaryOperator>(stmt)) {
    return evaluateBinaryOperator(stmt2);
  } if(CallExpr* stmt2 = dyn_cast<CallExpr>(stmt)) {
    return evaluateCallExpr(stmt2);
  // } if(WhileStmt* stmt = dyn_cast<WhileStmt>(stmt)) {
  //   return evaluateWhileStmt(stmt);
  // } if(ForStmt* stmt = dyn_cast<ForStmt>(stmt)) {
  //   return evaluateForStmt(stmt);
  } if(CompoundStmt* stmt2 = dyn_cast<CompoundStmt>(stmt)) {
    for (Stmt::child_iterator i = stmt2->child_begin(); i != stmt2->child_end();
       ++i) {
      evaluateStmt(*i);
    }
    return Location();
  } if(ImplicitCastExpr* stmt2 = dyn_cast<ImplicitCastExpr>(stmt)) {
    return evaluateStmt(stmt2->getSubExpr());
  } if(NullStmt* stmt2 = dyn_cast<NullStmt>(stmt)) {
    (void)stmt2;
    return Location();
  }
  // } if(DoStmt* stmt = dyn_cast<DoStmt>(stmt)) {
    // return evaluateDoStmt(stmt);
  // } if(BreakStmt* stmt = dyn_cast<BreakStmt>(stmt)) {
  //   return evaluateBreakStmt(stmt);
  // } if(ContinueStmt* stmt = dyn_cast<ContinueStmt>(stmt)) {
  //   return evaluateContinueStmt(stmt);
  // } if(SwitchStmt* stmt = dyn_cast<SwitchStmt>(stmt)) {
  //   return evaluateSwitchStmt(stmt);
  // } if(CaseStmt* stmt = dyn_cast<CaseStmt>(stmt)) {
  //   return evaluateCaseStmt(stmt);
  // } if(DefaultStmt* stmt = dyn_cast<DefaultStmt>(stmt)) {
  //   return evaluateDefaultStmt(stmt);
  // } if(LabelStmt* stmt = dyn_cast<LabelStmt>(stmt)) {
  //   return evaluateLabelStmt(stmt);
  // } if(GotoStmt* stmt

  llvm::errs() << "Error: unsupported statement: ";
  stmt->dump();
  return Location();
}

RsrccVisitor::Location RsrccVisitor::evaluateExpression(Expr *expr) {
  if (DeclRefExpr *declRefExpr = dyn_cast<DeclRefExpr>(expr)) {
    return evaluateDeclRefExpr(declRefExpr);
  }
  if (IntegerLiteral *integerLiteral = dyn_cast<IntegerLiteral>(expr)) {
    return evaluateIntegerLiteral(integerLiteral);
  }
  if (BinaryOperator *binaryOperator = dyn_cast<BinaryOperator>(expr)) {
    return evaluateBinaryOperator(binaryOperator);
  }
  if (ReturnStmt *returnStmt = dyn_cast<ReturnStmt>(expr)) {
    return evaluateReturnStmt(returnStmt);
  }
  if (CallExpr *callExpr = dyn_cast<CallExpr>(expr)) {
    return evaluateCallExpr(callExpr);
  }

  if (isa<ImplicitCastExpr>(expr)) {
    ImplicitCastExpr *implicitCastExpr = cast<ImplicitCastExpr>(expr);
    return evaluateExpression(implicitCastExpr->getSubExpr());
  }
  llvm::errs() << "Error: unsupported expression: ";
  expr->dump();
  return Location();
}

RsrccVisitor::Location RsrccVisitor::evaluateCompute(BinaryOperator *op) {
  std::string_view opStr = op->getOpcodeStr();

  Expr *lhs = op->getLHS()->IgnoreParenImpCasts();
  Expr *rhs = op->getRHS()->IgnoreParenImpCasts();

  debug("visitCompute: " + std::string(opStr));
  Location lhsLoc = evaluateExpression(lhs);
  Location rhsLoc = evaluateExpression(rhs);

  if (!lhsLoc.isReg() || !rhsLoc.isReg()) {
    llvm::errs() << "Error: compute requires registers\n";
    return Location();
  }

  if (opStr == "+") {
    emit("add " + lhsLoc.toString() + ", " + lhsLoc.toString() + ", " +
         rhsLoc.toString());
  } else if (opStr == "-") {
    emit("sub " + lhsLoc.toString() + ", " + lhsLoc.toString() + ", " +
         rhsLoc.toString());
  } else {
    llvm::errs() << "Error: unsupported compute: " << opStr << "\n";
  }

  return lhsLoc;
}

std::string RsrccVisitor::compareHelper(std::string_view opStr) {
  
  if (opStr == "<") {
    return "brmi";
  } else if (opStr == ">") {
    return "brpl";  // TODO positive or zero, so gt or eq, not gt
  } else {
    llvm::errs() << "Error: unsupported compare: " << opStr << "\n";
    return "";
  }
}

RsrccVisitor::Location RsrccVisitor::evaluateCompare(BinaryOperator *op) {
  std::string_view opStr = op->getOpcodeStr();

  Expr *lhs = op->getLHS()->IgnoreParenImpCasts();
  Expr *rhs = op->getRHS()->IgnoreParenImpCasts();

  debug("visitCompare: " + std::string(opStr));
  Location lhsLoc = evaluateExpression(lhs);
  Location rhsLoc = evaluateExpression(rhs);

  if (!lhsLoc.isReg() || !rhsLoc.isReg()) {
    // TODO compare stack
    llvm::errs() << "Error: compare requires registers\n";
    return Location();
  }

  std::string instruction = compareHelper(opStr);

  int falseLabel = currentLabel++;
  int endLabel = currentLabel++;
  // Comparison
  emit("sub " + EAXs + ", " + lhsLoc.toString() + ", " + rhsLoc.toString());
  emit("lar " + RTEMPs + ", label" + std::to_string(falseLabel));
  emit(instruction + " " + RTEMPs + ", " + EAXs + "; cmp");
  // True
  emit("lar " + EAXs + ", 0");
  emit("lar " + RTEMPs + ", label" + std::to_string(endLabel));
  emit("br " + RTEMPs + " ; cmp");
  // False
  emit("label" + std::to_string(falseLabel) + ":");
  emit("lar " + EAXs + ", 1");
  emit("label" + std::to_string(endLabel) + ":");

  return Location::EAX;
}

RsrccVisitor::Location
RsrccVisitor::evaluateBinaryOperator(BinaryOperator *op) {
  std::string_view opStr = op->getOpcodeStr();
  if (opStr == "=") {
    return evaluateAssign(op);
  } else if (opStr == "+" || opStr == "-") {
    return evaluateCompute(op);
  } else if (opStr == "<" || opStr == ">") {
    return evaluateCompare(op);
  }

  llvm::errs() << "Error: unsupported binary operator: " << opStr << "\n";
  return Location();
}

RsrccVisitor::Location RsrccVisitor::evaluateIfStmt(IfStmt *stmt) {
  Expr *cond = stmt->getCond()->IgnoreParenImpCasts();
  Stmt *then = stmt->getThen();
  Stmt *elseStmt = stmt->getElse();

  debug("evaluateIfStmt");

  int elseLabel = currentLabel++;
  int endLabel = currentLabel++;

  Location condLoc = evaluateExpression(cond);
  emit("lar " + RTEMPs + ", label" + std::to_string(elseLabel));
  emit("brzr " + RTEMPs + ", " + condLoc.toString() + " ; if");

  // Then
  if (then) {
    evaluateStmt(then);
  }
  emit("lar " + RTEMPs + ", label" + std::to_string(endLabel));
  emit("br " + RTEMPs + " ; if");

  // Else
  emit("label" + std::to_string(elseLabel) + ":");
  if (elseStmt) {
    evaluateStmt(elseStmt);
  }
  emit("label" + std::to_string(endLabel) + ":");

  return Location();
}

RsrccVisitor::Location RsrccVisitor::evaluateReturnStmt(ReturnStmt *stmt) {
  Expr *expr = stmt->getRetValue()->IgnoreParenImpCasts();
  Location loc = evaluateExpression(expr);
  emit("addi " + RsrccVisitor::Location::EAX.toString() + ", " +
       loc.toString() + ", 0; mov");
  return RsrccVisitor::Location::EAX;
}

RsrccVisitor::Location RsrccVisitor::evaluateCallExpr(CallExpr *expr) {
  FunctionDecl *decl = expr->getDirectCallee();
  std::string name = decl->getNameAsString();

  debug("evaluateCallExpr: " + name);

  // Push arguments
  for (Expr *arg : expr->arguments()) {
    Location loc = evaluateExpression(arg);
    push(loc);
  }

  // Call function
  call(name);

  // Clean stack (args)
  emit("addi " + ESPs + ", " + ESPs + ", " +
       std::to_string(4 * expr->getNumArgs()) + " ; clean stack");

  return RsrccVisitor::Location::EAX;
}

bool RsrccVisitor::evaluateFunctionDecl(FunctionDecl *decl) {
  std::string label;
  if (decl->getNameAsString() == "main") {
    label = std::string(ENTRY_POINT);
  } else {
    label = "func" + std::to_string(currentLabel++);
  }

  // Add to symbol table
  SymTabEntry entry;
  entry.name = decl->getNameAsString();
  entry.label = label;
  symTab[entry.name] = entry;

  // Emit label
  emit(label + ":");

  ArrayRef<ParmVarDecl *> params = decl->parameters();

  Stmt *body = decl->getBody();
  int numLocals = 0;
  for (Stmt::child_iterator i = body->child_begin(); i != body->child_end();
       ++i) {
    // Check for VarDecls
    if (isa<DeclStmt>(*i)) {
      DeclStmt *declStmt = cast<DeclStmt>(*i);
      for (DeclStmt::decl_iterator DI = declStmt->decl_begin();
           DI != declStmt->decl_end(); ++DI) {
        if (isa<VarDecl>(*DI)) {
          numLocals++;
          // evaluateVarDecl(cast<VarDecl>(*DI));
        }
      }
    }
  }

  debug("VisitFunctionDecl: " + std::string(decl->getName()) + " with " +
        std::to_string(params.size()) + " params and " +
        std::to_string(numLocals) + " locals");

  // Emit prolog
  push(Location::EBP);
  emit("addi " + EBPs + ", " + ESPs + ", 0 ; mov ebp, esp");
  // Allcate space for locals (assuming 4 bytes each)
  emit("addi " + ESPs + ", " + ESPs + ", " + std::to_string(-4 * numLocals) +
       " ; allocate locals");

  // Move params from stack
  int i = 0;
  for (ParmVarDecl *param : params) {
    Location newLoc = evaluateParmVarDecl(param);
    Location currentLoc{-1, (i++ + 2) * 4};
    move(newLoc, currentLoc);
  }

  // Emit body
  for (Stmt::child_iterator i = body->child_begin(); i != body->child_end();
       ++i) {
    evaluateStmt(*i);
  }

  // Emit epilog
  emit("addi " + ESPs + ", " + EBPs + ", 0 ; mov esp, ebp");
  pop(Location::EBP);
  ret();

  return false;
}

bool RsrccVisitor::VisitTranslationUnitDecl(TranslationUnitDecl *decl) {
  debug("VisitTranslationUnitDecl");
  for (Decl *child : decl->decls()) {
    if (FunctionDecl *funcDecl = dyn_cast<FunctionDecl>(child)) {
      if (funcDecl->getNameAsString() == "main") {
        // Setup stack
        emit(".org " + std::to_string(STACK_BEGIN));
        emit("STACK: .dw " + std::to_string(STACK_SIZE / 4));
        emit(".org 0");
        emit("la " + ESPs + ", STACK");
        emit("la " + EBPs + ", STACK");

        // Create main stack frame
        emit("la " + RTEMPs + ", END ; main exit");
        push(Location::RTEMP); // Push return address
        emit("la " + RTEMPs + ", " +
             std::string(ENTRY_POINT)); // must evaluate main first
        emit("br " + RTEMPs + " ; call " + "main");
      }
    }
  }
  for (Decl *child : decl->decls()) {
    if (FunctionDecl *funcDecl = dyn_cast<FunctionDecl>(child)) {
      evaluateFunctionDecl(funcDecl);
    }
  }

  // Emit end
  emit("stop");
  emit("la " + RTEMPs + ", END");
  emit("END:");
  emit("br " + RTEMPs);
  return false; // I guess this means this will only work for one TU
}