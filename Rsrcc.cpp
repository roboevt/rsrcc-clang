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
RsrccVisitor::Location RsrccVisitor::Location::INVALID{-1, -1};

std::array<bool, 31 - 5> RsrccVisitor::Register::usedRegs = {false};

namespace {

std::string ESPs = RsrccVisitor::Location::ESP.toString();
std::string EBPs = RsrccVisitor::Location::EBP.toString();
std::string EAXs = RsrccVisitor::Location::EAX.toString();
std::string RTEMPs = RsrccVisitor::Location::RTEMP.toString();

void emit(std::string str) { llvm::outs() << str << "\n"; }

void move(const RsrccVisitor::Location &dest,
          const RsrccVisitor::Location &src) {
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

void moveToPointer(const RsrccVisitor::Location &destAddr,
                   const RsrccVisitor::Location &src) {
  if (destAddr.isReg()) {
    if (src.isReg()) {
      emit("st " + src.toString() + ", 0(" + destAddr.toString() + ") ; mov");
    } else {
      emit("ld " + RsrccVisitor::Location::RTEMP.toString() + ", " +
           src.toString() + " ; mov");
      emit("st " + RsrccVisitor::Location::RTEMP.toString() + ", 0(" +
           destAddr.toString() + ") ; mov");
    }
  } else { // storing to stack
    if (src.isReg()) {
      emit("st " + src.toString() + ", " + destAddr.toString() + " ; mov");
    } else {
      emit("ld " + RsrccVisitor::Location::RTEMP.toString() + ", " +
           src.toString() + " ; mov");
      emit("st " + RsrccVisitor::Location::RTEMP.toString() + ", " +
           destAddr.toString() + " ; mov");
    }
  }
}

void push(int reg) {
  emit("addi " + ESPs + ", " + ESPs + ", -4 ; push0");
  emit("st r" + std::to_string(reg) + ", 0(" + ESPs + ") ; push1");
}

void push(std::shared_ptr<RsrccVisitor::Register> reg) { push(*reg); }

void push(RsrccVisitor::Location reg) {
  if (!reg.isReg()) {
    move(RsrccVisitor::Location::RTEMP, reg);
    push(RsrccVisitor::Location::RTEMP.reg);
  } else {
    push(reg.reg);
  }
}

void pop(int reg) {
  emit("ld r" + std::to_string(reg) + ", 0(" + ESPs + ") ; pop0");
  emit("addi " + ESPs + ", " + ESPs + ", 4 ; pop1");
  // clear stack (doesn't work if reg is rtemp)
  // emit("la " + RTEMPs + ", 0 ; pop clear0");
  // emit("st " + RTEMPs + ", -4(" + ESPs + ") ; pop clear1");
}

void pop(std::shared_ptr<RsrccVisitor::Register> reg) { pop(*reg); }

void pop(RsrccVisitor::Location reg) {
  if (!reg.isReg()) {
    llvm::errs() << "Error: pop " << reg.toString() << " is not a register\n";
    return;
  }
  pop(reg.reg);
}

void ret() {
  auto &reg = RsrccVisitor::Location::RTEMP;
  pop(reg.reg);
  emit("br " + reg.toString() + " ; ret");
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
  push(Location::RTEMP.reg); // Push return address
  emit("la " + RTEMPs + ", " + symTab[funcName].label);
  emit("br " + RTEMPs + " ; call " + funcName);
}

std::string RsrccVisitor::Location::toString() const {
  if (isReg()) {
    return "r" + std::to_string(*reg);
  } else if (isStack()) {
    return std::to_string(stackOffset) + "(" +
           RsrccVisitor::Location::EBP.toString() + ")";
  } else {
    return "unallocated";
  }
}

std::string RsrccVisitor::Location::toRegString() const {
  if (isReg()) {
    return "r" + std::to_string(*reg);
  } else {
    return "unallocated";
  }
}

std::string RsrccVisitor::Location::toStackString() const {
  if (isStack()) {
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
  if (Register::anyRegAvailable()) {
    loc.reg->allocReg();
  } else {
    // TODO allocate on stack
    llvm::errs() << "Error: out of registers\n";
  }
  return loc;
}

RsrccVisitor::Location RsrccVisitor::evaluateVarDecl(VarDecl *Declaration) {
  std::string name = Declaration->getNameAsString();
  // Get parent function name
  FunctionDecl *funcDecl =
      dyn_cast<FunctionDecl>(Declaration->getDeclContext());
  if (!funcDecl) {
    llvm::errs() << "Error: " << name << " is not in a function\n";
    return Location::INVALID;
  }
  std::string funcName = funcDecl->getNameAsString();
  name = funcName + "::" + name;

  if (symTab.find(name) != symTab.end()) {
    llvm::errs() << "Error: " << name << " is already declared\n";
    return Location::INVALID;
  }

  // Alloc loc on stack
  Location loc = {-1, -((currentFuncAllocatedLocals++ + 1) * 4)};

  SymTabEntry entry;
  entry.name = name;
  entry.location = loc;
  symTab[name] = entry;

  debug("VarDecl: " + name + " at " + entry.location.toString());

  if (Declaration->hasInit()) {
    Expr *init = Declaration->getInit();
    Location initLoc = evaluateExpression(init);
    if (isa<ImplicitCastExpr>(init)) { // LvalueToRvalue
      if (initLoc.isStack()) {
        initLoc.reg->allocReg();
        emit("ld " + initLoc.toRegString() + ", " + initLoc.toStackString() +
             " ; init");
      } else {
        emit("ld " + initLoc.toRegString() + ", 0(" + initLoc.toString() +
             ") ; init");
      }
    }
    move(entry.location, initLoc);
  }

  return entry.location;
}

RsrccVisitor::Location RsrccVisitor::evaluateParmVarDecl(ParmVarDecl *decl) {
  std::string name = decl->getNameAsString();
  // Get parent function name
  FunctionDecl *funcDecl = dyn_cast<FunctionDecl>(decl->getDeclContext());
  std::string funcName = funcDecl->getNameAsString();
  name = funcName + "::" + name;

  if (symTab.find(name) != symTab.end()) {
    llvm::errs() << "Error: " << name << " is already declared\n";
    return Location::INVALID;
  }

  SymTabEntry entry;
  entry.name = name;
  entry.location = allocateLoc();
  symTab[name] = entry;

  debug("ParmVarDecl: " + name + " at " + entry.location.toString());

  return entry.location;
}

RsrccVisitor::Location RsrccVisitor::evaluateParmVarDecl(ParmVarDecl *decl,
                                                         int stackOffset) {
  std::string name = decl->getNameAsString();
  // Get parent function name
  FunctionDecl *funcDecl = dyn_cast<FunctionDecl>(decl->getDeclContext());
  std::string funcName = funcDecl->getNameAsString();
  name = funcName + "::" + name;

  if (symTab.find(name) != symTab.end()) {
    llvm::errs() << "Error: " << name << " is already declared\n";
    return Location::INVALID;
  }

  SymTabEntry entry;
  entry.name = name;
  entry.location = {-1, stackOffset};
  symTab[name] = entry;

  debug("ParmVarDecl: " + name + " at " + entry.location.toString());

  return entry.location;
}

RsrccVisitor::Location RsrccVisitor::evaluateAssign(BinaryOperator *op) {
  Expr *lhs = op->getLHS();
  bool LValueToRValue = isa<ImplicitCastExpr>(lhs);
  Expr *rhs = op->getRHS()->IgnoreParenImpCasts();

  Location lhsLoc = evaluateExpression(lhs);
  Location rhsLoc = evaluateExpression(rhs);
  if (isa<ImplicitCastExpr>(rhs)) { // LvalueToRvalue
    emit("ld " + rhsLoc.toString() + ", " + rhsLoc.toString() + " ; assign");
  }

  // Premature optimization
  // if (IntegerLiteral *rhsInt = dyn_cast<IntegerLiteral>(rhs)) {
  //   emit("addi " + lhsLoc.toString() + ", " + lhsLoc.toString() + ", " +
  //        std::to_string(rhsInt->getValue().getSExtValue()) + " ; assign");
  // } else {
  //   Location rhsLoc = evaluateExpression(rhs);
  //   move(lhsLoc, rhsLoc);
  // }

  if (lhsLoc.isStack()) {
    lhsLoc.reg->allocReg();
    emit("st " + lhsLoc.toRegString() + ", " + lhsLoc.toStackString() +
         " ; assign");
    lhsLoc.reg->deallocReg();
  }
  if (!LValueToRValue) {
    // Should move rhs to address in lhsLoc
    moveToPointer(lhsLoc, rhsLoc);
  }

  move(lhsLoc, rhsLoc);

  return lhsLoc;
}

RsrccVisitor::Location RsrccVisitor::evaluateDeclRefExpr(DeclRefExpr *expr) {
  ValueDecl *decl = expr->getDecl();
  std::string name = decl->getNameAsString();
  // Get parent function name
  FunctionDecl *funcDecl = dyn_cast<FunctionDecl>(decl->getDeclContext());
  if (!funcDecl) {
    llvm::errs() << "Error: " << name
                 << " is not in a function, globals not supported yet.\n";
    return Location::INVALID;
  }
  std::string funcName = funcDecl->getNameAsString();
  name = funcName + "::" + name;

  SymTabEntry &entry = symTab[name];
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
  if (Expr *expr = dyn_cast<Expr>(stmt)) {
    return evaluateExpression(expr);
  }
  if (ReturnStmt *stmt2 = dyn_cast<ReturnStmt>(stmt)) {
    return evaluateReturnStmt(stmt2);
  }
  if (IfStmt *stmt2 = dyn_cast<IfStmt>(stmt)) {
    return evaluateIfStmt(stmt2);
  }
  if (WhileStmt *stmt2 = dyn_cast<WhileStmt>(stmt)) {
    return evaluateWhileStmt(stmt2);
  }
  if (ForStmt *stmt2 = dyn_cast<ForStmt>(stmt)) {
    return evaluateForStmt(stmt2);
  }
  if (DeclStmt *stmt2 = dyn_cast<DeclStmt>(stmt)) {
    for (DeclStmt::decl_iterator DI = stmt2->decl_begin();
         DI != stmt2->decl_end(); ++DI) {
      if (isa<VarDecl>(*DI)) {
        evaluateVarDecl(cast<VarDecl>(*DI));
      }
    }
    return Location();
  }
  if (BinaryOperator *stmt2 = dyn_cast<BinaryOperator>(stmt)) {
    return evaluateBinaryOperator(stmt2);
  }
  if (CallExpr *stmt2 = dyn_cast<CallExpr>(stmt)) {
    return evaluateCallExpr(stmt2);
  }
  if (CompoundStmt *stmt2 = dyn_cast<CompoundStmt>(stmt)) {
    for (Stmt::child_iterator i = stmt2->child_begin(); i != stmt2->child_end();
         ++i) {
      evaluateStmt(*i);
    }
    return Location();
  }
  if (ImplicitCastExpr *stmt2 = dyn_cast<ImplicitCastExpr>(stmt)) {
    return evaluateStmt(stmt2->getSubExpr());
  }
  if (NullStmt *stmt2 = dyn_cast<NullStmt>(stmt)) {
    (void)stmt2;
    return Location();
  }

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
  if (UnaryOperator *unaryOperator = dyn_cast<UnaryOperator>(expr)) {
    return evaluateUnaryOperator(unaryOperator);
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

  bool lhsRegOrig = lhsLoc.isReg();
  bool rhsRegOrig = rhsLoc.isReg();

  if (!lhsRegOrig) {
    if (!lhsLoc.reg->allocReg()) {
      llvm::errs() << "Error: compute requires registers\n";
      return Location();
    }
    emit("ld " + lhsLoc.toRegString() + ", " + lhsLoc.toStackString() +
         "; load lhs");
  }

  if (!rhsRegOrig) {
    if (!rhsLoc.reg->allocReg()) {
      llvm::errs() << "Error: compute requires registers\n";
      return Location();
    }
    emit("ld " + rhsLoc.toRegString() + ", " + rhsLoc.toStackString() +
         "; load rhs");
  }

  Location destLoc;
  if (op->isAssignmentOp()) {
    destLoc = lhsLoc;
  } else {
    // We need to allocate a register for the result
    if (!destLoc.reg->allocReg()) {
      llvm::errs() << "Error: compute requires registers, unable to allocate\n";
      return Location();
    }
  }

  if (opStr == "+") {
    emit("add " + destLoc.toString() + ", " + lhsLoc.toString() + ", " +
         rhsLoc.toString());
  } else if (opStr == "-") {
    emit("sub " + destLoc.toString() + ", " + lhsLoc.toString() + ", " +
         rhsLoc.toString());
  } else if (opStr == "<<") {
    emit("shl " + destLoc.toString() + ", " + lhsLoc.toString() + ", " +
         rhsLoc.toString());
  } else if (opStr == ">>") {
    emit("shr " + destLoc.toString() + ", " + lhsLoc.toString() + ", " +
         rhsLoc.toString());
  } else if (opStr == "&") {
    emit("and " + destLoc.toString() + ", " + lhsLoc.toString() + ", " +
         rhsLoc.toString());
  } else if (opStr == "|") {
    emit("or " + destLoc.toString() + ", " + lhsLoc.toString() + ", " +
         rhsLoc.toString());
  } else if (opStr == "^") { // no xor instruction
    // TODO
    llvm::errs() << "Error: unsupported compute (xor coming soon!): " << opStr
                 << "\n";
  } else if (opStr == "+=") {
    emit("add " + destLoc.toString() + ", " + lhsLoc.toString() + ", " +
         rhsLoc.toString());
  } else if (opStr == "-=") {
    emit("sub " + destLoc.toString() + ", " + lhsLoc.toString() + ", " +
         rhsLoc.toString());
  } else if (opStr == "<<=") {
    emit("shl " + destLoc.toString() + ", " + lhsLoc.toString() + ", " +
         rhsLoc.toString());
  } else if (opStr == ">>=") {
    emit("shr " + destLoc.toString() + ", " + lhsLoc.toString() + ", " +
         rhsLoc.toString());
  } else if (opStr == "&=") {
    emit("and " + destLoc.toString() + ", " + lhsLoc.toString() + ", " +
         rhsLoc.toString());
  } else if (opStr == "|=") {
    emit("or " + destLoc.toString() + ", " + lhsLoc.toString() + ", " +
         rhsLoc.toString());
  } else if (opStr == "^=") { // no xor instruction
    // TODO
    llvm::errs() << "Error: unsupported compute (xor coming soon!): " << opStr
                 << "\n";
  } else {
    llvm::errs() << "Error: unsupported compute: " << opStr << "\n";
  }

  // Clean up regs
  if (!rhsRegOrig && lhsLoc.stackOffset != rhsLoc.stackOffset) {
    // store rhs back to stack, unless it's the same as lhs (a += a for example)
    rhsLoc.reg->deallocReg();
  }

  if (!lhsRegOrig && op->isAssignmentOp()) {
    // store lhs back to stack
    // TODO negative offset?
    emit("st " + lhsLoc.toString() + ", " + std::to_string(lhsLoc.stackOffset) +
         "(" + EBPs + ")");
    lhsLoc.reg->deallocReg();
  }

  return destLoc;
}

std::string RsrccVisitor::compareHelper(std::string_view opStr) {

  if (opStr == "<") {
    return "brmi";
  } else if (opStr == ">") {
    return "brpl"; // TODO positive or zero, so gt or eq, not gt
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

  bool lhsRegOrig = lhsLoc.isReg();
  bool rhsRegOrig = rhsLoc.isReg();

  if (!lhsRegOrig) {
    lhsLoc.reg->allocReg();
    emit("ld " + lhsLoc.toRegString() + ", " + lhsLoc.toStackString() +
         "; load lhs");
  }
  if (!rhsRegOrig) {
    rhsLoc.reg->allocReg();
    emit("ld " + rhsLoc.toRegString() + ", " + rhsLoc.toStackString() +
         "; load rhs");
  }

  if (!lhsLoc.isReg() || !rhsLoc.isReg()) {
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

  // Clean up regs
  if (!rhsRegOrig) {
    rhsLoc.reg->deallocReg();
  }
  if (!lhsRegOrig) {
    lhsLoc.reg->deallocReg();
  }

  return Location::EAX;
}

RsrccVisitor::Location
RsrccVisitor::evaluateBinaryOperator(BinaryOperator *op) {
  std::string_view opStr = op->getOpcodeStr();
  if (opStr == "=") {
    return evaluateAssign(op);
  } else if (op->isRelationalOp()) {
    return evaluateCompare(op);
  } else {
    return evaluateCompute(op);
  }
}

RsrccVisitor::Location RsrccVisitor::evaluateUnaryOperator(UnaryOperator *op) {
  std::string_view opStr = op->getOpcodeStr(op->getOpcode());
  if (opStr == "++") {
    if (op->isPrefix()) {
      Expr *expr = op->getSubExpr()->IgnoreParenImpCasts();
      Location loc = evaluateExpression(expr);
      if (loc.isStack()) {
        emit("ld " + RTEMPs + ", " + loc.toString() + " ; pre++");
        emit("addi " + RTEMPs + ", " + RTEMPs + ", 1 ; pre++");
        emit("st " + RTEMPs + ", " + loc.toString() + " ; pre++");
      } else {
        emit("addi " + loc.toString() + ", " + loc.toString() + ", 1 ; pre++");
      }
      return loc;
    } else if (op->isPostfix()) {
      // TODO
      llvm::errs() << "Error: unsupported unary operator: " << opStr
                   << " (use prefix)\n";
      return Location::INVALID;
    }
  }

  if (opStr == "--") {
    if (op->isPrefix()) {
      Expr *expr = op->getSubExpr()->IgnoreParenImpCasts();
      Location loc = evaluateExpression(expr);
      if (loc.isStack()) {
        emit("ld " + RTEMPs + ", " + loc.toString() + " ; pre--");
        emit("addi " + RTEMPs + ", " + RTEMPs + ", -1 ; pre--");
        emit("st " + RTEMPs + ", " + loc.toString() + " ; pre--");
      } else {
        emit("addi " + loc.toString() + ", " + loc.toString() + ", -1 ; pre--");
      }
      return loc;
    } else if (op->isPostfix()) {
      // TODO
      llvm::errs() << "Error: unsupported unary operator: " << opStr
                   << " (use prefix)\n";
      return Location::INVALID;
    }
  }

  if (opStr == "-") {
    Expr *expr = op->getSubExpr()->IgnoreParenImpCasts();
    Location loc = evaluateExpression(expr);
    emit("neg " + loc.toString() + ", " + loc.toString() + " ; neg");
    return loc;
  }

  if (opStr == "~") {
    Expr *expr = op->getSubExpr()->IgnoreParenImpCasts();
    Location loc = evaluateExpression(expr);
    emit("not " + loc.toString() + ", " + loc.toString() + " ; not");
    return loc;
  }

  if (opStr == "!") {
    Expr *expr = op->getSubExpr()->IgnoreParenImpCasts();
    Location loc = evaluateExpression(expr);
    emit("lar " + RTEMPs + ", label" + std::to_string(currentLabel));
    emit("brzr " + RTEMPs + ", " + loc.toString() + " ; bang!");
    emit("lar " + loc.toString() + ", 0");
    emit("lar " + RTEMPs + ", label" + std::to_string(currentLabel + 1));
    emit("br " + RTEMPs + " ; bang!");
    emit("label" + std::to_string(currentLabel) + ":");
    emit("lar " + loc.toString() + ", 1");
    emit("label" + std::to_string(currentLabel + 1) + ":");
    currentLabel += 2;
    return loc;
  }

  if (opStr == "&") {
    Expr *expr = op->getSubExpr()->IgnoreParenImpCasts();
    Location loc = evaluateExpression(expr);
    if (loc.isStack()) { // TODO assert?
      emit("la " + RTEMPs + ", " + loc.toString() + " ; addr");
      return Location::RTEMP;
    } else {
      llvm::errs() << "Error: & requires stack location\n";
      return Location::INVALID;
    }
    return loc;
  }

  if (opStr == "*") {
    Expr *expr = op->getSubExpr()->IgnoreParenImpCasts();
    Location loc = evaluateExpression(expr);
    if (loc.isStack()) {
      emit("la " + RTEMPs + ", " + loc.toStackString() + " ; deref");
      emit("ld " + RTEMPs + ", 0(" + RTEMPs + ") ; deref");
      return Location::RTEMP;
    } else {
      llvm::errs() << "Error: deref requires stack location\n";
      return Location::INVALID;
    }
  }

  llvm::errs() << "Error: unsupported unary operator: " << opStr << "\n";
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

RsrccVisitor::Location RsrccVisitor::evaluateWhileStmt(WhileStmt *stmt) {
  Expr *cond = stmt->getCond()->IgnoreParenImpCasts();
  Stmt *body = stmt->getBody();

  debug("evaluateWhileStmt");

  int condLabel = currentLabel++;
  int endLabel = currentLabel++;

  emit("label" + std::to_string(condLabel) + ":");
  Location condLoc = evaluateExpression(cond);
  emit("lar " + RTEMPs + ", label" + std::to_string(endLabel));
  emit("brzr " + RTEMPs + ", " + condLoc.toString() + " ; while");

  // Body
  if (body) {
    evaluateStmt(body);
  }
  emit("lar " + RTEMPs + ", label" + std::to_string(condLabel));
  emit("br " + RTEMPs + " ; while");

  emit("label" + std::to_string(endLabel) + ":");

  return Location();
}

RsrccVisitor::Location RsrccVisitor::evaluateForStmt(ForStmt *stmt) {
  Stmt *init = stmt->getInit();
  Expr *cond = stmt->getCond();
  Expr *inc = stmt->getInc();
  Stmt *body = stmt->getBody();

  debug("evaluateForStmt");

  int condLabel = currentLabel++;
  int endLabel = currentLabel++;

  // Init
  if (init) {
    evaluateStmt(init);
  }
  emit("label" + std::to_string(condLabel) + ":");
  // Cond
  if (cond) {
    Location condLoc = evaluateExpression(cond);
    emit("lar " + RTEMPs + ", label" + std::to_string(endLabel));
    emit("brzr " + RTEMPs + ", " + condLoc.toString() + " ; for");
  }
  // Body
  if (body) {
    evaluateStmt(body);
  }
  // Inc
  if (inc) {
    evaluateStmt(inc);
  }
  emit("lar " + RTEMPs + ", label" + std::to_string(condLabel));
  emit("br " + RTEMPs + " ; for");

  emit("label" + std::to_string(endLabel) + ":");

  return Location::INVALID;
}

RsrccVisitor::Location RsrccVisitor::evaluateReturnStmt(ReturnStmt *stmt) {
  Expr *expr = stmt->getRetValue()->IgnoreParenImpCasts();
  Location loc = evaluateExpression(expr);
  if (loc.isStack()) {
    emit("ld " + EAXs + ", " + loc.toString() + " ; return mov");
  } else {
    emit("addi " + EAXs + ", " + loc.toString() + ", 0 ; return mov");
  }
  return RsrccVisitor::Location::EAX;
}

RsrccVisitor::Location RsrccVisitor::evaluateCallExpr(CallExpr *expr) {
  FunctionDecl *decl = expr->getDirectCallee();
  std::string name = decl->getNameAsString();

  debug("evaluateCallExpr: " + name);

  // Push caller saved registers (all for now) that are used (slow!)
  for (int i = 1; i < MAX_REGS; i++) {
    if (Register::isUsed(i)) {
      push(i);
      debug("push caller saved register: " + std::to_string(i));
    }
  }

  // Push arguments
  std::vector<Location> argLocs;

  for (Expr *arg : expr->arguments()) {
    Location loc = evaluateExpression(arg);
    argLocs.push_back(loc);
  }
  for (std::vector<Location>::reverse_iterator i = argLocs.rbegin();
       i != argLocs.rend(); ++i) {
    push(*i);
  }

  // Call function
  call(name);

  // Clean stack (args)
  emit("addi " + ESPs + ", " + ESPs + ", " +
       std::to_string(4 * expr->getNumArgs()) + " ; clean stack");

  // Pop caller saved registers (all for now) that are used
  for (int i = 31; i > 0; i--) {
    if (Register::isUsed(i)) {
      pop(i);
    }
  }

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

  // Count locals
  Stmt *body = decl->getBody();
  currentFuncTotalLocals = 0;
  for (Stmt::child_iterator i = body->child_begin(); i != body->child_end();
       ++i) {
    // Check for VarDecls
    if (isa<DeclStmt>(*i)) {
      DeclStmt *declStmt = cast<DeclStmt>(*i);
      for (DeclStmt::decl_iterator DI = declStmt->decl_begin();
           DI != declStmt->decl_end(); ++DI) {
        if (isa<VarDecl>(*DI)) {
          currentFuncTotalLocals++;
          // evaluateVarDecl(cast<VarDecl>(*DI));
        }
      }
    }
  }

  debug("VisitFunctionDecl: " + std::string(decl->getName()) + " with " +
        std::to_string(params.size()) + " params and " +
        std::to_string(currentFuncTotalLocals) + " locals");

  // Emit prolog
  push(Location::EBP);
  emit("addi " + EBPs + ", " + ESPs + ", 0 ; mov ebp, esp");
  // Allcate space for locals (assuming 4 bytes each)
  emit("addi " + ESPs + ", " + ESPs + ", " +
       std::to_string(-4 * currentFuncTotalLocals) + " ; allocate locals");

  // Move params from stack
  int i = 0;
  for (ParmVarDecl *param : params) {
    Location newLoc = evaluateParmVarDecl(param, (i++ + 2) * 4);
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