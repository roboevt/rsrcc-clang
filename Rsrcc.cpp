// https://stackoverflow.com/questions/71318084/how-visitnameddecl-and-visitcxxrecorddecl-are-called-by-recursiveastvisitor

#include "Rsrcc.h"

using namespace clang;

RsrccVisitor::Location RsrccVisitor::Location::INVALID{-1};

const RsrccVisitor::Register RsrccVisitor::Register::ESP{1};
const RsrccVisitor::Register RsrccVisitor::Register::EBP{2};
const RsrccVisitor::Register RsrccVisitor::Register::EAX{3};

std::array<bool, 31 - 5> RsrccVisitor::Register::usedRegs = {false};

namespace {

std::string ESPs = RsrccVisitor::Register::ESP.toString();
std::string EBPs = RsrccVisitor::Register::EBP.toString();
std::string EAXs = RsrccVisitor::Register::EAX.toString();

std::shared_ptr<RsrccVisitor::Register> ESPp =
    std::make_shared<RsrccVisitor::Register>(RsrccVisitor::Register::ESP.reg);
std::shared_ptr<RsrccVisitor::Register> EBPp =
    std::make_shared<RsrccVisitor::Register>(RsrccVisitor::Register::EBP.reg);
std::shared_ptr<RsrccVisitor::Register> EAXp =
    std::make_shared<RsrccVisitor::Register>(RsrccVisitor::Register::EAX.reg);

void emit(std::string str) { llvm::outs() << str << "\n"; }

void move(const RsrccVisitor::Location &dst,
          const RsrccVisitor::Location &src) {
  std::cout << ("; move: " + dst.toString() + " <- " + src.toString())
            << std::endl;
  RsrccVisitor::Register reg;
  if (dst.indirectionLevel > 1) { // move to pointer
    std::cout << "; move to pointer" << std::endl;
    if (src.isReg()) {
      if (dst.isReg()) {
        emit("st " + src.toString() + ", 0(" + dst.toString() + ") ; mov");
      } else {
        emit("ld " + reg.toString() + ", " + dst.toString() + " ; mov");
        emit("st " + src.toString() + ", 0(" + reg.toString() + ") ; mov");
      }
    } else { // src is stack
      if (dst.isReg()) {
        emit("ld " + dst.toString() + ", " + src.toString() + " ; mov");
      } else {
        emit("ld " + reg.toString() + ", " + src.toString() + " ; mov");
        emit("st " + reg.toString() + ", 0(" + dst.toString() + ") ; mov");
      }
    }
  } else { // move to lvalue
    if (dst.isStack()) {
      if (src.isReg()) {
        emit("st " + src.toRegString() + ", " + dst.toStackString() + " ; mov");
      } else {
        emit("ld " + reg.toString() + ", " + src.toRegString() + " ; mov");
        emit("st " + reg.toString() + ", " + dst.toStackString() + " ; mov");
      }
    } else { // storing to register
      if (src.isReg()) {
        emit("addi " + dst.toRegString() + ", " + src.toRegString() +
             ", 0 ; mov");
      } else {
        emit("ld " + dst.toRegString() + ", " + src.toStackString() + " ; mov");
      }
    }
  }
}

// void moveToAddr(const RsrccVisitor::Location &destAddr,
//                 const RsrccVisitor::Location &src) {
//   RsrccVisitor::Register reg;
//   if (destAddr.isReg()) {
//     if (src.isReg()) {
//       emit("st " + src.toString() + ", 0(" + destAddr.toString() + ") ;
//       mov");
//     } else {
//       emit("ld " + reg.toString() + ", " + src.toString() + " ; mov");
//       emit("st " + reg.toString() + ", 0(" + destAddr.toString() + ") ;
//       mov");
//     }
//   } else { // storing to stack
//     if (src.isReg()) {
//       emit("st " + src.toString() + ", " + destAddr.toString() + " ; mov");
//     } else {
//       emit("ld " + reg.toString() + ", " + src.toString() + " ; mov");
//       emit("st " + reg.toString() + ", " + destAddr.toString() + " ; mov");
//     }
//   }
// }

void move(std::shared_ptr<RsrccVisitor::Register> dst,
          const RsrccVisitor::Location &src) {
  RsrccVisitor::Location dstLoc = {dst};
  move(dstLoc, src);
}

// void move(const RsrccVisitor::Location &dst,
//           std::shared_ptr<RsrccVisitor::Register> src) {
//   RsrccVisitor::Location srcLoc = {src};
//   move(dst, srcLoc);
// }

// void moveToPointer(const RsrccVisitor::Location &destAddr,
//                    const RsrccVisitor::Location &src) {
//   if (destAddr.isReg()) {
//     if (src.isReg()) {
//       emit("st " + src.toString() + ", 0(" + destAddr.toString() + ") ;
//       mov");
//     } else {
//       emit("ld " + RsrccVisitor::Location::RTEMP.toString() + ", " +
//            src.toString() + " ; mov");
//       emit("st " + RsrccVisitor::Location::RTEMP.toString() + ", 0(" +
//            destAddr.toString() + ") ; mov");
//     }
//   } else { // storing to stack
//     if (src.isReg()) {
//       emit("st " + src.toString() + ", " + destAddr.toString() + " ; mov");
//     } else {
//       emit("ld " + RsrccVisitor::Location::RTEMP.toString() + ", " +
//            src.toString() + " ; mov");
//       emit("st " + RsrccVisitor::Location::RTEMP.toString() + ", " +
//            destAddr.toString() + " ; mov");
//     }
//   }
// }

// RsrccVisitor::Location derefToReg(const RsrccVisitor::Location &loc) {
//   RsrccVisitor::Location addr;
//   addr.addr->allocReg();
//   if (loc.isStack()) {
//     emit("ld " + addr.toString() + ", " + loc.toString() + " ; deref");
//   } else {
//     emit("ld " + addr.toString() + ", 0(" + loc.toString() + ") ; deref");
//   }
//   return addr;
// }

RsrccVisitor::Location deref(const RsrccVisitor::Location &loc) {
  RsrccVisitor::Location addr;
  addr.indirectionLevel = loc.indirectionLevel - 1;
  addr.reg->allocReg();
  if (loc.isStack()) {
    emit("ld " + addr.toRegString() + ", " + loc.toStackString() + " ; deref1");
  } else {
    emit("ld " + addr.toRegString() + ", 0(" + loc.toRegString() +
         ") ; deref1");
  }
  emit("ld " + addr.toRegString() + ", 0(" + addr.toRegString() + ") ; deref2");
  return addr;
}

// RsrccVisitor::Location derefInPlace(const RsrccVisitor::Location &loc) {
//   if (loc.isStack()) {
//     RsrccVisitor::Register val, addr;
//     emit("ld " + val.toString() + ", " + loc.toString() + " ; deref1");
//     emit("la " + addr.toString() + ", " + loc.toString() + " ; deref2");
//     emit("st " + val.toString() + ", 0(" + addr.toString() + ") ; deref3");
//   } else {
//     emit("ld " + loc.toString() + ", 0(" + loc.toString() + ") ; deref");
//   }
//   return loc; // By definition of in place
// }

void push(int reg) {
  emit("addi " + ESPs + ", " + ESPs + ", -4 ; push1");
  emit("st r" + std::to_string(reg) + ", 0(" + ESPs + ") ; push2");
}

void push(RsrccVisitor::Register &reg) { push(reg.reg); }

void pop(int reg) {
  emit("ld r" + std::to_string(reg) + ", 0(" + ESPs + ") ; pop0");
  emit("addi " + ESPs + ", " + ESPs + ", 4 ; pop1");
  // clear stack (doesn't work if reg is rtemp)
  // emit("la " + RTEMPs + ", 0 ; pop clear0");
  // emit("st " + RTEMPs + ", -4(" + ESPs + ") ; pop clear1");
}

void pop(RsrccVisitor::Register &reg) { pop(reg.reg); }

void ret() {
  RsrccVisitor::Register reg;
  pop(reg);
  emit("br " + reg.toString() + " ; ret");
}

} // namespace

std::string RsrccVisitor::Register::toString() const {
  return "r" + std::to_string(reg);
}

std::string RsrccVisitor::Location::toString() const {
  if (isStack()) {
    return std::to_string(stackOffset) + "(" +
           RsrccVisitor::Register::EBP.toString() + ")";
  } else if (isReg()) {
    return reg->toString();
  } else {
    llvm::errs() << "Error: toString called on unallocated location\n";
    return "unallocated";
  }
}

std::string RsrccVisitor::Location::toRegString() const {
  if (isReg()) {
    return "r" + std::to_string(*reg);
  } else {
    llvm::errs() << "Error: toRegString called on unallocated location\n";
    return "Reg unallocated";
  }
}

std::string RsrccVisitor::Location::toAddrString() const {
  if (isReg()) {
    return "0(" + reg->toString() + ")";
  } else {
    llvm::errs() << "Error: toAddrString called on unallocated location\n";
    return "Addr unallocated";
  }
}

std::string RsrccVisitor::Location::toStackString() const {
  if (isStack()) {
    return std::to_string(stackOffset) + "(" +
           RsrccVisitor::Register::EBP.toString() + ")";
  } else {
    llvm::errs() << "Error: toStackString called on unallocated location\n";
    return "unallocated";
  }
}

void RsrccVisitor::call(std::string funcName) {
  if (symTab.find(funcName) == symTab.end()) {
    llvm::errs() << "Error: function " << funcName << " is not found\n";
    return;
  }
  RsrccVisitor::Register reg;
  emit("brlnv " + reg.toString() + " ; call1 " + funcName);
  // Account for return address + upcoming instructions
  emit("addi " + reg.toString() + ", " + reg.toString() + ", 20 ; call2 " +
       funcName);
  push(reg); // Push return address
  emit("la " + reg.toString() + ", " + symTab[funcName].label + " ; call3 " +
       funcName);
  emit("br " + reg.toString() + " ; call4 " + funcName);
}

void RsrccVisitor::debug(std::string str) {
  if (DEBUG)
    llvm::outs() << "\t;debug: " << str << "\n";
}

// TODO alloca? For locals within scope such as for loop.
// RsrccVisitor::Location RsrccVisitor::allocateLoc() {
//   Location loc;
//   if (Register::anyRegAvailable()) {
//     loc.reg->allocReg();
//   } else {
//     // TODO allocate on stack
//     llvm::errs() << "Error: out of registers\n";
//   }
//   return loc;
// }

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

  int size = 1;
  int indirectionLevel = 1;
  if (auto arrDecl =
          dyn_cast<ConstantArrayType>(Declaration->getType().getTypePtr())) {
    size = arrDecl->getSize().getSExtValue();
    indirectionLevel++;
  }

  // Alloc loc on stack
  Location loc(-((currentFuncAllocatedLocals + 1) * 4));
  currentFuncAllocatedLocals += size;

  SymTabEntry entry;
  entry.name = name;
  entry.location = loc;
  entry.location.indirectionLevel = indirectionLevel;

  debug("VarDecl: " + name + " at " + entry.location.toString());

  if (Declaration->hasInit()) {
    Expr *init = Declaration->getInit();
    Location initLoc = evaluateExpression(init->IgnoreParenImpCasts());
    move(entry.location, initLoc);
  }

  const Type *type = Declaration->getType().getTypePtr();
  while (type->isPointerType()) {
    entry.location.indirectionLevel++;
    type = type->getPointeeType().getTypePtr();
  }
  if (type->isArrayType()) {
    entry.location.indirectionLevel++;
  }
  symTab[name] = entry;

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

  Location loc = {((currentFuncArgs++ + 1) * 4)};

  SymTabEntry entry;
  entry.name = name;
  entry.location = loc;
  entry.location.indirectionLevel = 1; // var is an lvalue
  const Type *type = decl->getType().getTypePtr();
  while (type->isPointerType()) {
    entry.location.indirectionLevel++;
    type = type->getPointeeType().getTypePtr();
  }
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
  entry.location = {stackOffset};
  entry.location.indirectionLevel = 1; // var is an lvalue
  const Type *type = decl->getType().getTypePtr();
  while (type->isPointerType()) {
    entry.location.indirectionLevel++;
    type = type->getPointeeType().getTypePtr();
  }
  symTab[name] = entry;

  debug("ParmVarDecl: " + name + " at " + entry.location.toString());

  return entry.location;
}

RsrccVisitor::Location RsrccVisitor::evaluateAssign(BinaryOperator *op) {
  debug("visitAssign");
  Expr *lhs = op->getLHS();
  Expr *rhs = op->getRHS();

  // Location lhsLoc = evaluateExpression(lhs);
  Location lhsLoc = evaluateExpressionLvalue(lhs);
  Location rhsLoc = evaluateExpression(rhs);

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
  debug("evaluateDeclRefExpr: " + name + " at " + entry.location.toString() +
        " ind " + std::to_string(entry.location.indirectionLevel));
  if (entry.location.isAllocated()) {
    return entry.location;
  } else {
    llvm::errs() << "Error DeclRefExpr " << name << " is not found\n";
    return Location::INVALID;
  }
}

RsrccVisitor::Location
RsrccVisitor::evaluateIntegerLiteral(IntegerLiteral *expr) {
  Location loc;
  if (!loc.reg->allocReg()) {
    llvm::errs() << "Error: integer literal requires registers\n";
    return Location::INVALID;
  }
  debug("evaluateIntegerLiteral: " +
        std::to_string(expr->getValue().getSExtValue()) + " at " +
        loc.toString());
  emit("lar " + loc.toString() + ", " +
       std::to_string(expr->getValue().getSExtValue()) + " ; int literal");
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
    return evaluateImplicitCastExpr(stmt2);
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
  if (ImplicitCastExpr *castExpr = dyn_cast<ImplicitCastExpr>(expr)) {
    return evaluateImplicitCastExpr(castExpr);
  }
  if (ArraySubscriptExpr *arraySubsciptExpr =
          dyn_cast<ArraySubscriptExpr>(expr)) {
    return evaluateArraySubscriptExpr(arraySubsciptExpr);
  }
  llvm::errs() << "Error: unsupported expression: ";
  expr->dump();
  return Location();
}

RsrccVisitor::Location RsrccVisitor::evaluateExpressionLvalue(Expr *expr) {
  if (UnaryOperator *unaryOperator = dyn_cast<UnaryOperator>(expr)) {
    debug("evaluateExpressionLvalue: UnaryOperator");
    if (unaryOperator->getOpcode() == UO_Deref) {
      return evaluateDerefLvalue(unaryOperator);
    }
  }
  if (DeclRefExpr *declRefExpr = dyn_cast<DeclRefExpr>(expr)) {
    return evaluateDeclRefExpr(declRefExpr);
  }
  if (ImplicitCastExpr *castExpr = dyn_cast<ImplicitCastExpr>(expr)) {
    return evaluateExpression(castExpr->getSubExpr());
  }
  if (ArraySubscriptExpr *arraySubsciptExpr =
          dyn_cast<ArraySubscriptExpr>(expr)) { // TODO funcion
    return evaluateArraySubscriptExprLvalue(arraySubsciptExpr);
  }
  llvm::errs() << "Error: unsupported expression: " << expr->getStmtClassName()
               << "\n";
  return Location::INVALID;
}

RsrccVisitor::Location RsrccVisitor::evaluateCompute(BinaryOperator *op) {
  std::string_view opStr = op->getOpcodeStr();

  Expr *lhs = op->getLHS()->IgnoreParenImpCasts();
  Expr *rhs = op->getRHS()->IgnoreParenImpCasts();

  debug("visitCompute: " + std::string(opStr));
  Location lhsLoc = evaluateExpression(lhs);
  Location rhsLoc = evaluateExpression(rhs);

  bool lhsStack = lhsLoc.isStack();
  bool rhsStack = rhsLoc.isStack();

  // Computation must be done between registers, so we need to load from stack
  if (lhsStack) {
    if (!lhsLoc.reg->allocReg()) {
      llvm::errs() << "Error: compute requires registers\n";
      return Location::INVALID;
    }
    emit("ld " + lhsLoc.toRegString() + ", " + lhsLoc.toStackString() +
         "; load lhs");
  }

  if (rhsStack) {
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
    emit("add " + destLoc.toRegString() + ", " + lhsLoc.toRegString() + ", " +
         rhsLoc.toRegString());
  } else if (opStr == "-") {
    emit("sub " + destLoc.toRegString() + ", " + lhsLoc.toRegString() + ", " +
         rhsLoc.toRegString());
  } else if (opStr == "<<") {
    emit("shl " + destLoc.toRegString() + ", " + lhsLoc.toRegString() + ", " +
         rhsLoc.toRegString());
  } else if (opStr == ">>") {
    emit("shr " + destLoc.toRegString() + ", " + lhsLoc.toRegString() + ", " +
         rhsLoc.toRegString());
  } else if (opStr == "&") {
    emit("and " + destLoc.toRegString() + ", " + lhsLoc.toRegString() + ", " +
         rhsLoc.toRegString());
  } else if (opStr == "|") {
    emit("or " + destLoc.toRegString() + ", " + lhsLoc.toRegString() + ", " +
         rhsLoc.toRegString());
  } else if (opStr == "^") { // no xor instruction
    // TODO
    llvm::errs() << "Error: unsupported compute (xor coming soon!): " << opStr
                 << "\n";
  } else if (opStr == "+=") {
    // if (!LValueToRValue) {
    //   emit("add " + lhsLoc.toString() + ", " + lhsLoc.toString() + ", " +
    //        rhsLoc.toString());
    // } else {
    //   Location tempLoc;
    //   tempLoc.reg->allocReg();
    //   emit("ld " + tempLoc.toRegString() + ", 0(" + lhsLoc.toRegString() +
    //        ") ; += load lhs");
    //   emit("add " + tempLoc.toString() + ", " + tempLoc.toString() + ", " +
    //        rhsLoc.toString());
    //   emit("st " + tempLoc.toString() + ", 0(" + lhsLoc.toRegString() +
    //        ") ; += store lhs");
    // }
    if (lhsStack) {
      emit("add " + lhsLoc.toRegString() + ", " + lhsLoc.toRegString() + ", " +
           rhsLoc.toRegString() + " ; += compute");
      emit("st " + lhsLoc.toRegString() + ", " + lhsLoc.toStackString() +
           " ; += store");
    } else {
      emit("add " + lhsLoc.toRegString() + ", " + lhsLoc.toRegString() + ", " +
           rhsLoc.toRegString());
    }
  } else if (opStr == "-=") {
    if (lhsStack) {
      emit("sub " + lhsLoc.toRegString() + ", " + lhsLoc.toRegString() + ", " +
           rhsLoc.toRegString() + " ; -= compute");
      emit("st " + lhsLoc.toRegString() + ", " + lhsLoc.toStackString() +
           " ; -= store");
    } else {
      emit("sub " + lhsLoc.toRegString() + ", " + lhsLoc.toRegString() + ", " +
           rhsLoc.toRegString());
    }
  } else if (opStr == "<<=") {
    if (lhsStack) {
      emit("shl " + lhsLoc.toRegString() + ", " + lhsLoc.toRegString() + ", " +
           rhsLoc.toRegString() + " ; <<= compute");
      emit("st " + lhsLoc.toRegString() + ", " + lhsLoc.toStackString() +
           " ; <<= store");
    } else {
      emit("shl " + lhsLoc.toRegString() + ", " + lhsLoc.toRegString() + ", " +
           rhsLoc.toRegString());
    }
  } else if (opStr == ">>=") {
    if (lhsStack) {
      emit("shr " + lhsLoc.toRegString() + ", " + lhsLoc.toRegString() + ", " +
           rhsLoc.toRegString() + " ; >>= compute");
      emit("st " + lhsLoc.toRegString() + ", " + lhsLoc.toStackString() +
           " ; >>= store");
    } else {
      emit("shr " + lhsLoc.toRegString() + ", " + lhsLoc.toRegString() + ", " +
           rhsLoc.toRegString());
    }
  } else if (opStr == "&=") {
    if (lhsStack) {
      emit("and " + lhsLoc.toRegString() + ", " + lhsLoc.toRegString() + ", " +
           rhsLoc.toRegString() + " ; &= compute");
      emit("st " + lhsLoc.toRegString() + ", " + lhsLoc.toStackString() +
           " ; &= store");
    } else {
      emit("and " + lhsLoc.toRegString() + ", " + lhsLoc.toRegString() + ", " +
           rhsLoc.toRegString());
    }
  } else if (opStr == "|=") {
    if (lhsStack) {
      emit("or " + lhsLoc.toRegString() + ", " + lhsLoc.toRegString() + ", " +
           rhsLoc.toRegString() + " ; |= compute");
      emit("st " + lhsLoc.toRegString() + ", " + lhsLoc.toStackString() +
           " ; |= store");
    } else {
      emit("or " + lhsLoc.toRegString() + ", " + lhsLoc.toRegString() + ", " +
           rhsLoc.toRegString());
    }
  } else if (opStr == "^=") { // no xor instruction
    // TODO
    llvm::errs() << "Error: unsupported compute (xor coming soon!): " << opStr
                 << "\n";
  } else {
    llvm::errs() << "Error: unsupported compute: " << opStr << "\n";
  }

  // Clean up regs
  if (lhsStack && op->isAssignmentOp()) {
    lhsLoc.reg->deallocReg();
  }
  if (rhsStack) {
    // store rhs back to stack, unless it's the same as lhs (a += a for example)
    rhsLoc.reg->deallocReg();
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

  Expr *lhs = op->getLHS();
  // bool LValueToRValue = isa<ImplicitCastExpr>(lhs);
  Expr *rhs = op->getRHS();

  debug("visitCompare: " + std::string(opStr));
  Location lhsLoc = evaluateExpression(lhs);
  Location rhsLoc = evaluateExpression(rhs);

  bool lhsStack = lhsLoc.isStack();
  bool rhsStack = rhsLoc.isStack();

  if (lhsStack) {
    if (!lhsLoc.reg->allocReg()) {
      llvm::errs() << "Error: compare requires registers\n";
      return Location::INVALID;
    }
    emit("ld " + lhsLoc.toRegString() + ", " + lhsLoc.toStackString() +
         "; compare load lhs");
  }
  if (rhsStack) {
    if (!rhsLoc.reg->allocReg()) {
      llvm::errs() << "Error: compare requires registers\n";
      return Location::INVALID;
    }
    emit("ld " + rhsLoc.toRegString() + ", " + rhsLoc.toStackString() +
         "; compare load rhs");
  }

  Register addr;

  std::string instruction = compareHelper(opStr);

  int falseLabel = currentLabel++;
  int endLabel = currentLabel++;
  // Comparison
  emit("sub " + EAXs + ", " + lhsLoc.toRegString() + ", " +
       rhsLoc.toRegString());
  emit("lar " + addr.toString() + ", label" + std::to_string(falseLabel));
  emit(instruction + " " + addr.toString() + ", " + EAXs + "; cmp");
  // True
  emit("lar " + EAXs + ", 0");
  emit("lar " + addr.toString() + ", label" + std::to_string(endLabel));
  emit("br " + addr.toString() + " ; cmp");
  // False
  emit("label" + std::to_string(falseLabel) + ":");
  emit("lar " + EAXs + ", 1");
  emit("label" + std::to_string(endLabel) + ":");

  // Clean up regs
  if (lhsStack) {
    lhsLoc.reg->deallocReg();
  }
  if (rhsStack) {
    rhsLoc.reg->deallocReg();
  }

  return Location(EAXp);
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
  std::string lval = op->isLValue() ? "lval" : "rval";
  std::string_view opStr = op->getOpcodeStr(op->getOpcode());
  debug("visitUnaryOperator: " + std::string(opStr) + " " + lval);
  Location loc = evaluateExpression(op->getSubExpr());

  if (opStr == "++") {
    if (op->isPrefix()) {
      if (loc.isStack()) {
        Register reg;
        emit("ld " + reg.toString() + ", " + loc.toString() + " ; pre++");
        emit("addi " + reg.toString() + ", " + reg.toString() + ", 1 ; pre++");
        emit("st " + reg.toString() + ", " + loc.toString() + " ; pre++");
      } else {
        emit("addi " + loc.toRegString() + ", " + loc.toRegString() +
             ", 1 ; pre++");
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
      if (loc.isStack()) {
        Register reg;
        emit("ld " + reg.toString() + ", " + loc.toStackString() + " ; pre--");
        emit("addi " + reg.toString() + ", " + reg.toString() + ", -1 ; pre--");
        emit("st " + reg.toString() + ", " + loc.toStackString() + " ; pre--");
      } else {
        emit("addi " + loc.toRegString() + ", " + loc.toRegString() +
             ", -1 ; pre--");
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
    if (loc.isStack()) {
      Register reg;
      emit("ld " + reg.toString() + ", " + loc.toStackString() + " ; neg");
      emit("neg " + reg.toString() + ", " + reg.toString() + " ; neg");
      emit("st " + reg.toString() + ", " + loc.toStackString() + " ; neg");
    } else {
      emit("sub " + loc.toRegString() + ", " + loc.toRegString() + ", 0 ; neg");
    }
    return loc;
  }

  if (opStr == "~") {
    if (loc.isStack()) {
      Register reg;
      emit("ld " + reg.toString() + ", " + loc.toStackString() + " ; not");
      emit("not " + reg.toString() + ", " + reg.toString() + " ; not");
      emit("st " + reg.toString() + ", " + loc.toStackString() + " ; not");
    } else {
      emit("not " + loc.toRegString() + ", " + loc.toRegString() + " ; not");
    }
    return loc;
  }

  if (opStr == "!") {
    Register addr;
    if (loc.isStack()) {
      loc.reg->allocReg();
      move(loc.reg, loc);
    }
    emit("lar " + addr.toString() + ", label" + std::to_string(currentLabel) +
         " ; bang1");
    emit("brzr " + addr.toString() + ", " + loc.toRegString() + " ; bang2");
    emit("lar " + loc.toRegString() + ", 0 ; bang3");
    emit("lar " + addr.toString() + ", label" +
         std::to_string(currentLabel + 1) + " ; bang4");
    emit("br " + addr.toString() + " ; bang5");
    emit("label" + std::to_string(currentLabel) + ":");
    emit("lar " + loc.toRegString() + ", 1");
    emit("label" + std::to_string(currentLabel + 1) + ":");
    currentLabel += 2;

    if (loc.isStack()) {
      emit("st " + loc.toRegString() + ", " + loc.toStackString() + " ; bang6");
      loc.reg->deallocReg();
    }
    return loc;
  }

  if (opStr == "&") {
    if (loc.isStack()) {
      Location loc2;
      loc2.reg->allocReg();
      emit("la " + loc2.toRegString() + ", " + loc.toStackString() + " ; &");
      loc2.indirectionLevel++;
      return loc2;
    } else {
      llvm::errs() << "Error: & requires stack location\n";
      return Location::INVALID;
    }
  }

  if (opStr == "*") {
    return deref(loc);
  }

  llvm::errs() << "Error: unsupported unary operator: " << opStr << "\n";
  return Location::INVALID;
}

RsrccVisitor::Location RsrccVisitor::evaluateDerefLvalue(UnaryOperator *op) {
  Location addr;
  addr.reg->allocReg();
  Location loc = evaluateExpression(op->getSubExpr());
  addr.indirectionLevel = loc.indirectionLevel;
  if (loc.isStack()) {
    emit("ld " + addr.toRegString() + ", " + loc.toStackString() + " ; deref");
  } else {
    emit("ld " + addr.toRegString() + ", 0(" + loc.toRegString() + ") ; deref");
  }
  return addr;
}

RsrccVisitor::Location
RsrccVisitor::evaluateImplicitCastExpr(ImplicitCastExpr *expr) {
  // check if array to pointer
  if (expr->getCastKind() == CK_ArrayToPointerDecay) {
    Location loc = evaluateExpression(expr->getSubExpr());
    Location reg;
    reg.indirectionLevel = loc.indirectionLevel - 1;
    reg.reg->allocReg();
    emit("la " + reg.toRegString() + ", " + loc.toStackString() +
         " ; array to pointer");
    return reg;
  }
  return evaluateExpression(expr->getSubExpr());
}

RsrccVisitor::Location
RsrccVisitor::evaluateArraySubscriptExpr(ArraySubscriptExpr *expr) {
  Expr *base = expr->getBase();
  Expr *index = expr->getIdx();
  Location baseLoc = evaluateExpression(base);
  Location indexLoc = evaluateExpression(index);

  debug("arraySubscriptExpr ind: " + std::to_string(baseLoc.indirectionLevel));

  if (!indexLoc.isReg()) {
    indexLoc.reg->allocReg();
    emit("ld " + indexLoc.toRegString() + ", " + indexLoc.toString() +
         " ; array subscript index load");
  }

  emit("shl " + indexLoc.toRegString() + ", " + indexLoc.toRegString() +
       ", 2 ; array subscript index");
  emit("neg " + indexLoc.toRegString() + ", " + indexLoc.toRegString() +
       " ; array subscript index"); // Stack grows downwards. // TODO make
                                    // arr[0] bottom of array instead of top.
  if (baseLoc.isStack()) {
    Location reg;
    if (baseLoc.isReg()) {
      reg = baseLoc;
    } else {
      reg.indirectionLevel = baseLoc.indirectionLevel;
      debug("array ind: " + std::to_string(reg.indirectionLevel));
      reg.reg->allocReg();
      emit("la " + reg.toRegString() + ", " + baseLoc.toStackString() +
           " ; array subscript base");
           if(reg.indirectionLevel > 1) {  // TODO loop?
             emit("ld " + reg.toRegString() + ", 0(" + reg.toRegString() + ") ; array subscript base deref");
           }
    }
    emit("add " + reg.toRegString() + ", " + reg.toString() + ", " +
         indexLoc.toRegString() + " ; array subscript loc");
    emit("ld " + reg.toRegString() + ", 0(" + reg.toRegString() +
         ") ; array subscript deref");
    return reg;
  } else {
    llvm::errs() << "Error: array subscript requires stack location\n";
    return Location::INVALID;
  }
}

RsrccVisitor::Location
RsrccVisitor::evaluateArraySubscriptExprLvalue(ArraySubscriptExpr *expr) {
  Expr *base = expr->getBase();
  Expr *index = expr->getIdx();
  Location baseLoc = evaluateExpression(base);
  Location indexLoc = evaluateExpression(index);

  debug("arraySubscriptExprLval ind: " +
        std::to_string(baseLoc.indirectionLevel));

  if (!indexLoc.isReg()) {
    indexLoc.reg->allocReg();
    emit("ld " + indexLoc.toRegString() + ", " + indexLoc.toStackString() +
         " ; array subscript index load");
  }

  emit("shl " + indexLoc.toRegString() + ", " + indexLoc.toRegString() +
       ", 2 ; array subscript index");
  emit("neg " + indexLoc.toRegString() + ", " + indexLoc.toRegString() +
       " ; array subscript index"); // Stack grows downwards. // TODO make
                                    // arr[0] bottom of array instead of top.
  // Location reg;
  // reg.indirectionLevel = baseLoc.indirectionLevel - 1;
  // reg.reg->allocReg();
  // emit("la " + reg.toRegString() + ", " + baseLoc.toStackString() +
  //      " ; array subscript base");
  emit("add " + baseLoc.toRegString() + ", " + baseLoc.toRegString() + ", " +
       indexLoc.toRegString() + " ; array subscript loc");
  return baseLoc;
}

RsrccVisitor::Location RsrccVisitor::evaluateIfStmt(IfStmt *stmt) {
  Expr *cond = stmt->getCond()->IgnoreParenImpCasts();
  Stmt *then = stmt->getThen();
  Stmt *elseStmt = stmt->getElse();

  debug("evaluateIfStmt");

  int elseLabel = currentLabel++;
  int endLabel = currentLabel++;

  Location condLoc = evaluateExpression(cond);
  Register addr;
  emit("lar " + addr.toString() + ", label" + std::to_string(elseLabel));
  emit("brzr " + addr.toString() + ", " + condLoc.toString() + " ; if");

  // Then
  if (then) {
    evaluateStmt(then);
  }
  emit("lar " + addr.toString() + ", label" + std::to_string(endLabel));
  emit("br " + addr.toString() + " ; if");

  // Else
  emit("label" + std::to_string(elseLabel) + ":");
  if (elseStmt) {
    evaluateStmt(elseStmt);
  }
  emit("label" + std::to_string(endLabel) + ":");

  return Location();
}

RsrccVisitor::Location RsrccVisitor::evaluateWhileStmt(WhileStmt *stmt) {
  Expr *cond = stmt->getCond();
  Stmt *body = stmt->getBody();

  debug("evaluateWhileStmt");

  int condLabel = currentLabel++;
  int endLabel = currentLabel++;

  emit("label" + std::to_string(condLabel) + ":");

  Location condLoc = evaluateExpression(cond);
  // if (!condLoc.isReg()) {
  //   condLoc.reg->allocReg();
  //   emit("ld " + condLoc.toRegString() + ", " + condLoc.toString() +
  //        " ; while load");
  // }

  Register addr;
  emit("lar " + addr.toString() + ", label" + std::to_string(endLabel));
  emit("brzr " + addr.toString() + ", " + condLoc.toRegString() + " ; while");

  // Body
  if (body) {
    evaluateStmt(body);
  }
  emit("lar " + addr.toString() + ", label" + std::to_string(condLabel));
  emit("br " + addr.toString() + " ; while");

  emit("label" + std::to_string(endLabel) + ":");

  if (condLoc.isStack()) {
    condLoc.reg->deallocReg();
  }

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

  Register addr;

  // Init
  if (init) {
    evaluateStmt(init);
  }
  emit("label" + std::to_string(condLabel) + ":");
  // Cond
  if (cond) {
    Location condLoc = evaluateExpression(cond);
    emit("lar " + addr.toString() + ", label" + std::to_string(endLabel));
    emit("brzr " + addr.toString() + ", " + condLoc.toString() + " ; for");
  }
  // Body
  if (body) {
    evaluateStmt(body);
  }
  // Inc
  if (inc) {
    evaluateStmt(inc);
  }
  emit("lar " + addr.toString() + ", label" + std::to_string(condLabel));
  emit("br " + addr.toString() + " ; for");

  emit("label" + std::to_string(endLabel) + ":");

  return Location::INVALID;
}

RsrccVisitor::Location RsrccVisitor::evaluateReturnStmt(ReturnStmt *stmt) {
  ;
  Location loc = evaluateExpression(stmt->getRetValue());
  if (loc.isStack()) {
    emit("ld " + EAXs + ", " + loc.toString() + " ; return mov");
  } else {
    emit("addi " + EAXs + ", " + loc.toString() + ", 0 ; return mov");
  }
  return Location(EAXp);
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
  currentFuncArgs = 0;

  for (Expr *arg : expr->arguments()) {
    Location loc = evaluateExpression(arg);
    argLocs.push_back(loc);
  }
  for (std::vector<Location>::reverse_iterator i = argLocs.rbegin();
       i != argLocs.rend(); ++i) {
    Register reg;
    if (i->isReg()) {
      push(*(i->reg));
    } else {
      emit("ld " + reg.toString() + ", " + i->toStackString() + " ; push arg");
      push(reg);
    }
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

  return Location(EAXp);
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
  currentFuncAllocatedLocals = 0;
  for (Stmt::child_iterator i = body->child_begin(); i != body->child_end();
       ++i) {
    // Check for VarDecls
    if (isa<DeclStmt>(*i)) {
      DeclStmt *declStmt = cast<DeclStmt>(*i);
      for (DeclStmt::decl_iterator DI = declStmt->decl_begin();
           DI != declStmt->decl_end(); ++DI) {
        if (auto *varDecl = dyn_cast<VarDecl>(*DI)) {
          if (auto arrDecl = dyn_cast<ConstantArrayType>(
                  varDecl->getType().getTypePtr())) {
            currentFuncTotalLocals += arrDecl->getSize().getSExtValue();
            debug("Found local array " + varDecl->getNameAsString() + " with " +
                  std::to_string(arrDecl->getSize().getSExtValue()) +
                  " elements");
          } else {
            // local var
            currentFuncTotalLocals++;
          }
        }
      }
    }
  }

  debug("VisitFunctionDecl: " + std::string(decl->getName()) + " with " +
        std::to_string(params.size()) + " params and " +
        std::to_string(currentFuncTotalLocals) + " locals");

  // Emit prolog
  push(Register::EBP);
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
  pop(Register::EBP);
  ret();

  return false;
}

bool RsrccVisitor::VisitTranslationUnitDecl(TranslationUnitDecl *decl) {
  debug("VisitTranslationUnitDecl");
  for (Decl *child : decl->decls()) {
    if (FunctionDecl *funcDecl = dyn_cast<FunctionDecl>(child)) {
      if (funcDecl->getNameAsString() == "main") {
        // Setup stack
        emit(".org " + std::to_string(STACK_BEGIN + STACK_SIZE));
        emit("STACK: .dw " + std::to_string(1));
        emit(".org 0");
        emit("la " + ESPs + ", STACK");
        emit("la " + EBPs + ", STACK");

        // Create main stack frame
        Register reg;
        emit("la " + reg.toString() + ", END ; main exit");
        push(reg); // Push return address
        emit("la " + reg.toString() + ", " +
             std::string(ENTRY_POINT)); // must evaluate main first
        emit("br " + reg.toString() + " ; call " + "main");
        reg.deallocReg(); // TODO needed? automatic, right?
      }
    }
  }
  for (Decl *child : decl->decls()) {
    if (FunctionDecl *funcDecl = dyn_cast<FunctionDecl>(child)) {
      evaluateFunctionDecl(funcDecl);
    }
  }

  // Emit end
  Register reg;
  emit("stop");
  emit("la " + reg.toString() + ", END");
  emit("END:");
  emit("br " + reg.toString());
  return false; // I guess this means this will only work for one TU
}