#include "clang/AST/RecursiveASTVisitor.h"
#include "clang/Tooling/Tooling.h"

#include <iostream>
#include <unordered_map>

class RsrccVisitor : public clang::RecursiveASTVisitor<RsrccVisitor> {
public:
  class Register {
  public:
    static std::array<bool, 31 - 5> usedRegs; // TODO: make this automatic
    int reg;

  public:
    static const Register ESP;
    static const Register EBP;
    static const Register EAX;

    Register() : reg(-1) { allocReg(); }
    Register(int reg) : reg(reg) {}
    bool allocReg() {
      if (isAllocated()) {
        return true;
      }
      // for(auto i : usedRegs) {
      //   std::cout << i << " ";
      // }endl(std::cout);
      auto nextReg = std::find(usedRegs.begin(), usedRegs.end(), false);
      if (nextReg == usedRegs.end())
        return false;
      reg = nextReg - usedRegs.begin() + RESERVED_REGS;
      // std::cout << "Found reg " << nextReg - usedRegs.begin() << " free" <<
      // std::endl;
      usedRegs[reg - RESERVED_REGS] = true;
      // std::cout << "Allocated reg " << reg << std::endl;
      return true;
    }
    void deallocReg() {
      if (!isAllocated())
        return;
      // std::cout << "Deallocating reg " << reg << std::endl;
      usedRegs[reg - RESERVED_REGS] = false;
      reg = -1;
    }
    bool isAllocated() const { return reg != -1; }
    static bool isUsed(int reg) {
      if (reg >= RESERVED_REGS && reg < MAX_REGS)
        return usedRegs[reg - RESERVED_REGS];
      return false;
    }
    static bool anyRegAvailable() {
      return std::find(usedRegs.begin(), usedRegs.end(), false) !=
             usedRegs.end();
    }
    operator int() const { return reg; }
    ~Register() {
      if (isAllocated())
        deallocReg();
    }

    std::string toString() const;

    Register(const Register &other) = delete;
    Register &operator=(const Register &other) = delete;
    Register &operator=(const Register &&other) = delete;
    Register(Register &&other) {
      reg = other.reg;
      other.reg = -1;
    }
    Register &operator=(Register &&other) {
      reg = other.reg;
      other.reg = -1;
      return *this;
    }
  };

  struct Location {
    std::shared_ptr<Register> reg;
    int stackOffset = -1;
    int indirectionLevel = 0;  // TODO 0 for rvalue, 1 for lvalue, 2 for *lvalue

    bool isReg() const { return reg->isAllocated(); }
    bool isStack() const { return stackOffset != -1; }
    bool isAllocated() const { return isReg() || isStack(); }

    std::string toString() const;
    std::string toRegString() const;
    std::string toAddrString() const;
    std::string toStackString() const;

    Location() : reg(std::make_shared<Register>(-1)) {}
    Location(int stackOffset)
        : reg(std::make_shared<Register>(-1)), stackOffset(stackOffset) {}
    Location(std::shared_ptr<Register> reg) : reg(reg) {}
    static Location INVALID;
  };

  struct SymTabEntry {
    std::string name;
    std::string label;
    RsrccVisitor::Location location;
  };

  bool DEBUG = true;

  static constexpr int RESERVED_REGS = 5;
  static constexpr int MAX_REGS = 31;
  static constexpr int STACK_BEGIN = 4096;
  static constexpr int STACK_SIZE = 4096;
  static constexpr std::string_view ENTRY_POINT = "main0";
  int currentlyAllocatedRegs = RESERVED_REGS;
  int currentStackOffset = 0;
  int currentLabel = 0;
  int currentFuncTotalLocals = 0;
  int currentFuncAllocatedLocals = 0;
  int currentFuncArgs = 0;

  explicit RsrccVisitor(clang::ASTContext *Context) : Context(Context) {}

  bool VisitTranslationUnitDecl(clang::TranslationUnitDecl *decl);

private:
  clang::ASTContext *Context;

  std::unordered_map<std::string, SymTabEntry> symTab;

  void debug(std::string);

  void call(std::string funcName);

  Location allocateLoc();

  bool evaluateFunctionDecl(clang::FunctionDecl *decl);
  Location evaluateCallExpr(clang::CallExpr *expr);
  Location evaluateStmt(clang::Stmt *stmt);
  Location evaluateExpression(clang::Expr *expr);
  Location evaluateExpressionLvalue(clang::Expr *expr);
  Location evaluateDeclRefExpr(clang::DeclRefExpr *expr);
  Location evaluateIntegerLiteral(clang::IntegerLiteral *expr);
  Location evaluateVarDecl(clang::VarDecl *decl);
  Location evaluateParmVarDecl(clang::ParmVarDecl *decl);
  Location evaluateParmVarDecl(clang::ParmVarDecl *decl, int stackOffset);
  Location evaluateReturnStmt(clang::ReturnStmt *stmt);
  Location evaluateIfStmt(clang::IfStmt *stmt);
  Location evaluateWhileStmt(clang::WhileStmt *stmt);
  Location evaluateForStmt(clang::ForStmt *stmt);
  Location evaluateBinaryOperator(clang::BinaryOperator *op);
  Location evaluateUnaryOperator(clang::UnaryOperator *op);
  Location evaluateCompute(clang::BinaryOperator *op);
  std::string compareHelper(std::string_view opStr);
  Location evaluateCompare(clang::BinaryOperator *op);
  Location evaluateAssign(clang::BinaryOperator *op);
  Location evaluateImplicitCastExpr(clang::ImplicitCastExpr *expr);
};