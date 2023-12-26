#include "clang/AST/RecursiveASTVisitor.h"
#include "clang/Tooling/Tooling.h"

#include <unordered_map>

class RsrccVisitor : public clang::RecursiveASTVisitor<RsrccVisitor> {
public:
  struct Location {
    int regNum, stackOffset;

    constexpr bool isReg() const { return regNum != -1; }
    constexpr bool isStack() const { return stackOffset != -1; }
    constexpr bool isAllocated() const { return isReg() || isStack(); }

    std::string toString() const;

    constexpr Location() : regNum(-1), stackOffset(-1) {}
    constexpr Location(int regNum, int stackOffset)
        : regNum(regNum), stackOffset(stackOffset) {}

    static constexpr int esp = 1;
    static constexpr int ebp = 2;
    static constexpr int eax = 3;
    static constexpr int rtemp = 4;
    static const Location ESP;
    static const Location EBP;
    static const Location EAX;
    static const Location RTEMP;
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

  explicit RsrccVisitor(clang::ASTContext *Context) : Context(Context) {}

  //   bool VisitCXXRecordDecl(clang::CXXRecordDecl *Declaration);

  //   bool VisitFunctionDecl(clang::FunctionDecl *Declaration);

//   bool VisitVarDecl(clang::VarDecl *Declaration);

  //   bool VisitDecl(clang::Decl *Declaration);

  //   bool VisitBinaryOperator(clang::BinaryOperator *op);

//   bool VisitReturnStmt(clang::ReturnStmt *stmt);

  bool VisitTranslationUnitDecl(clang::TranslationUnitDecl *decl);
//   bool VisitFunctionDecl(clang::FunctionDecl *decl);

private:
  clang::ASTContext *Context;

  std::unordered_map<std::string, SymTabEntry> symTab;

  void debug(std::string);

  void call(std::string funcName);

  //   bool visitAssign(clang::BinaryOperator *op);
  //   bool visitCompute(clang::BinaryOperator *op);

  Location allocateLoc();

  bool evaluateFunctionDecl(clang::FunctionDecl *decl);
  Location evaluateCallExpr(clang::CallExpr *expr);
  Location evaluateExpression(clang::Expr *expr);
  Location evaluateDeclRefExpr(clang::DeclRefExpr *expr);
  Location evaluateIntegerLiteral(clang::IntegerLiteral *expr);
  Location evaluateVarDecl(clang::VarDecl *decl);
  Location evaluateParmVarDecl(clang::ParmVarDecl *decl);
  Location evaluateReturnStmt(clang::ReturnStmt *stmt);
  Location evaluateBinaryOperator(clang::BinaryOperator *op);
  Location evaluateCompute(clang::BinaryOperator *op);
  Location evaluateAssign(clang::BinaryOperator *op);
};