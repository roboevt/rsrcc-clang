#include "clang/AST/RecursiveASTVisitor.h"
#include "clang/Tooling/Tooling.h"

#include <unordered_map>

class RsrccVisitor : public clang::RecursiveASTVisitor<RsrccVisitor> {
public:
  class Register {
    static int nextReg;
    int reg;

  public:
    Register() : reg(-1) {}
    Register(int reg) : reg(reg) {}
    bool allocate() {
      if (nextReg >= RESERVED_REGS) {
        reg = nextReg--;
        return true;
      }
      return false;
    }
    bool isAllocated() const { return reg != -1; }
    static bool isUsed(int reg) { return reg > nextReg; }
    static bool anyRegAvailable() { return nextReg >= RESERVED_REGS; }
    operator int() const { return reg; }
    ~Register() {
      if (reg != -1)
        nextReg++;
    }
    Register(const Register &other) = delete;
    Register &operator=(const Register &other) = delete;
    Register &operator=(const Register &&other) = delete;
    Register(Register &&other) = default;
    Register &operator=(Register &&other) = default;
  };
  struct Location {
    std::shared_ptr<Register> reg;
    int stackOffset;

    bool isReg() const { return reg->isAllocated(); }
    bool isStack() const { return stackOffset != -1; }
    bool isAllocated() const { return isReg() || isStack(); }

    std::string toString() const;

    Location() : reg(std::make_shared<Register>()), stackOffset(-1) {}
    Location(int reg, int stackOffset)
        : reg(std::make_shared<Register>(reg)), stackOffset(stackOffset) {}

    static constexpr int esp = 1;
    static constexpr int ebp = 2;
    static constexpr int eax = 3;
    static constexpr int rtemp = 4;
    static const Location ESP;
    static const Location EBP;
    static const Location EAX;
    static const Location RTEMP;
    static Location INVALID;
  };

  struct SymTabEntry {
    std::string name;
    std::string label;
    RsrccVisitor::Location location;
  };

  bool DEBUG = false;

  static constexpr int RESERVED_REGS = 5;
  static constexpr int MAX_REGS = 31;
  static constexpr int STACK_BEGIN = 4096;
  static constexpr int STACK_SIZE = 4096;
  static constexpr std::string_view ENTRY_POINT = "main0";
  int currentlyAllocatedRegs = RESERVED_REGS;
  int currentStackOffset = 0;
  int currentLabel = 0;

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
  Location evaluateDeclRefExpr(clang::DeclRefExpr *expr);
  Location evaluateIntegerLiteral(clang::IntegerLiteral *expr);
  Location evaluateVarDecl(clang::VarDecl *decl);
  Location evaluateParmVarDecl(clang::ParmVarDecl *decl);
  Location evaluateReturnStmt(clang::ReturnStmt *stmt);
  Location evaluateIfStmt(clang::IfStmt *stmt);
  Location evaluateWhileStmt(clang::WhileStmt *stmt);
  Location evaluateForStmt(clang::ForStmt *stmt);
  Location evaluateBinaryOperator(clang::BinaryOperator *op);
  Location evaluateCompute(clang::BinaryOperator *op);
  std::string compareHelper(std::string_view opStr);
  Location evaluateCompare(clang::BinaryOperator *op);
  Location evaluateAssign(clang::BinaryOperator *op);
};