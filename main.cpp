#include "clang/AST/ASTConsumer.h"
#include "clang/AST/RecursiveASTVisitor.h"
#include "clang/Frontend/CompilerInstance.h"
#include "clang/Frontend/FrontendAction.h"
#include "clang/Tooling/Tooling.h"

#include "clang/Tooling/CommonOptionsParser.h"
#include "llvm/Support/CommandLine.h"

#include "Rsrcc.h"

using namespace clang;
using namespace clang::tooling;
using namespace llvm;

class RsrccConsumer : public clang::ASTConsumer {
public:
  explicit RsrccConsumer(ASTContext *Context)
    : Visitor(Context) {}

  virtual void HandleTranslationUnit(clang::ASTContext &Context) override {
    Visitor.TraverseDecl(Context.getTranslationUnitDecl());
  }
private:
  RsrccVisitor Visitor;
};

class RsrccAction : public clang::ASTFrontendAction {
public:
  virtual std::unique_ptr<clang::ASTConsumer> CreateASTConsumer(
    clang::CompilerInstance &Compiler, llvm::StringRef InFile) override {

    Compiler.getDiagnostics().setClient(new IgnoringDiagConsumer);

    // Suppress all warnings
    Compiler.getDiagnostics().setWarningsAsErrors(false);
    Compiler.getDiagnostics().setSuppressAllDiagnostics(true);

    return std::make_unique<RsrccConsumer>(&Compiler.getASTContext());
  }
};

static llvm::cl::OptionCategory MyToolCategory("my-tool options");
static cl::extrahelp CommonHelp(CommonOptionsParser::HelpMessage);

int main(int argc, const char **argv) {
  std::vector<const char*> new_argv(argv, argv + argc);
  new_argv.push_back("--");  // Remove default compiler arguments (that don't work for some reason)
  argv = new_argv.data();
  argc = new_argv.size();
  auto expectedOption = CommonOptionsParser::create(argc, argv, MyToolCategory);
  if (!expectedOption) {
    // Fail gracefully for unsupported options.
    llvm::errs() << expectedOption.takeError();
    return 1;
  }
  
  CommonOptionsParser& options = expectedOption.get();
  
  ClangTool tool(options.getCompilations(), options.getSourcePathList());

  return tool.run(newFrontendActionFactory<RsrccAction>().get());
}