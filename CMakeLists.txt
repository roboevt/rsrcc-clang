set(LLVM_LINK_COMPONENTS support)

add_clang_executable(rsrcc
  Rsrcc.cpp
  main.cpp
  )
target_link_libraries(rsrcc
  PRIVATE
  clangAST
  clangASTMatchers
  clangBasic
  clangFrontend
  clangSerialization
  clangTooling
  )