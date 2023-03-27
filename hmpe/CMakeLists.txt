project (HMPE)

set(CMAKE_CXX_STANDARD 17)

add_library(hmpe SHARED world.cpp world.hpp utils.hpp body.cpp body.hpp shape.hpp shape.cpp collision.hpp collision.cpp solver.hpp solver.hpp solver.cpp)

target_compile_definitions(hmpe PRIVATE -DHMPE_DLL_EXPORT)
