if(DEFINED LCA039_COMMUNICATION_VOFA_ENABLED AND NOT LCA039_COMMUNICATION_VOFA_ENABLED)
    return()
endif()

add_library(lca039_vofa STATIC)

target_sources(lca039_vofa PRIVATE
    Core/VOFA_Buffer.cpp
    Core/VOFA_Sender.cpp)

target_include_directories(lca039_vofa PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Public
    ${CMAKE_CURRENT_SOURCE_DIR}/Core
    ${CMAKE_SOURCE_DIR}/Platform/Communication)

target_compile_options(lca039_vofa PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wall;-Wextra;-Wpedantic>)
