include_guard(GLOBAL)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(TOOLCHAIN_PREFIX arm-none-eabi-)

find_program(CMAKE_C_COMPILER NAMES ${TOOLCHAIN_PREFIX}gcc REQUIRED)
find_program(CMAKE_CXX_COMPILER NAMES ${TOOLCHAIN_PREFIX}g++ REQUIRED)
set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})
find_program(CMAKE_LINKER NAMES ${TOOLCHAIN_PREFIX}ld REQUIRED)
find_program(CMAKE_OBJCOPY NAMES ${TOOLCHAIN_PREFIX}objcopy REQUIRED)
find_program(CMAKE_OBJDUMP NAMES ${TOOLCHAIN_PREFIX}objdump REQUIRED)
find_program(CMAKE_READELF NAMES ${TOOLCHAIN_PREFIX}readelf REQUIRED)
find_program(CMAKE_SIZE NAMES ${TOOLCHAIN_PREFIX}size REQUIRED)

set(LCA039_ARCH_FLAGS "-mcpu=cortex-m0 -mthumb -mfloat-abi=soft")
set(LCA039_ARCH_OPTIONS
    -mcpu=cortex-m0
    -mthumb
    -mfloat-abi=soft
    CACHE INTERNAL "LCA039 architecture options"
)

set(CMAKE_C_FLAGS_INIT
    "${LCA039_ARCH_FLAGS} -Wall -ffunction-sections -fdata-sections"
)
set(CMAKE_CXX_FLAGS_INIT
    "${LCA039_ARCH_FLAGS} -Wall -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions -fno-threadsafe-statics"
)
set(CMAKE_ASM_FLAGS_INIT
    "${LCA039_ARCH_FLAGS} -x assembler-with-cpp -ffunction-sections -fdata-sections"
)

set(CMAKE_C_FLAGS_DEBUG_INIT "-Og -g3 -gdwarf-4")
set(CMAKE_C_FLAGS_RELWITHDEBINFO_INIT "-O2 -g3 -gdwarf-4")

set(CMAKE_CXX_FLAGS_DEBUG_INIT "-Og -g3 -gdwarf-4")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO_INIT "-O2 -g3 -gdwarf-4")

set(CMAKE_ASM_FLAGS_DEBUG_INIT "-g3 -gdwarf-4")
set(CMAKE_ASM_FLAGS_RELWITHDEBINFO_INIT "-g3 -gdwarf-4")
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_C_FLAGS_RELEASE "-Os -g0 -DNDEBUG"
        CACHE STRING "Flags used by the C compiler during Release builds." FORCE)
    set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0 -DNDEBUG"
        CACHE STRING "Flags used by the CXX compiler during Release builds." FORCE)
    set(CMAKE_ASM_FLAGS_RELEASE "-g0 -DNDEBUG"
        CACHE STRING "Flags used by the ASM compiler during Release builds." FORCE)
endif()
