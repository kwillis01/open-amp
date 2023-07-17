set (CMAKE_SYSTEM_PROCESSOR "arm" CACHE STRING "")
set (MACHINE                "j721e_r5" CACHE STRING "")
set (CROSS_PREFIX           "arm-none-eabi-" CACHE STRING "")

set (CMAKE_C_FLAGS          "-mcpu=cortex-r5 -g -O0 -DRPMSG_NO_IPI" CACHE STRING "")

include (cross_generic_gcc)
