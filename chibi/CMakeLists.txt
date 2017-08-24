

target_include_directories(${PROJECT_NAME} PUBLIC
  nil/include
  common/ports/ARMCMx/
  common/ports/ARMCMx/compilers/GCC/
  ./license
  common/oslib/include/
  .
)

target_sources(${PROJECT_NAME} PUBLIC

  ${CMAKE_CURRENT_SOURCE_DIR}/common/ports/ARMCMx/chcore_v6m.c
  ${CMAKE_CURRENT_SOURCE_DIR}/common/ports/ARMCMx/chcore.c

  ${CMAKE_CURRENT_SOURCE_DIR}/common/ports/ARMCMx/chcore.c

  ${CMAKE_CURRENT_SOURCE_DIR}/common/oslib/src/chheap.c
  ${CMAKE_CURRENT_SOURCE_DIR}/common/oslib/src/chmboxes.c
  ${CMAKE_CURRENT_SOURCE_DIR}/common/oslib/src/chmemcore.c
  ${CMAKE_CURRENT_SOURCE_DIR}/common/oslib/src/chmempools.c

  ${CMAKE_CURRENT_SOURCE_DIR}/nil/src/ch.c

  ${CMAKE_CURRENT_SOURCE_DIR}/common/ports/ARMCMx/compilers/GCC/chcoreasm_v6m.S

  ${CMAKE_CURRENT_SOURCE_DIR}/xpcc/event.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/xpcc/irq_wrapper.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/xpcc/semaphore.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/xpcc/util.cpp
)

# ADD_LIBRARY(chibi STATIC
#     ${SRCS}
# )
