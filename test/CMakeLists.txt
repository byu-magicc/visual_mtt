
# Project paths



# if(${CATKIN_FOUND})

  message(STATUS "Building Tests")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
  catkin_add_gtest(params_test params_test.cpp ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}2/src/common/params.cpp)



# else(${CATKIN_FOUND})

#   message(STATUS "GTEST not found. Adding it to package")


#   set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


#   set(GTEST_DIR ${CMAKE_CURRENT_BINARY_DIR}/googletest-download)

#   message(STATUS "${CMAKE_MODULE_PATH}")
#   message(STATUS "${CMAKE_CURRENT_BINARY_DIR}")

#   # Download and install GoogleTest
#   configure_file(
#     "${CMAKE_MODULE_PATH}/googletest.cmake.in"
#     "${GTEST_DIR}/CMakeLists.txt"
#     IMMEDIATE @ONLY)




#   execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" . WORKING_DIRECTORY ${GTEST_DIR} )
#   execute_process(COMMAND ${CMAKE_COMMAND} --build . WORKING_DIRECTORY ${GTEST_DIR})

#   # Prevent overriding the parent project's compiler/linker settings on Windows
#   set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

#   # Add googletest directly to the build. This defines the `gtest` and `gtest_main` targets. Don't add these
#   # targets to the ALL target so that `make install` will ignore googletest installation targets
#   add_subdirectory(${GTEST_DIR}/googletest-src ${GTEST_DIR}/googletest-build EXCLUDE_FROM_ALL)



#   add_executable(params_test params_test.cpp src/common/params.cpp)

#   target_link_libraries(params_test gtest_main )
# endif(${CATKIN_FOUND})

