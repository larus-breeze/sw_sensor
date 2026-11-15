# Host test: logging pause and resume in uSD_handler.cpp (#302).
# Linux only: the test maps the flash staging area at its fixed address.
#
#   cmake -S sw_stm32/tests/logging_resume -B build/logging_resume_test
#   cmake --build build/logging_resume_test
#   ctest --test-dir build/logging_resume_test --output-on-failure
cmake_minimum_required(VERSION 3.16)
project(logging_resume_test CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS ON) # gnu++14, as in the firmware build

get_filename_component(SW_STM32 "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(LIB "${SW_STM32}/lib")

# Compile a copy of uSD_handler.cpp: its Cortex-M instructions (barriers on
# the firmware-update jump path, the IPSR read in the crash handler; none of
# them reached here) don't assemble on the host, and from the build directory
# the stub headers take precedence over the real headers next to the
# original file.
set(HANDLER "${CMAKE_CURRENT_BINARY_DIR}/uSD_handler_host.cpp")
file(READ "${SW_STM32}/Communication/uSD_handler.cpp" handler_source)
string(REGEX REPLACE "__asm volatile[^;]*;" ";" handler_source "${handler_source}")
string(REPLACE "__get_IPSR()" "host_get_IPSR()" handler_source "${handler_source}")
file(WRITE "${HANDLER}.tmp" "${handler_source}")
configure_file("${HANDLER}.tmp" "${HANDLER}" COPYONLY)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${SW_STM32}/Communication/uSD_handler.cpp")
# The real headers from Communication/ are copied for the same reason: their
# own includes (flexible_log_file_implementation.h, ...) must find the stubs.
foreach(header uSD_handler.h uSD_helpers.h fatfs_access.h)
  configure_file("${SW_STM32}/Communication/${header}" "${CMAKE_CURRENT_BINARY_DIR}/include/${header}" COPYONLY)
endforeach()

add_executable(logging_resume_test
  logging_resume_test.cpp
  "${HANDLER}"
  "${LIB}/Output_Formatter/ascii_support.cpp")

target_include_directories(logging_resume_test PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/stubs" # before the real headers
  "${CMAKE_CURRENT_BINARY_DIR}/include" # copies of uSD_handler.h, uSD_helpers.h, fatfs_access.h
  "${LIB}/tests/stubs"
  "${LIB}/Generic_Algorithms"
  "${LIB}/NAV_Algorithms"
  "${LIB}/Output_Formatter"
  "${LIB}/Persistent_Data"
  "${LIB}/ARM_math"
  "${SW_STM32}/Core/Inc")             # emergency.h; last, so the host stubs win
target_compile_options(logging_resume_test PRIVATE -Wall)
# the fixed flash addresses are 32 bit on the target
set_source_files_properties("${HANDLER}" PROPERTIES COMPILE_OPTIONS -Wno-int-to-pointer-cast)
target_link_libraries(logging_resume_test PRIVATE m)

enable_testing()
foreach(scenario resume_after_stop resume_at_takeoff retry_when_sd_full)
  add_test(NAME ${scenario} COMMAND logging_resume_test ${scenario})
endforeach()
