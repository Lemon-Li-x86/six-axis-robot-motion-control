cmake_minimum_required(VERSION 3.16)
project(feedback_tick_order C)
enable_testing()
add_executable(feedback_tick_order_test
  feedback_tick_order_test.c
  ../../application/motor_state.c
)
target_compile_features(feedback_tick_order_test PRIVATE c_std_11)
target_compile_options(feedback_tick_order_test PRIVATE -Wall -Wextra -Werror)
target_include_directories(feedback_tick_order_test PRIVATE
  stubs ../../common ../../drivers ../../application
)
add_test(NAME feedback_tick_order_test COMMAND feedback_tick_order_test)
