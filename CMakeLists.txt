# Based on the excellent https://github.com/yashi/module-sample/
if(CONFIG_LIBMBUS)
project(mbus)
cmake_minimum_required(VERSION 3.20.0)

#find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

## https://stackoverflow.com/questions/70747892/adding-external-library-to-zephyr
# FetchContent_Declare(
#         my-lib
#         GIT_REPOSITORY git@github.com:<HostRepo>/<repoName>.git
#         GIT_TAG v0.1.7
# )
# FetchContent_MakeAvailable(my-lib)
# target_link_libraries(app PRIVATE my-lib)

#target_compile_options(libmbus PRIVATE -Wall -Wextra)
#set_target_properties(libmbus PROPERTIES C_STANDARD 11)
#zephyr_library_import(libmbus ${CMAKE_CURRENT_SOURCE_DIR}/libmbus.a)
# See https://github.com/zephyrproject-rtos/zephyr/issues/31613

add_library(libmbus INTERFACE)
#set_target_properties(libmbus PROPERTIES C_STANDARD 11)
#target_compile_options(libmbus PRIVATE -Wall -Wextra)
#target_include_directories(libmbus PUBLIC libmbus/mbus)
zephyr_library()
zephyr_sources(
  libmbus/mbus/mbus-protocol.c
  libmbus/mbus/mbus-protocol-aux.c
  libmbus/mbus/mbus-serial.c
  )

file(CONFIGURE OUTPUT .gitignore
  NEWLINE_STYLE UNIX
  CONTENT "*")

endif()
