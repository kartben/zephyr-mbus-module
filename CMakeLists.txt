# Based on the excellent https://github.com/yashi/module-sample/
if(CONFIG_MBUS)
project(mbus)
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

## https://stackoverflow.com/questions/70747892/adding-external-library-to-zephyr
# FetchContent_Declare(
#         my-lib
#         GIT_REPOSITORY git@github.com:<HostRepo>/<repoName>.git
#         GIT_TAG v0.1.7
# )
# FetchContent_MakeAvailable(my-lib)
# target_link_libraries(app PRIVATE my-lib)

zephyr_interface_library_named(mbus)
zephyr_sources(
  libmbus/mbus/mbus-protocol.c
  libmbus/mbus/mbus-protocol-aux.c
  src/module/mbus-serial.c
  src/module/mbus-master.c
  )
zephyr_include_directories(libmbus)

file(CONFIGURE OUTPUT .gitignore
  NEWLINE_STYLE UNIX
  CONTENT "*")

endif()
