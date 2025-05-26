cmake_minimum_required(VERSION 3.1.0)


if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(ColourBold  "${Esc}[1m")
  set(Red         "${Esc}[31m")
  set(Green       "${Esc}[32m")
  set(Yellow      "${Esc}[33m")
  set(Blue        "${Esc}[34m")
  set(Magenta     "${Esc}[35m")
  set(Cyan        "${Esc}[36m")
  set(White       "${Esc}[37m")
  set(BoldRed     "${Esc}[1;31m")
  set(BoldGreen   "${Esc}[1;32m")
  set(BoldYellow  "${Esc}[1;33m")
  set(BoldBlue    "${Esc}[1;34m")
  set(BoldMagenta "${Esc}[1;35m")
  set(BoldCyan    "${Esc}[1;36m")
  set(BoldWhite   "${Esc}[1;37m")
endif()


if ( ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR} )
message ("${Red} In-source builds not allowed! Create a build directory and run CMake from there.  ${ColourReset}")
message( FATAL_ERROR “In-source builds not allowed! Create a build directory and run CMake from there. ” )
exit(1)
endif()

SET (__APP__VERSION__ "0.2.0")
add_definitions( -D__APP__VERSION__="${__APP__VERSION__}")
project(DE_TRACKER VERSION ${__APP__VERSION__})



SET (CMAKE_C_COMPILER /usr/bin/gcc)
SET (CMAKE_CXX_COMPILER /usr/bin/g++)

#C++ Version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
find_package(OpenCV 4.5 REQUIRED )
find_package(Threads REQUIRED)


#define default build
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
SET(CMAKE_BUILD_TYPE DEBUG)
ENDIF()


message ("${Yellow}=========================================================================${ColourReset}")
message ("CMAKE_C_COMPILER: "  ${CMAKE_C_COMPILER})
message ("CMAKE_CXX_COMPILER: " ${CMAKE_CXX_COMPILER})

#deceide debug or release
IF(CMAKE_BUILD_TYPE MATCHES DEBUG)
message("CMAKE_BUILD_TYPE: ${BoldYellow} DEBUG ${ColourReset}")
ELSEIF (CMAKE_BUILD_TYPE MATCHES RELEASE)
message("CMAKE_BUILD_TYPE: ${BoldGreen} RELEASE ${ColourReset}")
ENDIF()

message("OpenCV_VERSION : ${BoldGreen} ${OpenCV_VERSION} ${ColourReset}")


# adding CMAKE_CXX_FLAGS properly
add_compile_options(
  -Wno-return-type 
  -Wno-address-of-packed-member
  $<$<CONFIG:DEBUG>:-g3>
  $<$<CONFIG:DEBUG>:-Og>
  $<$<CONFIG:RELEASE>:-O3>
)

# adding DEFINEs
add_compile_definitions(
    $<$<CONFIG:DEBUG>:DEBUG>
    $<$<CONFIG:RELEASE>:RELEASE>
)


set(OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
file(MAKE_DIRECTORY ${OUTPUT_DIRECTORY})
SET(EXECUTABLE_OUTPUT_PATH ${OUTPUT_DIRECTORY})

add_subdirectory(src)

configure_file(de_tracker.config.module.json ${OUTPUT_DIRECTORY}/de_tracker.config.module.json COPYONLY)


message ("${Yellow}=========================================================================${ColourReset}")
