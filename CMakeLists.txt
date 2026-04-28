cmake_minimum_required(VERSION 3.10)


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

# ──────────────────────────────────────────────────────────────
# Auto-increment version on each build
# ──────────────────────────────────────────────────────────────
set(VERSION_FILE "${CMAKE_SOURCE_DIR}/.version")

# Manually set major, minor, and bugfix versions (you can change these)
set(MAJOR_VERSION 1)
set(MINOR_VERSION 8)
set(BUGFIX_VERSION 0)

if(EXISTS ${VERSION_FILE})
    file(READ ${VERSION_FILE} CURRENT_VERSION)
    string(STRIP "${CURRENT_VERSION}" CURRENT_VERSION)
    # Extract current version components
    string(REPLACE "." ";" VERSION_LIST ${CURRENT_VERSION})
    list(LENGTH VERSION_LIST LIST_LENGTH)
    if(LIST_LENGTH EQUAL 4)
        list(GET VERSION_LIST 0 STORED_MAJOR)
        list(GET VERSION_LIST 1 STORED_MINOR)
        list(GET VERSION_LIST 2 STORED_BUGFIX)
        list(GET VERSION_LIST 3 STORED_BUILD)
        
        # Check if major, minor, or bugfix version changed
        if(NOT (${STORED_MAJOR} EQUAL ${MAJOR_VERSION} AND ${STORED_MINOR} EQUAL ${MINOR_VERSION} AND ${STORED_BUGFIX} EQUAL ${BUGFIX_VERSION}))
            set(BUILD_VERSION 0)  # Reset build when major/minor/bugfix changes
            message(STATUS "Version changed from ${STORED_MAJOR}.${STORED_MINOR}.${STORED_BUGFIX}.${STORED_BUILD} to ${MAJOR_VERSION}.${MINOR_VERSION}.${BUGFIX_VERSION}, resetting build to 0")
        else()
            set(BUILD_VERSION ${STORED_BUILD})  # Continue with existing build
        endif()
    else()
        set(BUILD_VERSION 0)  # Default build version
    endif()
else()
    set(BUILD_VERSION 0)  # Default build version
endif()

# Increment build version only for RELEASE builds
IF(CMAKE_BUILD_TYPE MATCHES RELEASE)
    math(EXPR BUILD_VERSION "${BUILD_VERSION} + 1")
    message(STATUS "Incrementing build version for RELEASE build")
ELSE()
    message(STATUS "Non-release build - keeping current build version")
ENDIF()

# Create new version string
set(__APP__VERSION__ "${MAJOR_VERSION}.${MINOR_VERSION}.${BUGFIX_VERSION}.${BUILD_VERSION}")

message(STATUS "Building version: ${__APP__VERSION__}")
message(STATUS "To change major/minor/bugfix versions, edit MAJOR_VERSION, MINOR_VERSION, and BUGFIX_VERSION above")

SET (__APP__VERSION__ "${__APP__VERSION__}")
add_definitions( -D__APP__VERSION__="${__APP__VERSION__}")
project(DE_TRACKER VERSION "${__APP__VERSION__}")



SET (CMAKE_C_COMPILER /usr/bin/gcc)
SET (CMAKE_CXX_COMPILER /usr/bin/g++)

#C++ Version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
find_package(Threads REQUIRED)

# Find OpenCV package
find_package(OpenCV 4.5 REQUIRED )
message(STATUS "OpenCV_INCLUDE_DIRS found: ${OpenCV_INCLUDE_DIRS}")


# Define an option for user control over DETAILED_DEBUG
option(DDEBUG "Detailed Debug" OFF) # Default is OFF


include_directories( ${OpenCV_INCLUDE_DIRS} )



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


# Gather all source files
file(GLOB folder_src "./src/*.cpp")
file(GLOB folder_tracker "./src/tracker/*.cpp")
file(GLOB folder_uavos "./src/de_common/*.cpp")
file(GLOB folder_uavos1 "./src/de_common/de_databus/*.cpp")
file(GLOB folder_uavos2 "./src/de_common/helpers/*.cpp")
set(files ${folder_src} ${folder_uavos} ${folder_uavos1} ${folder_uavos2} ${folder_tracker})

# Add the executable target and link its libraries
add_executable(OUTPUT_BINARY ${files})
target_link_libraries(OUTPUT_BINARY Threads::Threads ${OpenCV_LIBS})

# Conditionally define DDEBUG based on the DDEBUG option
target_compile_definitions(OUTPUT_BINARY PRIVATE
    $<$<BOOL:${DDEBUG}>:DDEBUG>
)

set(OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
file(MAKE_DIRECTORY ${OUTPUT_DIRECTORY})
SET(EXECUTABLE_OUTPUT_PATH ${OUTPUT_DIRECTORY})


configure_file(de_tracker.config.module.json ${OUTPUT_DIRECTORY}/de_tracker.config.module.json COPYONLY)

set_target_properties( OUTPUT_BINARY 
                PROPERTIES 
                    OUTPUT_NAME "de_tracker"
                )

# Highlight if DDEBUG or TEST_MODE_NO_HAILO_LINK are enabled
if (DDEBUG)
    message(STATUS "${Red}Option DDEBUG is ENABLED.${ColourReset}")
endif()

message ("${Yellow}=========================================================================${ColourReset}")
message (STATUS "Version              : ${__APP__VERSION__}")

# ──────────────────────────────────────────────────────────────
# Build success script and version management
# ──────────────────────────────────────────────────────────────
set(BUILD_SUCCESS_SCRIPT "${CMAKE_BINARY_DIR}/build_success.cmake")
file(WRITE "${BUILD_SUCCESS_SCRIPT}" "")
file(APPEND "${BUILD_SUCCESS_SCRIPT}" "execute_process(COMMAND \"${CMAKE_COMMAND}\" -E echo \"\")\n")
file(APPEND "${BUILD_SUCCESS_SCRIPT}" "execute_process(COMMAND \"${CMAKE_COMMAND}\" -E cmake_echo_color --green --bold \"=========================================================================\")\n")
file(APPEND "${BUILD_SUCCESS_SCRIPT}" "execute_process(COMMAND \"${CMAKE_COMMAND}\" -E cmake_echo_color --green --bold \"BUILD COMPLETED SUCCESSFULLY\")\n")
file(APPEND "${BUILD_SUCCESS_SCRIPT}" "execute_process(COMMAND \"${CMAKE_COMMAND}\" -E cmake_echo_color --cyan --bold \"Version: ${__APP__VERSION__}\")\n")
file(APPEND "${BUILD_SUCCESS_SCRIPT}" "execute_process(COMMAND \"${CMAKE_COMMAND}\" -E cmake_echo_color --cyan --bold \"Build Number: ${BUILD_VERSION}\")\n")
file(APPEND "${BUILD_SUCCESS_SCRIPT}" "execute_process(COMMAND \"${CMAKE_COMMAND}\" -E cmake_echo_color --green --bold \"=========================================================================\")\n")
file(APPEND "${BUILD_SUCCESS_SCRIPT}" "execute_process(COMMAND \"${CMAKE_COMMAND}\" -E echo \"\")\n")

# Write version file directly for RELEASE builds
IF(CMAKE_BUILD_TYPE MATCHES RELEASE)
    file(WRITE "${VERSION_FILE}" "${__APP__VERSION__}")
    message(STATUS "Version written to ${VERSION_FILE}: ${__APP__VERSION__}")
ENDIF()