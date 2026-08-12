cmake_minimum_required(VERSION 3.16)
project(orbbec_rgbd_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(orbbec_rgbd_bridge orbbec_rgbd_bridge.cpp)

# OrbbecSDK distributions vary: some install an OrbbecSDKConfig.cmake package,
# while source/build trees often only expose include/ and libOrbbecSDK.so.
# Support both forms. If OrbbecSDK_DIR is accidentally passed as the .so file
# path, treat it as ORBBECSDK_LIBRARY for convenience.
if(DEFINED OrbbecSDK_DIR AND EXISTS "${OrbbecSDK_DIR}" AND NOT IS_DIRECTORY "${OrbbecSDK_DIR}")
    set(ORBBECSDK_LIBRARY "${OrbbecSDK_DIR}" CACHE FILEPATH "Path to libOrbbecSDK.so" FORCE)
    unset(OrbbecSDK_DIR CACHE)
endif()

find_package(OrbbecSDK CONFIG QUIET)

if(TARGET OrbbecSDK::OrbbecSDK)
    target_link_libraries(orbbec_rgbd_bridge PRIVATE OrbbecSDK::OrbbecSDK)
else()
    set(ORBBECSDK_ROOT "" CACHE PATH "Path to the OrbbecSDK v2 source/install root")
    set(ORBBECSDK_INCLUDE_DIR "" CACHE PATH "Directory containing libobsensor/ObSensor.hpp")
    set(ORBBECSDK_GENERATED_INCLUDE_DIR "" CACHE PATH "Directory containing generated OrbbecSDK headers such as Export.h")
    set(ORBBECSDK_LIBRARY "" CACHE FILEPATH "Path to libOrbbecSDK.so")

    if(ORBBECSDK_INCLUDE_DIR AND EXISTS "${ORBBECSDK_INCLUDE_DIR}/ObSensor.hpp")
        get_filename_component(_orbbecsdk_include_leaf "${ORBBECSDK_INCLUDE_DIR}" NAME)
        if(_orbbecsdk_include_leaf STREQUAL "libobsensor")
            get_filename_component(_orbbecsdk_include_parent "${ORBBECSDK_INCLUDE_DIR}" DIRECTORY)
            set(ORBBECSDK_INCLUDE_DIR "${_orbbecsdk_include_parent}" CACHE PATH "Directory containing libobsensor/ObSensor.hpp" FORCE)
        endif()
    endif()

    set(_ORBBECSDK_INCLUDE_HINTS)
    if(ORBBECSDK_ROOT)
        list(APPEND _ORBBECSDK_INCLUDE_HINTS
            "${ORBBECSDK_ROOT}"
            "${ORBBECSDK_ROOT}/include"
            "${ORBBECSDK_ROOT}/install/include"
            "${ORBBECSDK_ROOT}/sdk/include"
            "${ORBBECSDK_ROOT}/SDK/include"
            "${ORBBECSDK_ROOT}/build/linux_arm64/include"
            "${ORBBECSDK_ROOT}/build/linux_arm64/install/include"
            "${ORBBECSDK_ROOT}/build/linux_x64/include"
            "${ORBBECSDK_ROOT}/build/linux_x64/install/include"
        )
    endif()

    if(ORBBECSDK_LIBRARY)
        get_filename_component(_orbbecsdk_library_dir "${ORBBECSDK_LIBRARY}" DIRECTORY)
        get_filename_component(_orbbecsdk_library_parent "${_orbbecsdk_library_dir}" DIRECTORY)
        get_filename_component(_orbbecsdk_library_grandparent "${_orbbecsdk_library_parent}" DIRECTORY)
        get_filename_component(_orbbecsdk_library_root_guess "${_orbbecsdk_library_grandparent}" DIRECTORY)
        list(APPEND _ORBBECSDK_INCLUDE_HINTS
            "${_orbbecsdk_library_parent}/include"
            "${_orbbecsdk_library_parent}/install/include"
            "${_orbbecsdk_library_grandparent}/include"
            "${_orbbecsdk_library_grandparent}/install/include"
            "${_orbbecsdk_library_root_guess}/include"
            "${_orbbecsdk_library_root_guess}/install/include"
        )
    endif()

    set(_ORBBECSDK_GENERATED_INCLUDE_HINTS)
    if(ORBBECSDK_ROOT)
        list(APPEND _ORBBECSDK_GENERATED_INCLUDE_HINTS
            "${ORBBECSDK_ROOT}/build/src/generated"
            "${ORBBECSDK_ROOT}/build/linux_arm64/src/generated"
            "${ORBBECSDK_ROOT}/build/linux_x64/src/generated"
            "${ORBBECSDK_ROOT}/src/generated"
        )
    endif()
    if(ORBBECSDK_LIBRARY)
        list(APPEND _ORBBECSDK_GENERATED_INCLUDE_HINTS
            "${_orbbecsdk_library_parent}/src/generated"
            "${_orbbecsdk_library_grandparent}/src/generated"
            "${_orbbecsdk_library_root_guess}/build/src/generated"
            "${_orbbecsdk_library_root_guess}/src/generated"
        )
    endif()

    if(NOT ORBBECSDK_INCLUDE_DIR)
        find_path(
            ORBBECSDK_INCLUDE_DIR
            NAMES libobsensor/ObSensor.hpp
            HINTS ${_ORBBECSDK_INCLUDE_HINTS}
        )
    endif()

    if(NOT ORBBECSDK_GENERATED_INCLUDE_DIR)
        find_path(
            ORBBECSDK_GENERATED_INCLUDE_DIR
            NAMES Export.h
            HINTS ${_ORBBECSDK_GENERATED_INCLUDE_HINTS}
        )
    endif()

    if(NOT ORBBECSDK_LIBRARY)
        find_library(
            ORBBECSDK_LIBRARY
            NAMES OrbbecSDK libOrbbecSDK
            HINTS
                "${ORBBECSDK_ROOT}/lib"
                "${ORBBECSDK_ROOT}/install/lib"
                "${ORBBECSDK_ROOT}/build/linux_arm64/lib"
                "${ORBBECSDK_ROOT}/build/linux_x64/lib"
        )
    endif()

    if(NOT ORBBECSDK_INCLUDE_DIR OR NOT ORBBECSDK_LIBRARY)
        message(STATUS "OrbbecSDK search summary:")
        message(STATUS "  ORBBECSDK_ROOT=${ORBBECSDK_ROOT}")
        message(STATUS "  ORBBECSDK_INCLUDE_DIR=${ORBBECSDK_INCLUDE_DIR}")
        message(STATUS "  ORBBECSDK_GENERATED_INCLUDE_DIR=${ORBBECSDK_GENERATED_INCLUDE_DIR}")
        message(STATUS "  ORBBECSDK_LIBRARY=${ORBBECSDK_LIBRARY}")
        message(STATUS "  include hints=${_ORBBECSDK_INCLUDE_HINTS}")
        message(STATUS "  generated include hints=${_ORBBECSDK_GENERATED_INCLUDE_HINTS}")
        message(FATAL_ERROR
            "Could not find OrbbecSDK v2.\n"
            "Pass either:\n"
            "  -DOrbbecSDK_DIR=/path/to/directory/containing/OrbbecSDKConfig.cmake\n"
            "or:\n"
            "  -DORBBECSDK_ROOT=/path/to/OrbbecSDK_v2 "
            "-DORBBECSDK_LIBRARY=/path/to/libOrbbecSDK.so "
            "-DORBBECSDK_INCLUDE_DIR=/path/to/include "
            "[-DORBBECSDK_GENERATED_INCLUDE_DIR=/path/to/build/src/generated]"
        )
    endif()

    if(NOT EXISTS "${ORBBECSDK_INCLUDE_DIR}/libobsensor/ObSensor.hpp")
        message(FATAL_ERROR
            "ORBBECSDK_INCLUDE_DIR must point to the directory containing "
            "libobsensor/ObSensor.hpp.\n"
            "Current value: ${ORBBECSDK_INCLUDE_DIR}"
        )
    endif()

    if(NOT EXISTS "${ORBBECSDK_INCLUDE_DIR}/Export.h"
        AND NOT EXISTS "${ORBBECSDK_INCLUDE_DIR}/libobsensor/h/Export.h"
        AND NOT EXISTS "${ORBBECSDK_GENERATED_INCLUDE_DIR}/Export.h")
        message(FATAL_ERROR
            "Could not find generated OrbbecSDK header Export.h.\n"
            "Pass -DORBBECSDK_GENERATED_INCLUDE_DIR=/path/to/build/src/generated.\n"
            "Current value: ${ORBBECSDK_GENERATED_INCLUDE_DIR}"
        )
    endif()

    message(STATUS "Using OrbbecSDK include dir: ${ORBBECSDK_INCLUDE_DIR}")
    if(ORBBECSDK_GENERATED_INCLUDE_DIR)
        message(STATUS "Using OrbbecSDK generated include dir: ${ORBBECSDK_GENERATED_INCLUDE_DIR}")
    endif()
    message(STATUS "Using OrbbecSDK library: ${ORBBECSDK_LIBRARY}")

    set(_ORBBECSDK_TARGET_INCLUDE_DIRS "${ORBBECSDK_INCLUDE_DIR}")
    if(ORBBECSDK_GENERATED_INCLUDE_DIR)
        list(APPEND _ORBBECSDK_TARGET_INCLUDE_DIRS "${ORBBECSDK_GENERATED_INCLUDE_DIR}")
    endif()
    target_include_directories(orbbec_rgbd_bridge PRIVATE ${_ORBBECSDK_TARGET_INCLUDE_DIRS})
    target_link_libraries(orbbec_rgbd_bridge PRIVATE "${ORBBECSDK_LIBRARY}")

    get_filename_component(ORBBECSDK_LIBRARY_DIR "${ORBBECSDK_LIBRARY}" DIRECTORY)
    set_target_properties(orbbec_rgbd_bridge PROPERTIES
        BUILD_RPATH "${ORBBECSDK_LIBRARY_DIR}"
        INSTALL_RPATH "${ORBBECSDK_LIBRARY_DIR}"
    )
endif()
