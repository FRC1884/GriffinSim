cmake_minimum_required(VERSION 3.20)
project(GriffinSimHalSimExtension LANGUAGES CXX)

option(GRIFFINSIM_BUILD_SHARED "Build GriffinSim HALSIM extension as shared library" ON)
option(GRIFFINSIM_ENABLE_WPILIB_HALSIM "Enable WPILib HALSIM callback registration support" OFF)

set(GRIFFINSIM_WPILIB_HAL_INCLUDE_DIR "" CACHE PATH "Path to WPILib HAL native include directory")
set(GRIFFINSIM_WPILIB_HAL_LIBRARY "" CACHE FILEPATH "Path to WPILib HAL native library")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(GRIFFINSIM_BUILD_SHARED)
  set(GRIFFINSIM_LIBRARY_TYPE SHARED)
else()
  set(GRIFFINSIM_LIBRARY_TYPE STATIC)
endif()

add_library(griffinsim-halsim-extension
  ${GRIFFINSIM_LIBRARY_TYPE}
  src/griffinsim_halsim_extension.cpp
)

target_include_directories(griffinsim-halsim-extension
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_definitions(griffinsim-halsim-extension
  PRIVATE
    GRIFFINSIM_EXTENSION_EXPORTS=1
)

if(GRIFFINSIM_ENABLE_WPILIB_HALSIM)
  if(GRIFFINSIM_WPILIB_HAL_INCLUDE_DIR STREQUAL "")
    message(FATAL_ERROR "GRIFFINSIM_WPILIB_HAL_INCLUDE_DIR is required when GRIFFINSIM_ENABLE_WPILIB_HALSIM=ON")
  endif()
  if(GRIFFINSIM_WPILIB_HAL_LIBRARY STREQUAL "")
    message(FATAL_ERROR "GRIFFINSIM_WPILIB_HAL_LIBRARY is required when GRIFFINSIM_ENABLE_WPILIB_HALSIM=ON")
  endif()

  target_compile_definitions(griffinsim-halsim-extension PRIVATE GRIFFINSIM_ENABLE_WPILIB_HALSIM=1)
  target_include_directories(griffinsim-halsim-extension PRIVATE ${GRIFFINSIM_WPILIB_HAL_INCLUDE_DIR})
  target_link_libraries(griffinsim-halsim-extension PRIVATE ${GRIFFINSIM_WPILIB_HAL_LIBRARY})
endif()
