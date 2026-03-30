cmake_minimum_required(VERSION 3.22)
project(ZED_IceoryX2_Camera_Publisher LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Prefer single-config generators; harmless on multi-config ones
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(LINK_SHARED_ZED "Link with the ZED SDK shared libraries" ON)

if(NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED=OFF: ZED SDK static libraries are not available on Windows")
endif()

find_package(ZED REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)

# iceoryx2 C++ package
find_package(iceoryx2-cxx REQUIRED)

add_executable(${PROJECT_NAME}
    src/camera_instance_service.cpp
    src/main.cpp
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${ZED_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
)

if(LINK_SHARED_ZED)
    set(ZED_LIBS
        ${ZED_LIBRARIES}
        ${CUDA_CUDA_LIBRARY}
        ${CUDA_CUDART_LIBRARY}
    )
else()
    set(ZED_LIBS
        ${ZED_STATIC_LIBRARIES}
        ${CUDA_CUDA_LIBRARY}
        ${CUDA_LIBRARY}
    )
endif()

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        ${ZED_LIBS}
        iceoryx2-cxx::static-lib-cxx
)

target_compile_options(${PROJECT_NAME}
    PRIVATE
        $<$<CONFIG:Debug>:-g>
)

if(INSTALL_SAMPLES)
    list(APPEND SAMPLE_LIST ${PROJECT_NAME})
    set(SAMPLE_LIST "${SAMPLE_LIST}" PARENT_SCOPE)
endif()
