cmake_minimum_required(VERSION 3.10)
project(Mujoco_Acrobot_Control)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    message(STATUS "OPENMP has been found")
endif()

set(Eigen3_FOUND True)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
if (TARGET Eigen3::Eigen)
    # Use the imported target
    message(STATUS "Eigen library found")
else()
    message(STATUS "could not find eigen library")
endif (TARGET Eigen3::Eigen)

set(Mujoco_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/mujoco/include)

find_library(GLFW libglfw.so.3 HINTS ${CMAKE_CURRENT_SOURCE_DIR}/mujoco/include/)
message(STATUS "GLFW lib found at: " ${GLFW})

file(GLOB LIB_MUJOCO ${CMAKE_CURRENT_SOURCE_DIR}/mujoco/include/libmujoco210.so)
message(STATUS "MuJoCo lib found at: " ${LIB_MUJOCO})

message(STATUS "Mujoco include directory: " ${Mujoco_INCLUDE_DIRS})

include_directories(${Mujoco_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        )

link_directories(${Mujoco_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} main.cpp
        Utility/MujocoController/MujocoUI.cpp Utility/MujocoController/MujocoUI.h
        Utility/MujocoController/MujocoController.cpp Utility/MujocoController/MujocoController.h
        iLQR/iLQR_dataCentric.cpp iLQR/iLQR_dataCentric.h
        modelTranslator/modelTranslator.cpp modelTranslator/modelTranslator.h)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen ${LIB_MUJOCO} ${GLFW} libGL.so libglew.so GL)
