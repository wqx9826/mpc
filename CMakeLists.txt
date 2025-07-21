cmake_minimum_required(VERSION 3.30)
project(mpc)

set(CMAKE_CXX_STANDARD 17)

include_directories(./utils)
include_directories(./mpc)

#include_directories(/usr/include/python3.10/)
#include_directories(/usr/include/eigen3/)
#include_directories(/usr/local/lib/python3.10/dist-packages/numpy/core/include)
#include_directories(/usr/lib/python3.10/config-3.10-x86_64-linux-gnu)
include_directories(/usr/include/eigen3)
include_directories(/usr/local/include/osqp)
find_package(Python3 3.7 COMPONENTS Interpreter Development NumPy REQUIRED)

add_executable(mpc main.cpp utils/KinematicModel.cpp utils/MyReferencePath.cpp
        mpc/MpcControl.h
        mpc/MpcControl.cpp
        utils/data_struct.h
        param.h
        mpc/mpcsolver.h
        mpc/mpcsolver.cpp
)

#target_link_libraries(mpc libpython3.10.so)
target_link_libraries(mpc PUBLIC Python3::Python Python3::NumPy )
target_link_libraries(mpc PUBLIC OsqpEigen)
target_link_libraries(mpc PUBLIC osqp)

