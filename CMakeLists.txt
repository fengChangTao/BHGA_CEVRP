cmake_minimum_required(VERSION 3.26)
project(HGA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXE_LINKER_FLAGS "-static")

add_executable(HGA Program/main.cpp
        Program/AlgorithmParameters.cpp
        Program/case.cpp
        Program/Genetic.cpp
        Program/Individual.cpp
        Program/InstanceCVRPLIB.cpp
        Program/LocalSearch.cpp
        Program/Params.cpp
        Program/Population.cpp
        Program/Split.cpp
        )
