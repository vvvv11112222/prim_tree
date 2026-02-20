cmake_minimum_required(VERSION 3.16)
project(PrimTreeDemo VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 REQUIRED COMPONENTS Core Widgets)

add_executable(prim_tree_demo
    src/main.cpp
    src/mainwindow.cpp
    src/graph.cpp
    src/prim_all_mst_solver.cpp
    include/mainwindow.h
    include/graph.h
    include/prim_all_mst_solver.h
)

target_include_directories(prim_tree_demo PRIVATE include)

target_link_libraries(prim_tree_demo PRIVATE Qt6::Core Qt6::Widgets)
