cmake_minimum_required(VERSION 3.1)
project(NavMeshERCA_Experiment)

set(CMAKE_CXX_STANDARD 11)

# 添加RecastNavigation库路径
set(RECAST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../recastnavigation")

# 包含目录
include_directories(
    ${RECAST_DIR}/Recast/Include
    ${RECAST_DIR}/Detour/Include
    ${RECAST_DIR}/DebugUtils/Include
)

# 链接库目录
link_directories(
    ${RECAST_DIR}/build/Recast
    ${RECAST_DIR}/build/Detour
)

# Source files
set(SOURCES
    main_en.cpp
    GraphConverter_en.cpp
)

# 可执行文件
add_executable(navmesh_erca_experiment ${SOURCES})

# 链接库
target_link_libraries(navmesh_erca_experiment
    Recast
    Detour
)

# Large NavMesh generator
add_executable(generate_large_navmesh
    generate_large_navmesh.cpp
    GraphConverter_en.cpp
)
target_link_libraries(generate_large_navmesh
    Recast
    Detour
)
