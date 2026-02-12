cmake_minimum_required(VERSION 3.14)
project(ThesisRCSPP LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)

# Source files
set(GRAPH_SRC
    src/graph/Graph.cpp
    src/graph/Roadmap.cpp
)

set(FRONTIER_SRC
    src/frontier/AVLTree.cpp
    src/frontier/ParetoFrontier.cpp
)

set(ALGORITHM_SRC
    src/algorithm/Dijkstra.cpp
    src/algorithm/ERCA.cpp
    src/algorithm/Pulse.cpp
    src/algorithm/Normalizer.cpp
)

set(BENCHMARK_SRC
    src/benchmark/Benchmark.cpp
)

# Core library (without NavMesh dependency)
add_library(rcspp_core STATIC
    ${GRAPH_SRC}
    ${FRONTIER_SRC}
    ${ALGORITHM_SRC}
    ${BENCHMARK_SRC}
)
target_include_directories(rcspp_core PUBLIC ${CMAKE_SOURCE_DIR}/include)

# Main executable
add_executable(main app/main.cpp)
target_link_libraries(main rcspp_core)

# Benchmark executable
add_executable(run_benchmark app/run_benchmark.cpp)
target_link_libraries(run_benchmark rcspp_core)

# NavMesh benchmark (loads DIMACS files from NavMeshERCA_Experiment)
add_executable(navmesh_benchmark app/navmesh_benchmark.cpp)
target_link_libraries(navmesh_benchmark rcspp_core)

# Pruning effectiveness analysis
add_executable(pruning_analysis app/pruning_analysis.cpp)
target_link_libraries(pruning_analysis rcspp_core)

# Ablation study
add_executable(ablation_study app/ablation_study.cpp)
target_link_libraries(ablation_study rcspp_core)

# Expanded benchmark (more data + iteration analysis)
add_executable(expanded_benchmark app/expanded_benchmark.cpp)
target_link_libraries(expanded_benchmark rcspp_core)

# Smart benchmark (alpha difficulty coefficient)
add_executable(smart_benchmark app/smart_benchmark.cpp)
target_link_libraries(smart_benchmark rcspp_core)

# Optional: NavMesh converter (requires RecastNavigation)
option(BUILD_NAVMESH "Build NavMesh converter (requires RecastNavigation)" OFF)
if(BUILD_NAVMESH)
    set(RECAST_DIR "${CMAKE_SOURCE_DIR}/../recastnavigation" CACHE PATH "Path to RecastNavigation")

    add_library(navmesh_converter STATIC
        src/graph/NavMeshConverter.cpp
    )
    target_include_directories(navmesh_converter PUBLIC
        ${CMAKE_SOURCE_DIR}/include
        ${RECAST_DIR}/Recast/Include
        ${RECAST_DIR}/Detour/Include
    )
    target_link_libraries(navmesh_converter rcspp_core)

    # Find Recast/Detour libraries
    find_library(RECAST_LIB Recast PATHS ${RECAST_DIR}/build/Recast/Release ${RECAST_DIR}/build/Recast/Debug)
    find_library(DETOUR_LIB Detour PATHS ${RECAST_DIR}/build/Detour/Release ${RECAST_DIR}/build/Detour/Debug)

    if(RECAST_LIB AND DETOUR_LIB)
        target_link_libraries(navmesh_converter ${RECAST_LIB} ${DETOUR_LIB})
    else()
        message(WARNING "Recast/Detour libraries not found. Build RecastNavigation first.")
    endif()
endif()
