include_directories(
    ${PROJECT_SOURCE_DIR}/dep
    ${PROJECT_SOURCE_DIR}/dep/eigen3
)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/lib)
link_directories(${PROJECT_SOURCE_DIR}/lib)

add_subdirectory(src)