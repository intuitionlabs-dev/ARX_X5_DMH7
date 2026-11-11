target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    joint_post_process.c
    algorithms/ruckig_post_process.c
    algorithms/quintic_post_process.c
)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
