# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

cmake_minimum_required(VERSION 3.1)
# set(CMAKE_CXX_FLAGS_DEBUG "-O2 -g -std=c++17")
# set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g -std=c++20")
set(CMAKE_CXX_FLAGS_DEBUG "-O2 -g -std=c++17")
set(COMPONENT_ADD_INCLUDEDIRS
    "."
    "include"
)
set(COMPONENT_SRCS 
    "main.cpp"
    "task/logging_task.cpp"
    "task/trajectory_creator.cpp"
    "task/path_creator.cpp"
    "task/motion_planning.cpp"
    "task/planning_task.cpp"
    "task/ui.cpp"
    "task/fs.cpp"
    "task/main_task.cpp"
    "task/icm20689.cpp"
    "task/lsm6dsr.cpp"
    "task/sensing_task.cpp"
    "task/as5147p.cpp"
    "task/kalman_filter.cpp"
    "search/adachi.cpp"
    "search/logic.cpp"
    "search/search_controller.cpp"
    "gen_code_mpc/mpc_tgt_calc_data.cpp"
    "gen_code_mpc/mpc_tgt_calc.cpp"
    "gen_code_mpc/rt_nonfinite.cpp"
    "gen_code_mpc/rtGetInf.cpp"
    "gen_code_mpc/rtGetNaN.cpp"
    "gen_code_conv_single2half/half_type.cpp"
    # "gen_code_pid/pid_controller_data.cpp"
    # "gen_code_pid/pid_controller.cpp"
    # "gen_code_pid_2dof/pid_controller_2dof_data.cpp"
    # "gen_code_pid_2dof/pid_controller_2dof.cpp"
    "gen_code_simple_pid/simple_pid_controller_data.cpp"
    "gen_code_simple_pid/simple_pid_controller.cpp"
    )
register_component()

# idf_component_register(SRCS ${COMPONENT_SRCS}
#                     INCLUDE_DIRS ${COMPONENT_ADD_INCLUDEDIRS}
#                     REQUIRES eigen)