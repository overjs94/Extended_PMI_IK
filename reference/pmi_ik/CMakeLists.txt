cmake_minimum_required(VERSION 3.0.2)
project(PMI_IK)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  urdf
  geometry_msgs
  ik_msgs
  message_generation
  #ik_visualization # ik visualization package
)
find_package(Eigen3 REQUIRED)
find_package(NLopt REQUIRED)
find_package(fcl REQUIRED)
include_directories(${fcl_INCLUDE_DIRS})
find_package(urdfdom REQUIRED) # Add this line

include_directories(${fcl_INCLUDE_DIRS} ${urdfdom_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include

  # LIBRARIES lasdra_ik
  CATKIN_DEPENDS roscpp rospy std_msgs urdf geometry_msgs ik_msgs message_runtime #ik_visualization

  # DEPENDS system_lib
)
set(QPOASES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/qpOASES)
include_directories(${QPOASES_DIR}/include)
link_directories(${QPOASES_DIR}/build/libs)

# ##########
# # Build ##
# ##########
add_library(PMI_IK_internal "")
target_include_directories(PMI_IK_internal PRIVATE include)
target_link_libraries(PMI_IK_internal PUBLIC Eigen3::Eigen)
target_compile_features(PMI_IK_internal PUBLIC cxx_std_17)
target_compile_options(PMI_IK_internal PUBLIC -fsanitize=address)
target_link_libraries(PMI_IK_internal PUBLIC -fsanitize=address)
add_subdirectory(include)

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${urdfdom_INCLUDE_DIRS}
)

## qp solver addon

add_subdirectory(qp_solver)


# # Link urdfdom libraries
# set(URDF_LIBS urdfdom_model urdfdom_world urdfdom_sensor urdfdom_headers)
set(SOURCE_FILES_FRANKA

  src/IK_evaluation.cpp


  ## franka evaluation w/ obstacles
  src/main_eval_franka_ranstart.cpp 

  
  ### 
  src/Inverse_kinematics.cpp
  src/IK_collision.cpp
  src/IK_new.cpp
)

set(SOURCE_FILES_LASDRA
  
  src/IK_evaluation.cpp
  src/panoc_solver.cpp


  ## franka evaluation w/ obstacles
  src/main_eval_lasdra_ranstart.cpp

  
  ### 
  src/Inverse_kinematics.cpp
  src/IK_collision.cpp
  src/IK_new.cpp
)

# add_executable(IK_franka_rand_node ${SOURCE_FILES_FRANKA})
# add_dependencies(IK_franka_rand_node ${catkin_EXPORTED_TARGETS})

add_executable(IK_lasdra_rand_node ${SOURCE_FILES_LASDRA})
add_dependencies(IK_lasdra_rand_node ${catkin_EXPORTED_TARGETS})

# target_include_directories(IK_franka_rand_node
#   PRIVATE
#   include
#   ${catkin_INCLUDE_DIRS}
#   ${Eigen3_INCLUDE_DIRS}
#   ${CMAKE_CURRENT_SOURCE_DIR}/qp_solver/src   # ← ADD THIS

# )

# target_link_libraries(IK_franka_rand_node
#   ${catkin_LIBRARIES}
#   ${Eigen3_LIBRARIES}
#   ${NLOPT_LIBRARIES}
#   ${FCL_LIBRARIES} # Link FCL
#   ${urdfdom_LIBRARIES} # Link urdfdom for parsing URDF

#   # ${URDF_LIBS}
#   ${QPOASES_DIR}/build/libs/libqpOASES.a
#   PMI_IK_internal
#   qpsolvers
# )

target_include_directories(IK_lasdra_rand_node
  PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/qp_solver/src   # ← ADD THIS

)

target_link_libraries(IK_lasdra_rand_node
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${NLOPT_LIBRARIES}
  ${FCL_LIBRARIES} # Link FCL
  ${urdfdom_LIBRARIES} # Link urdfdom for parsing URDF

  # ${URDF_LIBS}
  ${QPOASES_DIR}/build/libs/libqpOASES.a
  PMI_IK_internal
  qpsolvers
)

# set(SOURCE_FILES_NO_OBS_
  
#   #### Visualization w/ obstacles

#   # src/main.cpp

  

#   #### Evaluation wo/ obstacles

  
#   ## franka & lasdra wo/ obstacles
#   src/main_no_obs.cpp 
#   src/IK_evaluation_no_obs.cpp

#   ### 
#   src/Inverse_kinematics.cpp
#   src/IK_collision.cpp
#   src/IK_new.cpp
# )

# add_executable(IK_rand_no_obs ${SOURCE_FILES_NO_OBS_})
# add_dependencies(IK_rand_no_obs ${catkin_EXPORTED_TARGETS})

# target_include_directories(IK_rand_no_obs
#   PRIVATE
#   include
#   ${catkin_INCLUDE_DIRS}
#   ${Eigen3_INCLUDE_DIRS}
#   ${CMAKE_CURRENT_SOURCE_DIR}/qp_solver/src   # ← ADD THIS

# )

# target_link_libraries(IK_rand_no_obs
#   ${catkin_LIBRARIES}
#   ${Eigen3_LIBRARIES}
#   ${NLOPT_LIBRARIES}
#   ${FCL_LIBRARIES} # Link FCL
#   ${urdfdom_LIBRARIES} # Link urdfdom for parsing URDF

#   # ${URDF_LIBS}
#   ${QPOASES_DIR}/build/libs/libqpOASES.a
#   PMI_IK_internal
#   qpsolvers
# )
# add_subdirectory(src/dsf-mtd)

## trajectory tracking node

set(SOURCE_FILES_TRAJ_FRANKA
  # src/main_eval_franka_convergence.cpp
  src/main_eval_franka_traj.cpp
  # src/IK_traj_franka.cpp
  src/Inverse_kinematics.cpp
  src/IK_evaluation.cpp
  # src/Kinematics.cpp
  # src/optimization_IK.cpp
  src/IK_collision.cpp
  src/IK_new.cpp
  src/panoc_solver.cpp

)

add_executable(IK_traj_franka ${SOURCE_FILES_TRAJ_FRANKA})
add_dependencies(IK_traj_franka ${catkin_EXPORTED_TARGETS})

target_include_directories(IK_traj_franka
  PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/qp_solver/src   # ← ADD THIS

)

target_link_libraries(IK_traj_franka
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${NLOPT_LIBRARIES}
  ${FCL_LIBRARIES} # Link FCL
  ${urdfdom_LIBRARIES} # Link urdfdom for parsing URDF

  # ${URDF_LIBS}
  ${QPOASES_DIR}/build/libs/libqpOASES.a
  PMI_IK_internal
  qpsolvers
)

set(SOURCE_FILES_TRAJ_LASDRA
  src/main_eval_lasdra_traj.cpp
  # src/IK_traj_lasdra.cpp
  src/Inverse_kinematics.cpp
  src/IK_evaluation.cpp
  src/panoc_solver.cpp
  # src/Kinematics.cpp
  # src/optimization_IK.cpp
  src/IK_collision.cpp
  src/IK_new.cpp

)

add_executable(IK_traj_lasdra ${SOURCE_FILES_TRAJ_LASDRA})
add_dependencies(IK_traj_lasdra ${catkin_EXPORTED_TARGETS})

target_include_directories(IK_traj_lasdra
  PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/qp_solver/src   # ← ADD THIS

)

target_link_libraries(IK_traj_lasdra
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${NLOPT_LIBRARIES}
  ${FCL_LIBRARIES} # Link FCL
  ${urdfdom_LIBRARIES} # Link urdfdom for parsing URDF

  # ${URDF_LIBS}
  ${QPOASES_DIR}/build/libs/libqpOASES.a
  PMI_IK_internal
  qpsolvers
)

set(SOURCE_FILES_DUAL
  src/main_eval_dual_arm.cpp
  # src/IK_traj_lasdra.cpp
  src/Inverse_kinematics.cpp
  src/IK_evaluation.cpp
  src/panoc_solver.cpp
  # src/Kinematics.cpp
  # src/optimization_IK.cpp
  src/IK_collision.cpp
  src/IK_new.cpp

)

add_executable(IK_traj_franka_dual ${SOURCE_FILES_DUAL})
add_dependencies(IK_traj_franka_dual ${catkin_EXPORTED_TARGETS})

target_include_directories(IK_traj_franka_dual
  PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/qp_solver/src   # ← ADD THIS

)

target_link_libraries(IK_traj_franka_dual
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${NLOPT_LIBRARIES}
  ${FCL_LIBRARIES} # Link FCL
  ${urdfdom_LIBRARIES} # Link urdfdom for parsing URDF

  # ${URDF_LIBS}
  ${QPOASES_DIR}/build/libs/libqpOASES.a
  PMI_IK_internal
  qpsolvers
)

add_subdirectory(src/dsf-mtd)
