cmake_minimum_required(VERSION 2.8.3)
project(sl)

## find SL dependencies using catkin
find_package(catkin REQUIRED COMPONENTS
  sl_config
  utilities
  lwpr
)

## find system libraries using standard cmake
find_package(Libedit REQUIRED)
find_package(X11 REQUIRED)
find_package(OpenGL REQUIRED)
sl_find_package_GLUT()
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
find_package(Threads REQUIRED)
find_package(LibRt REQUIRED)

message(STATUS "GLUT_LIBRARIES=${GLUT_LIBRARIES}")
message(STATUS "GLUT_INCLUDE_DIR=${GLUT_INCLUDE_DIR}")
message(STATUS "GLUT_FOUND: ${GLUT_FOUND}")
message(STATUS "Libedit_INCLUDE_DIRS=${Libedit_INCLUDE_DIRS}")
message(STATUS "Libedit_LIBRARIES=${Libedit_LIBRARIES}")

# check for xenomai
find_package(xenomai_ros QUIET)

# check	for optional ros packages
find_package(roscpp QUIET)
find_package(sensor_msgs QUIET)
find_package(geometry_msgs QUIET)
if(roscpp_FOUND AND sensor_msgs_FOUND AND geometry_msgs_FOUND)
  message(STATUS "SLros dependencies found; compiling ROS servo.")
  set(SLros_REQUIREMENTS_FOUND "TRUE")
else()
  message(STATUS "SLros dependencies NOT found.")
endif()


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS utilities lwpr
  DEPENDS X11 OpenGL
)
sl_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Libedit_INCLUDE_DIRS}
  ${X11_INCLUDE_DIR}  
  ${OPENGL_INCLUDE_DIR}  
  ${GLUT_INCLUDE_DIR}  
)

# default SL_PLATFORM is unix
set (SL_PLATFORM unix)

if (Xenomai_FOUND)
  add_xenomai_flags()
  set (SL_PLATFORM xeno)
endif()

# SLcommon
set (SLcommon_FILES
  src/SL_collect_data.c
  src/SL_common.c
  src/SL_filters.c
  src/SL_man.c
  src/SL_oscilloscope.c
  src/SL_shared_memory.c
  src/SL_terrains.c
  src/SL_unix_common.c
  src/SL_vx2${SL_PLATFORM}_wrappers.c
)
if (Xenomai_FOUND)
  list(APPEND SLcommon_FILES src/SL_xeno_common.c)
endif()

add_library(SLcommon
  ${SLcommon_FILES}
)
target_link_libraries(SLcommon
  ${utilities_LIBRARIES}
  ${Libedit_LIBRARIES}
  ${LibRt_LIBRARIES}
  ${Xenomai_LIBRARIES_NATIVE}
  ${CMAKE_THREAD_LIBS_INIT}
  ${Xenomai_LIBRARY_RTDK}
)
set_target_properties(SLcommon PROPERTIES LINK_FLAGS ${SL_IGNORE_UNDEF_SYMBOLS})

# SLtask
add_library(SLtask
  src/SL_go_cart_task.c
  src/SL_goto_task.c
  src/SL_objects.c
  src/SL_sine_task.c
  src/SL_task_servo.c
  src/SL_task_servo_${SL_PLATFORM}.c
  src/SL_tasks.c
  src/SL_traj_task.c
)
set_target_properties(SLtask PROPERTIES LINK_FLAGS ${SL_IGNORE_UNDEF_SYMBOLS})

# SLmotor 
add_library(SLmotor
  src/SL_controller.c
  src/SL_motor_servo.c
  src/SL_motor_servo_${SL_PLATFORM}.c
  src/SL_sensor_proc.c
)
set_target_properties(SLmotor PROPERTIES LINK_FLAGS ${SL_IGNORE_UNDEF_SYMBOLS})

# SLsimulation
add_library(SLsimulation
  src/SL_integrate.c
  src/SL_objects.c
  src/SL_simulation_servo.c
  src/SL_simulation_servo_${SL_PLATFORM}.c
  src/SL_userSimulation.c
)
set_target_properties(SLsimulation PROPERTIES LINK_FLAGS ${SL_IGNORE_UNDEF_SYMBOLS})

# SLopenGL
add_library(SLopenGL
  src/SL_openGL_servo.c
  src/SL_openGL_servo_${SL_PLATFORM}.c
  src/SL_openGL.c
  src/SL_openGL_oscilloscope.c
  src/SL_objects.c
  src/SL_userGraphics.c
)
target_link_libraries(SLopenGL
  ${OPENGL_LIBRARIES}
  ${X11_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${Xenomai_LIBRARIES_NATIVE}
  ${Xenomai_LIBRARY_RTDK}
)
set_target_properties(SLopenGL PROPERTIES LINK_FLAGS ${SL_IGNORE_UNDEF_SYMBOLS})

# SLvision
add_library(SLvision
  src/SL_dbvision.c
  src/SL_serial_${SL_PLATFORM}.c
  src/SL_vision_proc.c
  src/SL_vision_servo.c
  src/SL_vision_servo_${SL_PLATFORM}.c
)
target_link_libraries(SLvision
  ${lwpr_LIBRARIES})
set_target_properties(SLvision PROPERTIES LINK_FLAGS ${SL_IGNORE_UNDEF_SYMBOLS})

# SLros
if(SLros_REQUIREMENTS_FOUND)
  include_directories(
    ${roscpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
  )
  add_library(SLros
    src/SL_ros_servo.cpp
    src/SL_ros_servo_${SL_PLATFORM}.cpp
    src/SL_ros_communicator.cpp
  )
  target_link_libraries(SLros
    ${roscpp_LIBRARIES}
  )
endif()

#install(TARGETS SLcommon SLtask SLmotor SLsimulation SLopenGL SLvision SLros 
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(FILES
#  include/SL.h
#  include/SL_collect_data.h
#  include/SL_common.h
#  include/SL_controller.h
#  include/SL_dynamics.h
#  include/SL_filters.h
#  include/SL_forDynComp_body.h
#  include/SL_forDynArt_body.h
#  include/SL_integrate.h
#  include/SL_invDynArt_body.h
#  include/SL_invDynNE_body.h
#  include/SL_kinematics.h
#  include/SL_kinematics_body.h
#  include/SL_man.h
#  include/SL_motor_servo.h
#  include/SL_objects.h
#  include/SL_objects_defines.h
#  include/SL_openGL.h
#  include/SL_openGL_servo.h
#  include/SL_oscilloscope.h
#  include/SL_parm_estimate_body.h
#  include/SL_ros_servo.h
#  include/SL_rt_mutex.h
#  include/SL_sensor_proc.h
#  include/SL_shared_memory.h
#  include/SL_simulation_servo.h
#  include/SL_system_headers.h
#  include/SL_task_servo.h
#  include/SL_tasks.h
#  include/SL_terrains.h
#  include/SL_unix_common.h
#  include/SL_userGraphics.h
#  include/SL_userSimulation.h
#  include/SL_user_common.h
#  include/SL_user_display_core.h
#  include/SL_user_idle_core.h
#  include/SL_user_main_core.h
#  include/SL_vision_servo.h
#  include/SL_serial_unix.h
#  include/SL_serial_xeno.h
#  include/SL_vx_wrappers.h
#  include/SL_xeno_common.h
#  include/SL_xeno_headers.h
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)
