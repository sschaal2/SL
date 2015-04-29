#############################################################################
#############################################################################
#
#  This is a CMakeList.txt file originally programmed for the CLMC/AMD labs
#  at the University of Southern California and the Max-Planck-Institute for
#  Intelligent Systems. We use a mixutre of explicit makefiles and cmake, but 
#  primarily we relay on cmake for all major compile dependencies. All our
#  software is provided under a slightly modified version of the LGPL license
#  to be found at http://www-clmc.usc.edu/software/license.
#
#  Copyright by Stefan Schaal, 2014
#
#############################################################################

cmake_minimum_required(VERSION 2.8)

# local defines
set(header_directory  ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(src_directory     ${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(BEFORE ${header_directory})
include_directories(BEFORE ${src_directory})

include_directories(${utilities_directory}/include)
include_directories(${lwpr_directory}/include)


# ------------------------------------------------------------------------

set(SRCS_SL_COMMON
	${src_directory}/SL_common.c
	${src_directory}/SL_collect_data.c
	${src_directory}/SL_shared_memory.c
	${src_directory}/SL_filters.c
	${src_directory}/SL_unix_common.c
	${src_directory}/SL_terrains.c
	${src_directory}/SL_oscilloscope.c
	${src_directory}/SL_man.c 
	)	

if(${MACHTYPE} MATCHES "(xeno)")
  set(SRCS_SL_COMMON
      ${src_directory}/SL_vx2xeno_wrappers.c
      ${src_directory}/SL_xeno_common.c
      ${SRCS_SL_COMMON}
	)
else()	
  set(SRCS_SL_COMMON
      ${src_directory}/SL_vx2unix_wrappers.c
      ${SRCS_SL_COMMON}
	)
endif()	


# ------------------------------------------------------------------------

set(SRCS_MOTOR_SERVO
	${src_directory}/SL_controller.c
	${src_directory}/SL_motor_servo.c
	${src_directory}/SL_sensor_proc.c
	)

if(${MACHTYPE} MATCHES "(xeno)")
  set(SRCS_MOTOR_SERVO
      ${src_directory}/SL_motor_servo_xeno.c
      ${SRCS_MOTOR_SERVO}
	)
else()
  set(SRCS_MOTOR_SERVO
      ${src_directory}/SL_motor_servo_unix.c
      ${SRCS_MOTOR_SERVO}
	)
endif()

# ------------------------------------------------------------------------

set(SRCS_TASK_SERVO
	${src_directory}/SL_tasks.c 
	${src_directory}/SL_task_servo.c 
	${src_directory}/SL_goto_task.c
	${src_directory}/SL_sine_task.c 
	${src_directory}/SL_go_cart_task.c
	${src_directory}/SL_objects.c
	${src_directory}/SL_traj_task.c
	)

if(${MACHTYPE}  MATCHES "(xeno)")
  set(SRCS_TASK_SERVO
      ${src_directory}/SL_task_servo_xeno.c
      ${SRCS_TASK_SERVO}
	)
else()
  set(SRCS_TASK_SERVO
      ${src_directory}/SL_task_servo_unix.c
      ${SRCS_TASK_SERVO}
	)
endif()

# ------------------------------------------------------------------------

set(SRCS_SIM_SERVO
	${src_directory}/SL_simulation_servo.c
	${src_directory}/SL_userSimulation.c
	${src_directory}/SL_objects.c
  ${src_directory}/SL_integrate.c
	) 

if(${MACHTYPE} MATCHES "(xeno)")
  set(SRCS_SIM_SERVO
      ${src_directory}/SL_simulation_servo_xeno.c
      ${SRCS_SIM_SERVO}
	) 
else()
  set(SRCS_SIM_SERVO
      ${src_directory}/SL_simulation_servo_unix.c
      ${SRCS_SIM_SERVO}
	) 
endif()

# ------------------------------------------------------------------------

set(SRCS_GL_SERVO
	${src_directory}/SL_openGL_servo.c
	${src_directory}/SL_openGL.c
	${src_directory}/SL_openGL_oscilloscope.c
	${src_directory}/SL_objects.c
	${src_directory}/SL_userGraphics.c 
	)

if(${MACHTYPE} MATCHES "(xeno)")
  set(SRCS_GL_SERVO
      ${src_directory}/SL_openGL_servo_xeno.c
      ${SRCS_GL_SERVO}
	)
else()
  set(SRCS_GL_SERVO
      ${src_directory}/SL_openGL_servo_unix.c
      ${SRCS_GL_SERVO}
	)
endif()

# ------------------------------------------------------------------------

set(SRCS_VISION_SERVO
	${src_directory}/SL_vision_servo.c
	${src_directory}/SL_dbvision.c
	${src_directory}/SL_vision_proc.c
	) 

if(${MACHTYPE}  MATCHES "(xeno)")
  set(SRCS_VISION_SERVO
      ${src_directory}/SL_vision_servo_xeno.c
      ${src_directory}/SL_serial_xeno.c
      ${SRCS_VISION_SERVO}
	) 
else()
  set(SRCS_VISION_SERVO
      ${src_directory}/SL_vision_servo_unix.c
      ${src_directory}/SL_serial_unix.c
      ${SRCS_VISION_SERVO}
	) 
endif()

# ------------------------------------------------------------------------

set(HEADERS
	  ${header_directory}/SL.h
	  ${header_directory}/SL_collect_data.h
	  ${header_directory}/SL_common.h 
	  ${header_directory}/SL_controller.h
	  ${header_directory}/SL_dynamics.h 
	  ${header_directory}/SL_filters.h 
	  ${header_directory}/SL_integrate.h
	  ${header_directory}/SL_kinematics.h 
	  ${header_directory}/SL_man.h
	  ${header_directory}/SL_motor_servo.h
	  ${header_directory}/SL_objects.h
	  ${header_directory}/SL_objects_defines.h
	  ${header_directory}/SL_openGL.h
	  ${header_directory}/SL_openGL_servo.h 
	  ${header_directory}/SL_oscilloscope.h
	  ${header_directory}/SL_rt_mutex.h
	  ${header_directory}/SL_sensor_proc.h
	  ${header_directory}/SL_shared_memory.h
	  ${header_directory}/SL_simulation_servo.h
	  ${header_directory}/SL_system_headers.h
	  ${header_directory}/SL_task_servo.h
	  ${header_directory}/SL_tasks.h
	  ${header_directory}/SL_terrains.h
	  ${header_directory}/SL_unix_common.h
	  ${header_directory}/SL_userGraphics.h
	  ${header_directory}/SL_userSimulation.h
	  ${header_directory}/SL_user_common.h
	  ${header_directory}/SL_user_display_core.h
	  ${header_directory}/SL_user_idle_core.h
	  ${header_directory}/SL_user_main_core.h
	  ${header_directory}/SL_vision_servo.h
	  ${header_directory}/SL_serial_unix.h
	  ${header_directory}/SL_serial_xeno.h
	  ${header_directory}/SL_vx_wrappers.h
	  ${header_directory}/SL_xeno_common.h
	  ${header_directory}/SL_xeno_headers.h 
	)

# ------------------------------------------------------------------------

add_library(SLcommon      ${SRCS_SL_COMMON})
list(APPEND SL_LIBRARIES SLcommon)
add_library(SLtask        ${SRCS_TASK_SERVO})
# undefined symbold break target_link_library in apollo
# list(APPEND SL_LIBRARIES SLtask)
add_library(SLmotor       ${SRCS_MOTOR_SERVO})
#list(APPEND SL_LIBRARIES SLmotor)
add_library(SLsimulation  ${SRCS_SIM_SERVO})
#list(APPEND SL_LIBRARIES SLsimulation)
add_library(SLopenGL      ${SRCS_GL_SERVO})
#list(APPEND SL_LIBRARIES SLopenGL)
add_library(SLvision      ${SRCS_VISION_SERVO})
#list(APPEND SL_LIBRARIES SLvision)

install(TARGETS SLcommon      
  ARCHIVE DESTINATION ${LAB_LIBDIR}
  LIBRARY DESTINATION ${LAB_LIBDIR})
install(TARGETS SLtask        
  ARCHIVE DESTINATION ${LAB_LIBDIR}
  LIBRARY DESTINATION ${LAB_LIBDIR})
install(TARGETS SLmotor       
  ARCHIVE DESTINATION ${LAB_LIBDIR}
  LIBRARY DESTINATION ${LAB_LIBDIR})
install(TARGETS SLsimulation  
  ARCHIVE DESTINATION ${LAB_LIBDIR}
  LIBRARY DESTINATION ${LAB_LIBDIR})
install(TARGETS SLopenGL      
  ARCHIVE DESTINATION ${LAB_LIBDIR}
  LIBRARY DESTINATION ${LAB_LIBDIR})
install(TARGETS SLvision     
  ARCHIVE DESTINATION ${LAB_LIBDIR}
  LIBRARY DESTINATION ${LAB_LIBDIR})

install(FILES ${HEADERS} DESTINATION ${LAB_INCLUDES})

# we make the libraries available for the parent scope
# such that we can use the catkin mechanisms
set(SL_LIBRARIES ${SL_LIBRARIES} PARENT_SCOPE)
