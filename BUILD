package(default_visibility = ["//visibility:public"])

licenses(["notice"])

exports_files(["LICENSE"])

# Every SL directory has a symbolic link to config/bazel to access the config files as local path.
# While not pretty, this allows BUILD files to be independt of the SL_ROOT workspace path, and only
# SL.bzl needs to be adjusted

load(":bazel/SL.bzl", "SL_ROOT")

# The following filegroups contain files that need to be compiled from each robots core directory.
# These files are skeletons, which include various math files that are robot specific. Thus, they
# cannot be compiled here. But only the math files are different between robots.
filegroup(
    name = "kin_and_dyn_srcs",
    srcs = [
        "src/SL_dynamics.c",
        "src/SL_forDynArt.cpp",
        "src/SL_forDynComp.cpp",
        "src/SL_invDynArt.cpp",
        "src/SL_invDynNE.cpp",
        "src/SL_kinematics.c",
    ],
)

filegroup(
    name = "parm_est_srcs",
    srcs = [
        "src/SL_parm_estimate.c",
    ],
)

# This libarary is used by all SL processes
cc_library(
    name = "SLcommon",
    srcs = [
        "src/SL_collect_data.c",
        "src/SL_common.c",
        "src/SL_filters.c",
        "src/SL_man.c",
        "src/SL_oscilloscope.c",
        "src/SL_shared_memory.c",
        "src/SL_terrains.c",
        "src/SL_unix_common.c",
        "src/SL_vx2unix_wrappers.c",
    ],
    includes = [
        "include",
    ],
    textual_hdrs = glob(["include/*.h"]),
    deps = [
        SL_ROOT + "lwpr",
        SL_ROOT + "utilities:utility",
        "//third_party/libedit:native",
        "//third_party/libedit:native_system",
    ],
)

# Library for the motor_servo process, the low level I/O of each robot that either
# connects to a physical simulator or a real robot.
cc_library(
    name = "SLmotor",
    srcs = [
        "src/SL_controller.c",
        "src/SL_motor_servo.c",
        "src/SL_motor_servo_unix.c",
        "src/SL_sensor_proc.c",
    ],
    includes = [
        "include",
    ],
    textual_hdrs = glob(["include/*.h"]),
    deps = [
        SL_ROOT + "lwpr",
        SL_ROOT + "utilities:utility",
    ],
)

# Library for the task_servo process, which runs user programmed tasks and skills
cc_library(
    name = "SLtask",
    srcs = [
        "src/SL_go_cart_task.c",
        "src/SL_goto_task.c",
        "src/SL_objects.c",
        "src/SL_sine_task.c",
        "src/SL_task_servo.c",
        "src/SL_task_servo_unix.c",
        "src/SL_tasks.c",
        "src/SL_traj_task.c",
    ],
    includes = [
        "include",
    ],
    textual_hdrs = glob(["include/*.h"]),
    deps = [
        SL_ROOT + "lwpr",
        SL_ROOT + "utilities:utility",
    ],
)

# Library for the physical simulator
cc_library(
    name = "SLsimulation",
    srcs = [
        "src/SL_integrate.c",
        "src/SL_objects.c",
        "src/SL_simulation_servo.c",
        "src/SL_simulation_servo_unix.c",
        "src/SL_userSimulation.c",
    ],
    includes = [
        "include",
    ],
    textual_hdrs = glob(["include/*.h"]),
    deps = [
        SL_ROOT + "lwpr",
        SL_ROOT + "utilities:utility",
    ],
)

# Library for graphics/visualziation of a robot
cc_library(
    name = "SLopenGL",
    srcs = [
        "src/SL_objects.c",
        "src/SL_openGL.c",
        "src/SL_openGL_oscilloscope.c",
        "src/SL_openGL_servo.c",
        "src/SL_openGL_servo_unix.c",
        "src/SL_userGraphics.c",
    ],
    includes = [
        "include",
    ],
    textual_hdrs = glob([
        "include/*.h",
        "include/GL/*.h",
        "include/X11/*.h",
    ]),
    deps = [
        SL_ROOT + "lwpr",
        SL_ROOT + "utilities:utility",
        "//third_party/Xorg:libX11",
        "//third_party/freeglut:headers",
        "//third_party/freeglut:native",
        "//third_party/glu:native",
    ],
)

# Library for a dedicated vision process. Mostly not used anymore, as it assume
# a color blob vision tracking system
cc_library(
    name = "SLvision",
    srcs = [
        "src/SL_dbvision.c",
        "src/SL_serial_unix.c",
        "src/SL_vision_proc.c",
        "src/SL_vision_servo.c",
        "src/SL_vision_servo_unix.c",
    ],
    includes = [
        "include",
    ],
    textual_hdrs = glob(["include/*.h"]),
    deps = [
        SL_ROOT + "lwpr",
        SL_ROOT + "utilities:utility",
    ],
)

cc_library(
    name = "src/sl_task_servo_xeno",
    srcs = ["src/SL_task_servo_xeno.c"],
)

cc_library(
    name = "src/sl_for_dyn_art",
    srcs = ["src/SL_forDynArt.cpp"],
)

cc_library(
    name = "src/sl_dynamics",
    srcs = ["src/SL_dynamics.c"],
)

cc_library(
    name = "src/sl_inv_dyn_ne",
    srcs = ["src/SL_invDynNE.c"],
)

cc_library(
    name = "src/sl_task_servo_vx",
    srcs = ["src/SL_task_servo_vx.c"],
)

cc_library(
    name = "src/sl_parm_estimate",
    srcs = ["src/SL_parm_estimate.c"],
)

cc_library(
    name = "src/sl_for_dyn_art_c",
    srcs = ["src/SL_forDynArt.c"],
)

cc_library(
    name = "src/sl_for_dyn_comp",
    srcs = ["src/SL_forDynComp.c"],
)

cc_library(
    name = "src/sl_vision_servo_vx",
    srcs = ["src/SL_vision_servo_vx.c"],
)

cc_library(
    name = "src/sl_motor_servo_xeno",
    srcs = ["src/SL_motor_servo_xeno.c"],
)

cc_library(
    name = "src/sl_inv_dyn_art",
    srcs = ["src/SL_invDynArt.cpp"],
)

cc_library(
    name = "src/sl_vx_serial",
    srcs = ["src/SL_vx_serial.c"],
)

cc_library(
    name = "src/sl_inv_dynamics_art",
    srcs = ["src/SL_inv_dynamics_art.c"],
)

cc_library(
    name = "src/sl_for_dyn_comp_cpp",
    srcs = ["src/SL_forDynComp.cpp"],
)

cc_library(
    name = "src/sl_motor_servo_vx",
    srcs = ["src/SL_motor_servo_vx.c"],
)

cc_library(
    name = "src/sl_vision_servo_xeno",
    srcs = ["src/SL_vision_servo_xeno.c"],
)

cc_library(
    name = "src/sl_kinematics",
    srcs = ["src/SL_kinematics.c"],
)

cc_library(
    name = "src/sl_inv_dyn_ne_cpp",
    srcs = ["src/SL_invDynNE.cpp"],
)

cc_library(
    name = "src/sl_register",
    srcs = ["src/SL_register.c"],
)

cc_library(
    name = "src/sl_vx2xeno_wrappers",
    srcs = ["src/SL_vx2xeno_wrappers.c"],
)

cc_library(
    name = "src/sl_unix_wrappers",
    srcs = ["src/SL_unix_wrappers.c"],
)

cc_library(
    name = "src/sl_simulation_servo_xeno",
    srcs = ["src/SL_simulation_servo_xeno.c"],
)

cc_library(
    name = "src/sl_xeno_common",
    srcs = ["src/SL_xeno_common.c"],
)

cc_library(
    name = "src/sl_open_gl_servo_xeno",
    srcs = ["src/SL_openGL_servo_xeno.c"],
)

cc_library(
    name = "src/sl_serial_xeno",
    srcs = ["src/SL_serial_xeno.c"],
)

cc_library(
    name = "src/sl_inv_dyn_art_c",
    srcs = ["src/SL_invDynArt.c"],
)