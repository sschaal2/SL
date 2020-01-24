package(default_visibility = ["//visibility:public"])

exports_files([
    "src/SL_parm_estimate.c",
    "src/SL_kinematics.c",
    "src/SL_dynamics.c",
    "src/SL_invDynNE.cpp",
    "src/SL_invDynArt.cpp",
    "src/SL_forDynComp.cpp",
    "src/SL_forDynArt.cpp",
])

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
        "//experimental/users/sschaal/SL/lwpr",
        "//experimental/users/sschaal/SL/utilities:utility",
        "//third_party/libedit:edit_impl",
        "//third_party/libedit:native_system",
    ],
)

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
        "//experimental/users/sschaal/SL/lwpr",
        "//experimental/users/sschaal/SL/utilities:utility",
    ],
)

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
        "//experimental/users/sschaal/SL/lwpr",
        "//experimental/users/sschaal/SL/utilities:utility",
    ],
)

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
        "//experimental/users/sschaal/SL/lwpr",
        "//experimental/users/sschaal/SL/utilities:utility",
    ],
)

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
        "//experimental/users/sschaal/SL/lwpr",
        "//experimental/users/sschaal/SL/utilities:utility",
        "//third_party/freeglut:freeglut_base",
        "//third_party/glu:native",
        "//third_party/Xorg:libX11",		
    ],
)

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
        "//experimental/users/sschaal/SL/lwpr",
        "//experimental/users/sschaal/SL/utilities:utility",
    ],
)
