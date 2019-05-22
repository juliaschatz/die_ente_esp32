set(COMPONENT_SRCS "src/die_ente_main.c" 
                   "src/motor_control.c"
                   "src/unified_i2c.c"
                   "src/ahrs.c"
                   "src/task_util.c"
                   "src/flight_controller.c")
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()