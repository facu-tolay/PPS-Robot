idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES mqtt_cmp wifi kinematics motor_control pid utils wifi)
