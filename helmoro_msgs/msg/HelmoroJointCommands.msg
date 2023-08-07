time stamp
uint32 seq

# Actuator command
# Control mode
int8 MODE_SHUTOFF=0         # Passive joints -> no torque
int8 MODE_JOINTPOSITION=1   # Joint position ref
int8 MODE_JOINTVELOCITY=2   # Joint velocity ref
int8 MODE_JOINTTORQUE=3     # Joint torque ref
int8[4] mode
# Commanded position; Joint[rad]
float64[4] position
# Commanded velocity; Joint[rad/s]
float64[4] velocity
# Commanded effort;   Joint Torque[Nm]
float64[4] effort