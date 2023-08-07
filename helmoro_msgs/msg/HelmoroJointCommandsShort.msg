time stamp
uint32 seq

# Actuator command
# Control mode
# int8 MODE_SHUTOFF=0         # Passive joints -> no torque
# int8 MODE_JOINTPOSITION=1   # Joint position ref
# int8 MODE_JOINTVELOCITY=2   # Joint velocity ref
# int8 MODE_JOINTTORQUE=3     # Joint torque ref
int8 mode
# Commanded ; Position[rad]; Velocity[rad/s]; Effort[Nm]
float64[4] command
