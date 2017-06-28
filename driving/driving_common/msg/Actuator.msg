Header header
uint8 direction
uint8 steering_mode
float64 steering_value     # depending on steering_mode this is either angle or torque
float64 brake_pressure
float64 throttle_fraction
float64 timestamp

  # for direction
uint8 DIRECTION_FORWARD = 0
uint8 DIRECTION_REVERSE = 1

  # for steering mode
uint8 ANGLE_CONTROL = 0
uint8 TORQUE_CONTROL = 1


