Header header
float32 throttle_position        # Throttle percentage - 0 to 100
float32 steering_angle           # Steering wheel angle in degees
float32 steering_rate            # Steering wheel velocity in deg/sec
float32 engine_rpm               # Engine RPM
uint8 parking_brake
uint8 target_gear                # Internal gear
uint8 gear_position              # Position of the gear shift lever
float32 wheel_speed_fl           # Front left wheel speed in km/hr
float32 wheel_speed_fr           # Front right wheel speed in km/hr
float32 wheel_speed_rl           # Rear left wheel speed in km/hr
float32 wheel_speed_rr           # Rear right wheel speed in km/hr
float32 wheel_height_fl          # Suspension encoder height in mm
float32 wheel_height_fr
float32 wheel_height_rl
float32 wheel_height_rr
float32 brake_pressure           # Brake pressure in unknown units
uint8 esp_status                 # 1 = active, 0 = inactive
uint8 abs_status                 # 1 = active, 0 = inactive
uint8 throttle_error
uint8 rpm_error
uint8 wheel_height_fl_error
uint8 wheel_height_fr_error
uint8 wheel_height_rl_error
uint8 wheel_height_rr_error
uint8 steering_status            # see CAN docu, lots of things...
float32 avg_wheel_revolutions    # 0..65278 rotations/sec
int32 distance_pulses_front_axle # 0..2047
float32 yaw_rate                 # 0..100 deg/sec 200 indicates invalid yaw_rate 
uint8 backing_up_light           # 1 = on 0 = off
float32 wheel_impulses_fl        # Front left wheel impulses 0-1000 1021=initValue 1022=underVoltage 1023=overVoltage 1030=QualificationBitImpulsesInvalid
int32 wheel_direction_fl         # Front left wheel -1=backwards 0=QualificationBitDirectionInvalid 1=forward
float32 wheel_impulses_fr 
int32 wheel_direction_fr
float32 wheel_impulses_rl
int32 wheel_direction_rl 
float32 wheel_impulses_rr 
int32 wheel_direction_rr
int32 wheel_direction_rr_added    # Wheel direction rear right, added from sensor/uC from VW Germany
float64 steer_angleCalculated     # -614,25..614,25 deg   1000 indicates invalid angleCalculated   
float64 steer_handTorque          # -15Nm..15Nm   1000 indicates invalid handTorque
uint8 steer_statusEPS             # Status of EPS according Lastenheft PLA/EPS
float64 timestamp
string host
  

uint8 CLUTCH_OPEN                = 0
uint8 CLUTCH_CONTROLLED_SLIP     = 1
uint8 CLUTCH_CLOSED              = 2
uint8 CLUTCH_ERROR               = 3

uint8 TARGET_GEAR_PARK_NEUTRAL   = 0
uint8 TARGET_GEAR_1ST            = 1
uint8 TARGET_GEAR_2ND            = 2
uint8 TARGET_GEAR_3RD            = 3
uint8 TARGET_GEAR_4TH            = 4
uint8 TARGET_GEAR_5TH            = 5
uint8 TARGET_GEAR_1M             = 6
uint8 TARGET_GEAR_REVERSE        = 7
uint8 TARGET_GEAR_6TH            = 8
uint8 TARGET_GEAR_7TH            = 9
uint8 TARGET_GEAR_ERROR          = 15

uint8 GEAR_POSITION_INTERMEDIATE = 0
uint8 GEAR_POSITION_1            = 1
uint8 GEAR_POSITION_2            = 2
uint8 GEAR_POSITION_3            = 3
uint8 GEAR_POSITION_4            = 4
uint8 GEAR_POSITION_D            = 5
uint8 GEAR_POSITION_N            = 6
uint8 GEAR_POSITION_R            = 7
uint8 GEAR_POSITION_P            = 8
uint8 GEAR_POSITION_RSP          = 9
uint8 GEAR_POSITION_Z1           = 10
uint8 GEAR_POSITION_Z2           = 11
uint8 GEAR_POSITION_S            = 12
uint8 GEAR_POSITION_L            = 13
uint8 GEAR_POSITION_TIPTRONIC    = 14
uint8 GEAR_POSITION_ERROR        = 15
 
