string name                       # traffic light name/id from rndf
uint8 state                       # state of main part of light: r=red, g=green, y=yellow, u=unknown
uint8 state_arrow                 # if has a turn arrow, include its state
                                  # otherwise this is set to 'n' to indicate that it does not have a turn arrow
float64 timestamp_rg_switch       # time of switch from red to green
float64 timestamp_gy_switch       # time of switch from green to red
float64 timestamp                 # should match timestamp of camera image that the state was determined from
float64 confidence                # correlation value from 0 to 1      
int32 u                           # section of the camera image plane where the light is predicted to be located
int32 v
