Header header
float32 rms_north           # North position error, in meters (global frame)
float32 rms_east            # East position error, in meters (global frame)
float32 rms_up              # Up position error, in meters (global frame)
float32 rms_v_north         # North velocity error, in m/s (global frame)
float32 rms_v_east          # East velocity error, in m/s (global frame) 
float32 rms_v_up            # Up velocity error, in m/s (global frame)
float32 rms_roll            # roll error, in radians
float32 rms_pitch           # pitch error, in radians
float32 rms_yaw             # yaw error, in radians
float32 semi_major          # length of semi major axis of error ellipse, in m
float32 semi_minor          # length of semi minor axis of error ellipse, in m
float32 orientation         # orientation of error ellipse, radians
uint32 id                   # unique ID for internal tracking
int32 postprocess_code      # 0 = Real Time.  1 = Post Processed.  2 = Post Processed with Base Station.
float64 hardware_timestamp  # Timestamp from the Applanix hardware, in UTC seconds of the week.
int32 hardware_time_mode    # Mode of timestamp from the Applanix hardware.  0 = None.  1 = Acquire.  2 = Locked.
float64 timestamp

