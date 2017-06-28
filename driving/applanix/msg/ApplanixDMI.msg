Header header
float64 signed_odometer       # DMI distance odometer, in meters, signed value
float64 unsigned_odometer     # DMI distance odometer, in meters, unsigned value
uint32 id                     # unique id for internal tracking
float64 hardware_timestamp    # Timestamp from the Applanix hardware, in UTC seconds of the week.
int32 hardware_time_mode      # Mode of timestamp from the Applanix hardware.  0 = None.  1 = Acquire.  2 = Locked.
float64 timestamp

