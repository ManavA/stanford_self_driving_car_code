Header header
int32 primary_sats          # primary GPS satellite constellation.
uint32 primary_id           # primary GPS unique id, for internal tracking.
float64 primary_timestamp   # primary GPS timestamp from machine receiving Applanix message, in seconds
int32 secondary_sats        # secondary GPS satellite constellation.  
uint32 secondary_id         # secondary GPS unique id, for internal tracking.
float64 secondary_timestamp # secondary GPS timestamp from machine receiving Applanix message, in seconds
int32 gams_solution_code    # GAMS status code.  Range is 0-7.  7 is full solution.
uint32 gams_id              # GAMS unique id, for internal tracking.
float64 gams_timestamp      # GAMS timestamp from machine receiving Applanix message, in seconds
float64 timestamp


