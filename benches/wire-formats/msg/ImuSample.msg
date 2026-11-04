# A fixed-size pose/IMU sample: 96 bytes of content, no strings, no sequences.
uint64 stamp_ns
uint32 seq
uint32 status
float64[4] orientation
float64[3] angular_velocity
float64[3] linear_acceleration
