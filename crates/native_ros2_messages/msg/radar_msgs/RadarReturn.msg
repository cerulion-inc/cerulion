# All variables below are relative to the radar's frame of reference.
# This message is not meant to be used alone but as part of a stamped or array message.

float32 range                            # Distance (m) from the sensor to the detected return.
float32 azimuth                          # Angle (in radians) in the azimuth plane between the sensor and the detected return.
                                         # Positive angles are anticlockwise from the sensor and negative angles clockwise from the sensor as per REP-0103.
float32 elevation                        # Angle (in radians) in the elevation plane between the sensor and the detected return.
                                         # Negative angles are below the sensor. For 2D radar, this will be 0.
float32 doppler_velocity                 # The doppler speed (m/s) of the return.
float32 amplitude                        # The amplitude of the return (dB).
