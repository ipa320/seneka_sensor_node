Header header

sensor_msgs/NavSatFix NavSatFix

### in NavSatFix --> float64 latitude_value 	# in semi-circles
### in NavSatFix --> float64 longitude_value 	# in semi-circles
### in NavSatFix --> float64 altitude_value 	# [] = m
float64 clock_offset 		# [] = m
float64 frequency_offset 	# [] = Hz
float64 pdop 				#
float64 latitude_rate 		# [] = rad/s
float64 longitude_rate 		# [] = rad/s
float64 altitude_rate 		# [] = rad/s
uint32 	gps_msec_of_week 	# [] = ms
uint8 	position_flags 		# see Trimble BD982 GNSS receiver manual, p. 140
uint8 	number_of_SVs 		# number of used satellites
uint8[] channel_numbers 	# 1 char for each satellite
uint8[] prn 				# 1 char for each satellite