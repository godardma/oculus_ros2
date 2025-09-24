bool ping_enable		# false: no ping  true: sonar pings
bool low_frequency		# false: sonar use high frequency   true: sonar use low frequncy
bool beams_512			# false: 256 beams    true: 512 beams
bool pixels_16bits		# false: pixels are 8 bits  true: pixels are 16 bits
#    pingRateNormal  = 0x00, // 10Hz max ping rate
#    pingRateHigh    = 0x01, // 15Hz max ping rate
#    pingRateHighest = 0x02, // 40Hz max ping rate
#    pingRateLow     = 0x03, // 5Hz max ping rate
#    pingRateLowest  = 0x04, // 2Hz max ping rate
#    pingRateStandby = 0x05, // Disable ping
uint8 ping_rate			# set sonar maximum ping rate 
float32 range_meters	# set sonar range in meters
float32 rx_gain			# from 0 to 100
bool gain_assist		#
float32 sound_speed		# sound speed to use in m/s
bool use_salinity		# use salinity to compuet sound speed
float32 salinity		# salinity value in ppt
uint8 gamma_correction	# 0 < gamma_correction < 255 default: 127
# v2 message extra flags
bool scientific_mode	# false: pixels are coded according pixels_16bits  true: pixels are 32 bits
bool gain_boost			# boost of rx gain?
bool transmit_boost		# boost of tx gain?

