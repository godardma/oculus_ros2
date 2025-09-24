std_msgs/Header header

OculusFireConfig2 fire_message
uint32 ping_id            # An incrementing number 
uint32 status
float64   frequency         # The acoustic frequency (Hz) 
float64   temperature       # The external temperature (deg C) 
float64   pressure          # The external pressure (bar) 
float64   heading           # The heading (degrees) 
float64   pitch             # The pitch (degrees) 
float64   roll              # The roll (degrees) 
float64   speeed_of_sound_used # The actual used speed of sound (m/s) 
float64   ping_start_time     # In seconds from sonar powerup (to microsecond resolution) 
uint8  data_size          # The size of the individual data entries  # was DataSizeType
float64   range_resolution   # The range in metres corresponding to a single range line 
uint16 n_ranges           # The number of range lines in the image
uint16 n_beams            # The number of bearings in the image 
uint32 spare0
uint32 spare1
uint32 spare2
uint32 spare3
uint32 image_offset     # The offset in bytes of the image data from the start 
uint32 image_size       # The size in bytes of the image data 
uint32 message_size     # The total size in bytes of the network message 