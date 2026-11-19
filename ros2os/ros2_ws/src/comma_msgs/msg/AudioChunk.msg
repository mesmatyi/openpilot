# A chunk of raw microphone samples.

std_msgs/Header header      # stamp = capture time of the first sample
uint32 sample_rate          # [Hz]
uint8 channels
string encoding             # "S16LE" (interleaved signed 16-bit little endian)
uint8[] data
