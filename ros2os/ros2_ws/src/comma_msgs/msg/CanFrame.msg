# One classic CAN or CAN FD frame received from / sent to the panda.
#
# The panda reports frames it transmitted back to the host ("returned"/echo) and
# frames its safety model refused to send ("rejected"); both flags are false for
# frames received from the bus.

uint8 bus           # CAN bus index on the panda (0..2 on comma four, 3 = optional)
uint32 id           # 11-bit or 29-bit arbitration ID
bool is_extended    # true for 29-bit IDs
bool returned       # echo of a frame this panda transmitted
bool rejected       # frame was blocked by the panda safety model
uint8[<=64] data    # payload; > 8 bytes implies CAN FD
