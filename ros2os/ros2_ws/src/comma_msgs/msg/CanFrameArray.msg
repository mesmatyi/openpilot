# All CAN frames drained from the panda in one poll (the panda is polled at
# 100 Hz). Batching keeps the per-message overhead low at several thousand
# frames per second.
#
# header.stamp is the host time the batch was read; the panda does not provide
# per-frame hardware timestamps.

std_msgs/Header header
CanFrame[] frames
