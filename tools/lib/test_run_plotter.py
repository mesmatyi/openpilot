from openpilot.tools.lib.route import Route
from openpilot.tools.lib.logreader import LogReader
import numpy as np
import matplotlib.pyplot as plt

# r_pid = Route("f4971592598fa9a3/0000004a--43e4827252")
r_lqr = Route("f4971592598fa9a3/00000046--039851256d")

r = Route("f4971592598fa9a3/0000004a--43e4827252")
# # get a list of paths for the route's rlog files

#setup a LogReader to read the route's first rlog
#lr = LogReader(r.log_paths()[0])

# # print out all the messages in the log
# import codecs
# codecs.register_error("strict", codecs.backslashreplace_errors)
# for msg in lr:
#   print(msg)

# setup a LogReader for the route's second qlog
lr = LogReader(r.log_paths())
r_lqr_l = LogReader(r_lqr.log_paths())

dt = 0.01  # time step in seconds

stage_entry = 1744792532000
stage_exit = 1744792772000

lateral_accel = []
lateral_accel_lqr = []

time_stamp = []
time_stamp_lqr = []



stage_lqr_entry = 1744791378000
stage_lqr_exit = 1744791604000



# print all the steering angles values from the log
for msg in lr:
  if msg.which() == "modelV2":
    lateral_accel.append(msg.modelV2.position.y[0])
  if msg.which() == "gpsLocationExternal":
    time_stamp.append(msg.gpsLocationExternal.unixTimestampMillis)
    
for msg in r_lqr_l:
  if msg.which() == "modelV2":
    lateral_accel_lqr.append(msg.modelV2.position.y[0])
  if msg.which() == "gpsLocationExternal":
    time_stamp_lqr.append(msg.gpsLocationExternal.unixTimestampMillis)

stage_time = []

start_index = 0
end_index = 0

start_lqr_index = 0
end_lqr_index = 0

print(len(time_stamp_lqr))


for i in range(len(time_stamp)):
  if time_stamp[i] > stage_entry:
    start_index = i
    break
for i in range(start_index, len(time_stamp)):
  if time_stamp[i] > stage_exit:
    end_index = i
    break
  
for i in range(len(time_stamp_lqr)):
  if time_stamp_lqr[i] > stage_lqr_entry:
    start_lqr_index = i
    break
for i in range(start_lqr_index, len(time_stamp_lqr)):
  if time_stamp_lqr[i] > stage_lqr_exit:
    end_lqr_index = i
    break 


scale_value = len(time_stamp) / len(lateral_accel)

start_acc_index = int(start_index / scale_value)
end_acc_index = int(end_index / scale_value)

stage_lateral_accel = lateral_accel[start_acc_index:end_acc_index]

start_acc_index = int(start_lqr_index / scale_value)
end_acc_index = int(end_lqr_index / scale_value)

lateral_accel_lqr = lateral_accel_lqr[start_acc_index:end_acc_index]


print(start_lqr_index, end_lqr_index)
print(start_index, end_index)
print(len(stage_lateral_accel), len(lateral_accel_lqr))


# drop the first 10 values of the jerk
# lateral_jerk = lateral_jerk[2500:]
stage_lateral_accel = stage_lateral_accel[:4900]
lateral_accel_lqr = lateral_accel_lqr[:4900]

# lateral_jerk = np.gradient(stage_lateral_accel, 0.01)
# lateral_jerk_lqr = np.gradient(lateral_accel_lqr, 0.01)

# lateral_jerk = lateral_jerk[1000:]
# lateral_jerk_lqr = lateral_jerk_lqr[1000:]

# Plot for visualization
time = np.arange(len(lateral_accel_lqr)) * dt
# plt.plot(time, lateral_accel, label='Lateral Acceleration (m/s²)')
plt.plot(time, stage_lateral_accel, label='Lateral Error (m)', linestyle='dotted')
plt.plot(time, lateral_accel_lqr, label= 'Later Error (m)', linestyle='dotted')
plt.xlabel('Time (s)')
plt.legend()
plt.grid(True)
plt.title('Lateral Error on Test Route')
plt.savefig("lateral_error_plot_east.svg",format="svg")
