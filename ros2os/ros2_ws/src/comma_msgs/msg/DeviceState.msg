# Thermal, power, resource and network state of the comma device.
# Published by comma_hardware; the same values are also available as
# diagnostic_msgs/DiagnosticArray on /diagnostics.

uint8 THERMAL_OK = 0
uint8 THERMAL_OVERHEATED = 1
uint8 THERMAL_CRITICAL = 2

std_msgs/Header header

string device_type          # "mici" (comma four), "tizi" (comma 3X)
string serial
string os_version
float64 uptime_s

# temperatures [degC]; NaN when a zone is not present on this device
float32[] cpu_temp_c
float32[] gpu_temp_c
float32 dsp_temp_c
float32 memory_temp_c
float32[] pmic_temp_c
float32 intake_temp_c
float32 exhaust_temp_c
float32 gnss_temp_c
float32 bottom_soc_temp_c
float32 max_temp_c          # control temperature used for fan and thermal status
uint8 thermal_status
uint8 fan_speed_percent_desired

# resources [%]
float32[] cpu_usage_percent
float32 memory_usage_percent
float32 gpu_usage_percent
float32 free_space_percent  # of /data

# power
float32 power_draw_w        # total input power
float32 som_power_draw_w    # SoM power
float32 car_voltage         # from the panda, NaN if unknown
bool ignition

# network
string network_type         # "wifi", "ethernet", "cellular", "none"
string[] ip_addresses
