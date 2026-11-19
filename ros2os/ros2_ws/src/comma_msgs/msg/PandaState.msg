# Health of the panda (the STM32H7 CAN interface inside the comma device).

uint8 HARNESS_NOT_CONNECTED = 0
uint8 HARNESS_NORMAL = 1
uint8 HARNESS_FLIPPED = 2

std_msgs/Header header

string serial
string hw_type              # e.g. "cuatro", "tres"
string firmware_version
float32 uptime_s

float32 voltage             # car supply voltage [V] (12 V battery when harnessed)
float32 current             # supply current [A]

bool ignition_line          # ignition sensed on the harness ignition pin
bool ignition_can           # ignition inferred from CAN traffic
bool ignition               # ignition_line or ignition_can

bool controls_allowed
string safety_model         # opendbc CarParams.SafetyModel name, e.g. "noOutput"
uint16 safety_param
uint16 alternative_experience
bool heartbeat_lost
bool power_save_enabled

uint8 harness_status
uint32 fault_status
uint32 faults

uint32 safety_tx_blocked
uint32 safety_rx_invalid
uint32 tx_buffer_overflow
uint32 rx_buffer_overflow
uint32 spi_error_count

float32 interrupt_load      # 0..1
uint8 fan_power             # commanded fan power [%]
uint16 fan_speed_rpm
uint16 sbu1_voltage_mv
uint16 sbu2_voltage_mv

CanBusHealth[] can_health
