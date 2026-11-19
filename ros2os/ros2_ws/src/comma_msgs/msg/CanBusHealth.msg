# Error counters and configuration of one panda CAN controller.

uint8 bus
bool bus_off
uint32 bus_off_cnt
bool error_warning
bool error_passive
string last_error         # e.g. "noError", "stuffError", "formError", "ackError"
uint8 receive_error_cnt
uint8 transmit_error_cnt
uint32 total_error_cnt
uint32 total_tx_lost_cnt
uint32 total_rx_lost_cnt
uint32 total_tx_cnt
uint32 total_rx_cnt
uint32 total_fwd_cnt
uint16 can_speed_kbps
uint16 can_data_speed_kbps
bool canfd_enabled
bool brs_enabled
bool canfd_non_iso
uint32 irq0_call_rate
uint32 irq1_call_rate
uint32 irq2_call_rate
uint32 can_core_reset_cnt
