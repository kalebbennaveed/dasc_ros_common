# Mission started
# 1: Initialize Mission pressed in panel
# 0: Otherwise
bool cand_precomp_done 
bool at_charging_station # 
bool cand_timer_state
string mission_mode

# Charging
float64 battery_soc_state
float64 charge_start_time

# Flight status
bool landing
bool charging
bool done_charging
