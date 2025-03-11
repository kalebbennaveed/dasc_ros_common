# Mission started
# 1: Initialize Mission pressed in panel
# 0: Otherwise

# This message contains things which candidate trajectory node will communicate with the setpoint node

bool cand_precomp_done # Precompilation is done_charging
string cand_mode # The status of the mode in the candidate trajectory node
float32[3] charge_position # in meters ENU