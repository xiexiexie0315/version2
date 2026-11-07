# Periodic HNU intent. Coordinator never advances the workflow itself.
std_msgs/Header header
uint64 request_id
uint8 NORMAL=0
uint8 APPROACH=1
uint8 DETECT=2
uint8 ALIGN=3
uint8 FOLD=4
uint8 VERIFY=5
uint8 TRAVERSE=6
uint8 EXIT=7
uint8 UNFOLD=8
uint8 COMPLETE=9
uint8 FALLBACK=10
uint8 ABORT=11
uint8 phase
