# Actual feedback only, never synthesize completion from sent requests.
# Transport stamps ROS receipt time and rejects stale/replayed wire feedback.
std_msgs/Header header
uint64 accepted_request_id
# Mode values match MotionControlCommand.
uint8 chassis_mode
uint8 gimbal_mode
bool valid
bool stopped
bool centered
bool folded
bool unfolded
bool gimbal_ready
bool fault
geometry_msgs/Twist velocity
string reason
