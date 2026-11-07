# ROS contract, not serial wire layout. Velocity in base_frame: x forward,
# y left, z CCW, m/s and rad/s. Header is command generation time.
std_msgs/Header header
uint64 request_id
uint8 SOURCE_NONE=0
uint8 SOURCE_NAV2=1
uint8 SOURCE_TUNNEL=2
uint8 source
uint8 CHASSIS_STOP=0
uint8 CHASSIS_SPIN=1
uint8 CHASSIS_FOLLOW=2
uint8 CHASSIS_NAV_DIRECT=3
uint8 chassis_mode
uint8 GIMBAL_AUTO=0
uint8 GIMBAL_CENTER=1
uint8 GIMBAL_STOW=2
uint8 GIMBAL_HOLD=3
uint8 gimbal_mode
uint8 POSTURE_KEEP=0
uint8 POSTURE_FOLD=1
uint8 POSTURE_UNFOLD=2
uint8 posture_request
geometry_msgs/Twist velocity
# False: stop, no spin, no new actuator action or firing.
bool command_valid
bool motion_allowed
# Future transport must AND this with the independent autoaim fire request.
bool allow_fire
string reason
