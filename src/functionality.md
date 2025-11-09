



<-----------for twist mux------------>
added twist_mux for dynamic velocity commands

humbot controller gets Twist msgs(cmd_vel_unstamped)                and publishes TwistStamped (cmd_vel)
joystick          gets TwistStamped msgs(input_joy/cmd_vel_stamped) and publishes Twist (input_joy/cmd_vel)


<---------------for safety stop-------->

danger dist <= 0.2 
warning_distance <= 0.6

for using warnig dist, we have to create action client, the twist_mux interface(turbo vala) is itself a ros2 action server.
for using twist mux interface , have to include <twist_mux_msgs/action/joy_turbo.hpp> 

will not move untill and unless joy_turbo_dec/inc(since this is action server) is available



for action server, client 
used msgs to define 3 msgs that define interface with action

<---------------action client---------->
options for sending goal, send_goal_ options is action name and then inside it we are giving 3 options to it response, feedback, result, 
and also writing functs for the same


lifecycle node steps


<---------------nav2 map server--------------------->
it is implemented as lifecycle node so to start it we need to configure it

so if we have map, we first need to set /lifecycle node to 1 (on_configure mode) and then to 3 (on_active mode)
this will load the map.

but this map would not be visible in rviz, here comes QOS 
if the qos field configured for subs is diff than pub then issues arise,

For our implementation used -- Reliability and Durability



<-------------mapping with known poses-------------->
origin in centre of map, if we write 0,0 thenit will start at top right corner of map
using odometry to map our environment and localize the robot
data = vector of int , all cells are vectors containnnf staus of cell (free, occupied, unknown (-1) )

taking input form scan topic laserscan msgs

timer_ func will publish the map every second


<-------mappingg ------------->
occupancy grid method used and then optimised it by using probabilistic occupancy grid
1st we were directly working with laser scan values and assigning the cells a value

but what if noise is present and gives contradicting readings?
then we are using prob occupancy grid




<----------lifecycle nodes------------->
takes care of both map server and amcl nodes


<-------------------------------------PD motion planner----------------------------------------------------->


