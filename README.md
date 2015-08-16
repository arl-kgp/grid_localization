# quad-autopilot

This repository holds the ros package for the Autopilot of our Quadrotor Project

Package name: autopilot
<br>Node name: autopilot_node

<pre>
<p>
Topics its subscribes to:
<br>  */ardrone/bottom/image_raw : For the camera
<br>  */ardrone/navdata          : For the quad altitude
<br>  */autopilot/command        : To publish commands for the autopilot (std_msgs/String)
<br>  */autopilot/cmd_vel        : To publish manual velocity to the quad which overrides the autopilot (geometry_msgs/Twist)
<br>  */autopilot/coordinate     : To publish target grid coordiate for grid following (geometry_msgs/Point32)
</p>

<p>
Topics its publishes to:
<br>  */ardrone/takeoff 
<br>  */ardrone/reset          
<br>  */ardrone/land      
<br>  */quad/cmd_vel 
</p>
</pre>

Autopilot commands:
<br>
<pre>
<br>to            :   Take off
<br>ld            :   Land
<br>reset         :   Reset the quad and autopilot
<br>holdalt       :   Go to and hold a defined altitude
<br>up/down       :   Increase or decrease the holding altitude by 50cm
<br>holdnode      :   Go to the nearest node and hold over it
<br>stopholdalt   :   Stop holding altitude
<br>stopholdnode  :   Stop holding node
<br>grid          :   Enable grid follow
<br>stopgrid      :   Disable grid follow
<br>of            : Autopilot master off
</pre>

Example: Basic run for grid following to reach 3,2 node taking nearest node as origin
<br>rostopic pub -1 autopilot/command std_msgs/String "to"
<br>rostopic pub -1 autopilot/command std_msgs/String "holdalt"
<br>rostopic pub -1 autopilot/command std_msgs/String "holdnode"
<br>
<br>rostopic pub -1 autopilot/coordinate geometry_msgs/Point32 -- 3 2 0
<br>rostopic pub -1 autopilot/command std_msgs/String "grid"
