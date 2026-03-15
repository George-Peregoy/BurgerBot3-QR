
Will format later

In RRT#, check collision is hardcoded to assume robot radius of 
TurtleBot3 burger bot. If using a differnt bot, change.
L x W x H is 138mm x 178mm x 192 mm as per https://robotis.us/turtlebot-3-burger-rpi4-4gb-us/?srsltid=AfmBOopVrxRs5QI4pK4bpUb24-oybPMQE-vqyCv-mMc6h-KLMzfUwSD6.

Start, goal, bounds, robot size, env to world factor are all located in src/path_planning/path_planning/config.py

Make world number launch arg, fix env_to_world to accomadate as well as pose_publihser