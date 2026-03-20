"""
File used to store global variables.

Notes
-----
File is currenly being used for TurtleBot3 Burgerbots.

L x W x H is 138mm x 178mm x 192 mm as per https://robotis.us/turtlebot-3-burger-rpi4-4gb-us/?srsltid=AfmBOopVrxRs5QI4pK4bpUb24-oybPMQE-vqyCv-mMc6h-KLMzfUwSD6.

Will use a radius of 178/2 mm -> 89mm -> 0.089m with an added buffer of radius + 10% of radius. 
"""

ENV_X_BOUNDS = (0,50) 
ENV_Y_BOUNDS = (0,50)
BOUNDS = (ENV_X_BOUNDS, ENV_Y_BOUNDS)

START = (45, 45)
GOAL = (5, 5)

ROBOT_RADIUS = 0.089 # meters
RADIUS_BUFFER = ROBOT_RADIUS * 0.1
BUFFER = ROBOT_RADIUS + RADIUS_BUFFER # in meters

WORLD_SCALE = 0.1 # world_units / env_units

CHAR_LIMIT = 25 # for qr codes
STEP_SIZE = 5 # used for consistency 