import numpy as np
from robot import Robot
from robot_ft import Robot as RobotFT

def get_bias():
    length = len(leader.read_position())
    leader_pos = leader.read_position()
    print(f"leader_pos: {leader_pos}")
    follower_pos = follower.read_position()
    print(f"follower_pos: {follower_pos}")
    bias = [leader_pos[i] - follower_pos[i] for i in range(length)]
    print(bias)
    return bias


# init robots
# leader = Robot(device_name="COM4", servo_ids=[1,2,3,4,5,6])
# follower = RobotFT(device_name="COM5", servo_ids=[1,2,3,4,5,6])

leader = Robot(device_name="/dev/ttyACM0", servo_ids=[1,2,3,4,5,6])
follower = RobotFT(device_name="/dev/ttyUSB0", servo_ids=[1,2,3,4,5,6])

# activate the leader gripper torque

# leader._disable_torque()
get_bias()
exit()
leader.set_trigger_torque(value=-200)
# leader._disable_torque()
bias = np.array([1098, 1123, 88, 1556, -189, -916])

while True:
    # print(leader.read_position()-bias)
    action = leader.read_position()-bias
    print(action)
    follower.set_goal_pos(action)

leader._disable_torque()
#follower._disable_torque()



