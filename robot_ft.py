import numpy as np
from typing import Union
from enum import Enum, auto
from ft_sdk.scservo_sdk import *
from feetech import FeeTechSTS


class MotorControlType(Enum):
    PWM = auto()
    POSITION_CONTROL = auto()
    DISABLED = auto()
    UNKNOWN = auto()

class Robot:
    def __init__(self, device_name: str, baudrate=1_000_000, servo_ids=[1, 2, 3, 4, 5, 6]) -> None:
        self.servo_ids = servo_ids
        self.port_handler = PortHandler(device_name)
        self.feetech =  FeeTechSTS.Config(device_name=device_name, baudrate=baudrate).instantiate()
        self._init_motors()

    def _init_motors(self):
        self.position_reader = GroupSyncRead(
            self.feetech.packetHandler,
            SMS_STS_PRESENT_POSITION_L,
            11)
        for scs_id in self.servo_ids:
            scs_addparam_result = self.position_reader.addParam(scs_id)
            if scs_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % scs_id)

    
    def read_data(self, tries=2):
        """
        Reads the joint positions of the robot. 2048 is the center position. 0 and 4096 are 180 degrees in each direction.
        :param tries: maximum number of tries to read the position
        :return: list of joint positions in range [0, 4096]
        """
        result = self.position_reader.txRxPacket()
        if result != 0:
            if tries > 0:
                return self.read_data(tries=tries - 1)
            else:
                print(f'failed to read position!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        positions = []
        speeds = []
        movings = []
        for scs_id in self.servo_ids:
            scs_data_result, scs_error = self.position_reader.isAvailable(scs_id, SMS_STS_PRESENT_POSITION_L, 11)
            if scs_data_result == True:
                # Get SCServo#scs_id present position moving value
                scs_present_position = self.position_reader.getData(scs_id, SMS_STS_PRESENT_POSITION_L, 2)
                scs_present_speed = self.position_reader.getData(scs_id, SMS_STS_PRESENT_SPEED_L, 2)
                scs_present_moving = self.position_reader.getData(scs_id, SMS_STS_MOVING, 1)
                # print(scs_present_moving)
                # print("[ID:%03d] PresPos:%d PresSpd:%d" % (scs_id, scs_present_position, self.feetech.packetHandler.scs_tohost(scs_present_speed, 15)))
                positions.append(scs_present_position)
                speeds.append(scs_present_speed)
                movings.append(scs_present_moving)
        return np.array(positions), np.array(speeds), np.array(movings)


    def read_position(self, tries=2):
        positions, speeds, movings = self.read_data(tries=tries)
        return positions
    
    def read_velocity(self, tries=2):
        positions, speeds, movings = self.read_data(tries=tries)
        return np.array(speeds)
    
    def _disable_torque(self):
        print(f'disabling torque for servos {self.servo_ids}')
        for motor_id in self.servo_ids:
            self.feetech._disable_torque(motor_id)

    def set_goal_pos(self, action):
        """
        :param action: list or numpy array of target joint positions in range [0, 4096]
        """
        for i, scs_id in enumerate(self.servo_ids):
            # Add SCServo#1~10 goal position\moving speed\moving accc value to the Syncwrite parameter storage
            scs_addparam_result = self.feetech.packetHandler.SyncWritePosEx(scs_id, action[i], 0, 0)
            if scs_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % scs_id)

        # Syncwrite goal position
        scs_comm_result = self.feetech.packetHandler.groupSyncWrite.txPacket()
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.feetech.packetHandler.getTxRxResult(scs_comm_result))

        # Clear syncwrite parameter storage
        self.feetech.packetHandler.groupSyncWrite.clearParam()
    


if __name__ == "__main__":
    # robot = Robot("/dev/tty.usbserial-14140", servo_ids=[1,2,3])

    robot = Robot("/dev/ttyUSB0", servo_ids=[1,2,3,4,5,6])
    # robot._disable_torque()
    # current_pos = robot.read_position()
    # for i in range(100):
    #     print(robot.read_position())
    #     time.sleep(.1)
    # print(current_pos)
    # print(current_pos - 20)
    # robot.set_goal_pos(current_pos + 60)
    # time.sleep(2)
    # current_pos = robot.read_position()
    # print(current_pos)
    for i in range(10):
        s = time.monotonic()
        pos = robot.read_data()
        delta = time.monotonic() - s
        print(f'read position took {delta}')
        print(f'position {pos}')

