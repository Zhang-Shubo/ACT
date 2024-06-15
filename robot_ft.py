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


    def read_position(self, tries=2):
        """
        Reads the joint positions of the robot. 2048 is the center position. 0 and 4096 are 180 degrees in each direction.
        :param tries: maximum number of tries to read the position
        :return: list of joint positions in range [0, 4096]
        """
        result = self.position_reader.txRxPacket()
        if result != 0:
            if tries > 0:
                return self.read_position(tries=tries - 1)
            else:
                print(f'failed to read position!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        positions = []
        for scs_id in self.servo_ids:
            scs_data_result, scs_error = self.position_reader.isAvailable(scs_id, SMS_STS_PRESENT_POSITION_L, 11)
            if scs_data_result == True:
                # Get SCServo#scs_id present position moving value
                scs_present_position = self.position_reader.getData(scs_id, SMS_STS_PRESENT_POSITION_L, 2)
                scs_present_speed = self.position_reader.getData(scs_id, SMS_STS_PRESENT_SPEED_L, 2)
                scs_present_moving = self.position_reader.getData(scs_id, SMS_STS_MOVING, 1)
                # print(scs_present_moving)
                print("[ID:%03d] PresPos:%d PresSpd:%d" % (scs_id, scs_present_position, self.feetech.packetHandler.scs_tohost(scs_present_speed, 15)))
                positions.append(scs_present_position)
        return np.array(positions)
    


if __name__ == "__main__":
    robot = Robot("/dev/tty.usbserial-14140", servo_ids=[1,2,3])
    print(robot.read_position())