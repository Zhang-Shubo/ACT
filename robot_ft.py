import numpy as np
from typing import Union
from enum import Enum, auto
from ft_sdk.scservo_sdk import *


class MotorControlType(Enum):
    PWM = auto()
    POSITION_CONTROL = auto()
    DISABLED = auto()
    UNKNOWN = auto()

class Robot:
    def __init__(self, device_name: str, baudrate=1_000_000, servo_ids=[1, 2, 3, 4, 5, 6]) -> None:
        self.servo_ids = servo_ids
        self.port_handler = PortHandler(device_name)
        self.packet_handler = scscl(self.port_handler)
        self._init_motors()

    def _init_motors(self):
        self.position_reader = GroupSyncRead(
            self.packet_handler,
            SCSCL_PRESENT_POSITION_L,
            4)
        for id in self.servo_ids:
            scs_addparam_result = self.position_reader.addParam(id)
            if scs_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % scs_id)

        self.velocity_reader = GroupSyncRead(
            self.packet_handler,
            SCSCL_PRESENT_SPEED_L,
            4)
        for id in self.servo_ids:
            self.velocity_reader.addParam(id)
            if scs_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % scs_id)

        self.pos_writer = self.packet_handler.groupSyncWrite
        for id in self.servo_ids:
            self.pos_writer.addParam(id, [2048])

        # for id in self.servo_ids:
        #     self.packet_handler.WritePWM(id, [2048])
        self._disable_torque()
        self.motor_control_state = MotorControlType.DISABLED

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
        for id in self.servo_ids:
            position = self.position_reader.getData(id, SCSCL_GOAL_POSITION_L, 4)
            if position > 2 ** 31:
                position -= 2 ** 32
            positions.append(position)
        return np.array(positions)
    
    def read_velocity(self):
        """
        Reads the joint velocities of the robot.
        :return: list of joint velocities,
        """
        self.velocity_reader.txRxPacket()
        velocties = []
        for id in self.servo_ids:
            velocity = self.velocity_reader.getData(id, SCSCL_GOAL_SPEED_L, 4)
            if velocity > 2 ** 31:
                velocity -= 2 ** 32
            velocties.append(velocity)
        return np.array(velocties)

    def set_goal_pos(self, action):
        """
        :param action: list or numpy array of target joint positions in range [0, 4096]
        """
        if not self.motor_control_state is MotorControlType.POSITION_CONTROL:
            self._set_position_control()
        for i, motor_id in enumerate(self.servo_ids):
            data_write = [action[i]]
            self.pos_writer.changeParam(motor_id, data_write)

        self.pos_writer.txPacket()


    def _disable_torque(self):
        print(f'disabling torque for servos {self.servo_ids}')
        for motor_id in self.servo_ids:
            self.packet_handler.disable_torque(motor_id)

    def _enable_torque(self):
        print(f'enabling torque for servos {self.servo_ids}')
        for motor_id in self.servo_ids:
            self.packet_handler.enable_torque(motor_id)

    def _set_position_control(self):
        self._enable_torque()
        self.motor_control_state = MotorControlType.POSITION_CONTROL


if __name__ == "__main__":
    robot = Robot("/dev/tty.usbserial-140", servo_ids=[1])
    robot.set_goal_pos([1024])