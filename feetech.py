import os

from dataclasses import dataclass
from ft_sdk.scservo_sdk import *


class FeeTechSCS:

    @dataclass
    class Config:
        def instantiate(self): return FeeTechSCS(self)

        baudrate: int = 1_000_000
        device_name: str = ''  # /dev/tty.usbserial-1120'


    def __init__(self, config):
        self.config = config
        self.connect()

    def connect(self):
        if self.config.device_name == '':
            for port_name in os.listdir('/dev'):
                if 'ttyUSB' in port_name or 'ttyACM' in port_name:
                    self.config.device_name = '/dev/' + port_name
                    print(f'using device {self.config.device_name}')
        self.portHandler = PortHandler(self.config.device_name)
        self.packetHandler = scscl(self.portHandler)

        if not self.portHandler.openPort():
            raise Exception(f'Failed to open port {self.config.device_name}')
        else:
            print("Succeeded to open the port")

        if not self.portHandler.setBaudRate(self.config.baudrate):
            raise Exception(f'failed to set baudrate to {self.config.baudrate}')
        else:
            print("Succeeded to change the baudrate")
        

        self.operating_modes = [None for _ in range(32)]
        self.torque_enabled = [None for _ in range(32)]
        return True
    
    def _process_response(self, scs_comm_result: int, scs_error: int, motor_id: int):
        if scs_comm_result != COMM_SUCCESS:
            raise ConnectionError(
                f"scs_comm_result for motor {motor_id}: {self.packetHandler.getTxRxResult(scs_comm_result)}")
        elif scs_error != 0:
            print(f'dxl error {scs_error}')
            raise ConnectionError(
                f"dynamixel error for motor {motor_id}: {self.packetHandler.getTxRxResult(scs_error)}")

    def _read_value(self, motor_id, address, num_bytes: int, tries=10):
        try:
            if num_bytes == 1:
                value, scs_comm_result, scs_error = self.packetHandler.read1ByteTxRx(motor_id,
                                                                                     address)
            elif num_bytes == 2:
                value, scs_comm_result, scs_error = self.packetHandler.read2ByteTxRx(motor_id,
                                                                                     address)
            elif num_bytes == 4:
                value, scs_comm_result, scs_error = self.packetHandler.read4ByteTxRx(motor_id,
                                                                                     address)
        except Exception:
            if tries == 0:
                raise Exception
            else:
                return self._read_value(motor_id, address, num_bytes, tries=tries - 1)
        if scs_comm_result != COMM_SUCCESS:
            if tries <= 1:
                # print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
                raise ConnectionError(f'scs_comm_result {scs_comm_result} for servo {motor_id} value {value}')
            else:
                print(f'dynamixel read failure for servo {motor_id} trying again with {tries - 1} tries')
                time.sleep(0.02)
                return self._read_value(motor_id, address, num_bytes, tries=tries - 1)
        elif scs_error != 0:  # # print("%s" % self.packetHandler.getRxPacketError(scs_error))
            # raise ConnectionError(f'scs_error {scs_error} binary ' + "{0:b}".format(37))
            if tries == 0 and scs_error != 128:
                raise Exception(f'Failed to read value from motor {motor_id} error is {scs_error}')
            else:
                return self._read_value(motor_id, address, num_bytes, tries=tries - 1)
        return value
    
    def write_pos(self, motor_id, position, time=0, speed=1000):
        # txpacket = [self.packetHandler.scs_lobyte(position), self.packetHandler.scs_hibyte(position), self.packetHandler.scs_lobyte(time), self.packetHandler.scs_hibyte(time), self.packetHandler.scs_lobyte(speed), self.packetHandler.scs_hibyte(speed)]
        # scs_comm_result, scs_error = self.packetHandler.writeTxRx(motor_id, SCSCL_GOAL_POSITION_L, len(txpacket), txpacket)
        scs_comm_result, scs_error = self.packetHandler.WritePos(motor_id, position, time, speed)
        
        self._process_response(scs_comm_result, scs_error, motor_id)

    def get_moving(self, motor_id):
        moving, scs_comm_result, scs_error = self.packetHandler.ReadMoving(motor_id)
        self._process_response(scs_comm_result, scs_error, motor_id)
        return moving



class FeeTechSTS:

    @dataclass
    class Config:
        def instantiate(self): return FeeTechSTS(self)

        baudrate: int = 1_000_000
        device_name: str = ''  # /dev/tty.usbserial-1120'

    def __init__(self, config: Config):
        self.config = config
        self.connect()

    def connect(self):
        if self.config.device_name == '':
            for port_name in os.listdir('/dev'):
                if 'ttyUSB' in port_name or 'ttyACM' in port_name:
                    self.config.device_name = '/dev/' + port_name
                    print(f'using device {self.config.device_name}')
        self.portHandler = PortHandler(self.config.device_name)
        self.packetHandler = sms_sts(self.portHandler)

        if not self.portHandler.openPort():
            raise Exception(f'Failed to open port {self.config.device_name}')
        else:
            print("Succeeded to open the port")

        if not self.portHandler.setBaudRate(self.config.baudrate):
            raise Exception(f'failed to set baudrate to {self.config.baudrate}')
        else:
            print("Succeeded to change the baudrate")
        

        self.operating_modes = [None for _ in range(32)]
        self.torque_enabled = [None for _ in range(32)]
        return True
    
    def _process_response(self, scs_comm_result: int, scs_error: int, motor_id: int):
        if scs_comm_result != COMM_SUCCESS:
            raise ConnectionError(
                f"scs_comm_result for motor {motor_id}: {self.packetHandler.getTxRxResult(scs_comm_result)}")
        elif scs_error != 0:
            print(f'dxl error {scs_error}')
            raise ConnectionError(
                f"dynamixel error for motor {motor_id}: {self.packetHandler.getTxRxResult(scs_error)}")
    
    def write_pos(self, motor_id, position, speed=1000, acc=50):
        # txpacket = [self.packetHandler.scs_lobyte(position), self.packetHandler.scs_hibyte(position), self.packetHandler.scs_lobyte(time), self.packetHandler.scs_hibyte(time), self.packetHandler.scs_lobyte(speed), self.packetHandler.scs_hibyte(speed)]
        # scs_comm_result, scs_error = self.packetHandler.writeTxRx(motor_id, SCSCL_GOAL_POSITION_L, len(txpacket), txpacket)
        scs_comm_result, scs_error = self.packetHandler.WritePosEx(motor_id, position, speed, acc)
        
        self._process_response(scs_comm_result, scs_error, motor_id)

    def write_id(self, old_id, new_id):
        # txpacket = [self.packetHandler.scs_lobyte(position), self.packetHandler.scs_hibyte(position), self.packetHandler.scs_lobyte(time), self.packetHandler.scs_hibyte(time), self.packetHandler.scs_lobyte(speed), self.packetHandler.scs_hibyte(speed)]
        # scs_comm_result, scs_error = self.packetHandler.writeTxRx(motor_id, SCSCL_GOAL_POSITION_L, len(txpacket), txpacket)
        scs_comm_result, scs_error = self.packetHandler.write1ByteTxRx(old_id, SMS_STS_ID, new_id)
        
        self._process_response(scs_comm_result, scs_error, old_id)


    def get_moving(self, motor_id):
        moving, scs_comm_result, scs_error = self.packetHandler.ReadMoving(motor_id)
        self._process_response(scs_comm_result, scs_error, motor_id)
        return moving

    def get_speed_and_pres_pos(self, motor_id):
        pre_speed, pre_pos, scs_comm_result, scs_error = self.packetHandler.ReadPosSpeed(motor_id)
        self._process_response(scs_comm_result, scs_error, motor_id)
        return pre_speed, pre_pos

    

if __name__ == "__main__":
    feetech_instant = FeeTechSTS.Config(
        baudrate=1_000_000,
        device_name='/dev/tty.usbserial-14140'
    ).instantiate()
    # feetech_instant.read_position(1)
    # feetech_instant.packetHandler.WritePos(1, 3000, 0, 400)
    # feetech_instant.write_pos(1, 1800, 0, 1200)
    ID=3
    print(feetech_instant.get_speed_and_pres_pos(ID))
    # feetech_instant.write_id(ID, 1)
    # feetech_instant.portHandler.closePort()
    exit(0)
    feetech_instant.write_pos(ID, 4000, 500, 50)
    # feetech_instant.set_goal_position(1, 1000)
    while 1:

        pres_pos, speed = feetech_instant.get_speed_and_pres_pos(ID)
        moving = feetech_instant.get_moving(ID)
        print(speed, pres_pos)
        if moving == 0:
            break
    feetech_instant.portHandler.closePort()
    # feetech_instant.read_position(1)
    # 
    # feetech_instant.disable_torque(1)
    # pos = feetech_instant.read_position(1)
    # print(pos)
    # motor_id = 1
    # pos = feetech_instant.read_position(motor_id)
    # for i in range(10):
    #     s = time.monotonic()
    #     pos = feetech_instant.read_position(motor_id)
    #     delta = time.monotonic() - s
    #     print(f'read position took {delta}')
    #     print(f'position {pos}')