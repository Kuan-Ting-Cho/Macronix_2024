import can
import time
import math
from can.interfaces.serial.serial_can import SerialBus

# Convert control option to hex.
# Shut down will cause motor torque off, while stop will be torque on.
c_option = {
    'shut_down': 0x80,  # torque off
    'stop': 0x81,   # still torque on
    'torque': 0xA1,
    'speed': 0xA2,
    'position': 0xA4,
    'pos_increment': 0xA8,
    'angle': 0x92,
    'status': 0x9C,
    'reset': 0x76, # switch between motion and normal mode
    'torque_off': 0x77,
    'torque_on': 0x78
}

# torque 2 current parameter
eqn_a = 56 /481
eqn_b = 45641/120250

class Myactuator():
    def __init__(self,name):
        # Setting CAN bus
        # self.bus = can.Bus(interface='socketcan', channel='can0', bitrate=1000000)
        # self.bus = can.ThreadSafeBus(interface='socketcan', channel='can0', bitrate=1000000)
        self.bus = SerialBus(name, baudrate=2000000, timeout=0.1, rtscts=False)


    def get_data(self, control, val):
        """
        This function aims to assign command for motor.

        Args:
            @ param control : string  control type
            @ param val : int control value
        
        return list, that will be transmitted
        """

        data = [0]*8
        data[0] = c_option[control]  # data[0] is based on cmd type

        # Last part is 00 cmd, most of them are getting information from motor.
        if control == 'shut_down' or control == 'stop':
            data[1] = data[2] = data[3] = data[4] = data[5] = data[6] = data[7] = 0x00

        elif control == 'torque':
            data[1] = data[2] = data[3] = 0x00
            val = self.tor2cur(val)
            hex_val = self.val_trans_16_all(val)  # Value transform
            for i in range(2):
                data[4+i] = hex_val[i]

        elif control == 'speed':
            data[1] = data[2] = data[3] = 0x00
            hex_val = self.val_trans_16_all(val)  # Value transform
            for i in range(4):
                data[4+i] = hex_val[i]

        elif control == 'position':
            data[1] = 0x00
            # data[2],[3] represent motor speed in dps(degree per second). [3] is high, [2] is low. Here 0x01F4 is 500dps
            data[2] = 0xF4
            data[3] = 0x01
            hex_val = self.val_trans_16_all(val)  # Value transform
            for i in range(4):
                data[4+i] = hex_val[i]

        elif control == 'pos_increment':
            data[1] = 0x00
            # data[2],[3] represent motor speed in dps(degree per second). [3] is high, [2] is low. 0x01F4 is 500 dps
            data[2] = 0xF4
            data[3] = 0x01
            hex_val = self.val_trans_16_all(val)  # Value transform
            for i in range(4):
                data[4+i] = hex_val[i]

        return data


    def val_trans_16_all(self, dec):
        """
        Function for decimal to hex list

        Args:
            @ param dec
        
        return list, hexdicemal list
        """
        hex_data = []
        hex_str = self.val_trans_16(dec)

        for i in range(4):
            hex_data.append(int(hex_str[6-i*2:8-i*2], 16))

        return hex_data

    def val_trans_16_motion(self, hex_data, dec, byte):
        """
        Function for decimal to hex list in motion control

        Args:
            @ param hex_data_list
            @ param dec value
            @ param byte (num of bytes)
        
        return list, hexdicemal list
        """
        # Negative need to be modified before transformed to hex
        if dec < 0:
            dec = dec+2**32

        # high to low
        dec = int(dec)
        str = '{:03X}'.format(dec)
        for i in range(byte):
            hex_data.append(int(str[i], 16))


    def val_trans_16(self, dec):
        """
        Function for decimal to hex

        Args:
            @ param dec
        
        return hexdicemal string
        """

        # According to 0.01 degree/LSB, input value must be 100 times larger.
        dec = dec * 100

        # Negative need to be modified before transformed to hex
        if dec < 0:
            dec = dec+2**32

        # low to high
        dec = int(dec)
        str = '{:08X}'.format(dec)

        return str
    
    def tor2cur(self, val):
        """
        Function for torque to current

        Args:
            @ param val
        
        return current in decimal
        """
        return eqn_a*val + eqn_b


    def send_msg(self, id, control, val):
        """
        Send msg to CAN bus

        Args:
            @ param id
            @ param control
            @ param val

        return sending status (0:fail, 1:success)
        """
        send_flag = False
        data = self.get_data(control, val)

        # Basic message formats
        msg = can.Message(
            arbitration_id=id, data=data, is_extended_id=False
        )

        try:
            self.bus.send(msg)
            send_flag = True
            # print(f"Message sent on {self.bus.channel_info}")
        except can.CanError:
            send_flag = False
            print("Message NOT sent")
        
        return send_flag


    def motion_cmd(self, id, pos, vel, t_ff, kp=10, kd=1):
        """
        Send motion cmd msg to CAN bus

        Args:
            @ param id
            @ param position (degree)
            @ param velocity (deg/s)
            @ param torque_feedforword (N-m)
            @ param kp
            @ param kd

        return sending status (0:fail, 1:success)
        """
        send_flag = False

        id = 0x400+id
        string_id = '{:X}'.format(int(id))
        id = int(string_id, 16)

        data = []
        result_cmd = []
        pos = ((pos*math.pi/180) + 12.5) / 25 * 65536
        self.val_trans_16_motion(data, pos, 4)

        vel = ((vel*math.pi/180) + 45) / 90 * 4095
        self.val_trans_16_motion(data, vel, 3)

        kp = kp / 500 * 4095
        self.val_trans_16_motion(data, kp, 3)

        kd = kd / 5 * 4095
        self.val_trans_16_motion(data, kd, 3)

        t_ff = (t_ff + 24) / 48 * 4095
        self.val_trans_16_motion(data, t_ff, 3)

        for i in range(0, 16, 2):
            result_cmd.append(data[i]*16+data[i+1])

        msg = can.Message(
            arbitration_id=id, data=result_cmd, is_extended_id=False
        )

        try:
            self.bus.send(msg)
            send_flag = True
            # print(f"Message sent on {self.bus.channel_info}")
        except can.CanError:
            send_flag = False
            print("Message NOT sent")
        
        return send_flag


    def received_data_process(self, data):   
        """    
        Function for processing received data 

        Args:
            @ param data
        """
        pn = 2**15

        # Motion control
        # if 0xA0 <= data[0] < 0xAA or data[0] == 0x9C:
        if data[0] == 0x9C or (data[0] >= 0xA0 and data[0] < 0xAA):
            deg = ((data[7] << 8)+data[6])
            cur = ((data[3] << 8)+data[2])
            spd = ((data[5] << 8)+data[4])
            
            deg = deg if deg < pn else (deg-2**16)
            cur = cur if cur < pn else (cur-2**16)
            spd = spd if spd < pn else (spd-2**16)

            # print("Temptuare: ", data[1])
            # print("Current: ", cur*0.01, "A")
            # print("Speed: ", spd, "dps (output)")
            # print("Degree: ", deg)

            try:
                torque = (cur - eqn_b)/eqn_a
                # print("Torque: ", torque)
            except:
                torque = 0
                # print("Torque: ", "unknown")

        elif data[0] == 0x77:
            print("Success torque off !!!")

        elif data[0] == 0x78:
            print("Success torque on !!!")
        
        elif data[0] == 0x80:
            print("Stop! Torque off!")

        elif data[0] == 0x81:
            print("Stop! Torque on!")

        # Asking
        elif data[0] < 0xA0:
            pos = (data[7] << 24)+(data[6] << 16)+(data[5] << 8)+data[4]
            pos *= 0.01
            print("Position: ", pos, " degree")
        
        return [deg,spd,cur*0.01]


    def receive(self):
        """
        Receive reply message.
        """
    
        # Using specific buses works similar:
        # bus = can.Bus(interface='socketcan', channel='can0', bitrate=1000000)

        while (1):
            msg = self.bus.recv()
            if (msg.arbitration_id > 0x240 and msg.arbitration_id < 0x300):
                id_str = '{:03X}'.format(msg.arbitration_id)
                # print("********ID:", id_str, "********")
                id_str=int(id_str,16)-0x240
                return [id_str]+self.received_data_process(msg.data)
