import can
import time

'''
This function aims to assign command for motor.
Args:
    control : string  control type
    val : int control value
return list, that will be transmitted
'''
def get_data(control, val):
    data = [0]*8

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
        'torque_off': 0x77,
        'torque_on': 0x78
    }
    data[0] = c_option[control]  # data[0] is based on cmd type

    # Last part is 00 cmd, most of them are getting information from motor.
    if control == 'shut_down' or control == 'stop':
        data[1] = data[2] = data[3] = data[4] = data[5] = data[6] = data[7] = 0x00

    elif control == 'torque' or control == 'speed':
        data[1] = data[2] = data[3] = 0x00
        hex_val = val_trans_8(val)  # Value transform
        for i in range(4):
            data[4+i] = hex_val[i]

    elif control == 'position':
        data[1] = 0x00
        # data[2],[3] represent motor speed in dps(degree per second). [3] is high, [2] is low. Here 0x01F4 is 500dps
        data[2] = 0xF4
        data[3] = 0x01
        hex_val = val_trans_8(val)  # Value transform
        for i in range(4):
            data[4+i] = hex_val[i]

    elif control == 'pos_increment':
        data[1] = 0x00
        # data[2],[3] represent motor speed in dps(degree per second). [3] is high, [2] is low. 0x01F4 is 500 dps
        data[2] = 0xF4
        data[3] = 0x01
        hex_val = val_trans_8(val)  # Value transform
        for i in range(4):
            data[4+i] = hex_val[i]

    return data


'''
Fuction for decimal to hex
Args:
    param dec
return list, hexdicemal list
'''
def val_trans_8(dec):
    # According to 0.01 degree/LSB, input value must be 100 times larger.
    dec = dec * 100

    # Negative need to be modified before transformed to hex
    if dec < 0:
        dec = dec+2**32

    # low to high
    hex_data = []
    dec = int(dec)
    str = '{:08X}'.format(dec)
    for i in range(4):
        hex_data.append(int(str[6-i*2:8-i*2], 16))

    return hex_data


'''
Send msg to CAN bus
Args:
    param id
    param control
    param val
'''
def send_msg(bus, id, control, val):
    data = get_data(control, val)

    # Basic message formats
    msg = can.Message(
        arbitration_id=id, data=data, extended_id=False
    )

    try:
        bus.send(msg)
        # print(f"Message sent on {bus.channel_info}")
    except can.CanError:
        print("Message NOT sent")


if __name__ == "__main__":
    # Setting CAN bus
    bus = can.Bus(interface='socketcan', channel='can0', bitrate=1000000)
    # send_msg(bus, 0x141, 'stop', 200)
    # send_msg(bus, 0x142, 'stop', 200)
    send_msg(bus, 0x141, 'position', 0)
    # send_msg(bus, 0x142, 'position', 0)

    # for i in range(10):
    #     send_msg(bus, 0x141, 'speed', 200)
    #     send_msg(bus, 0x142, 'speed', 200)
    #     time.sleep(5)
    #     print("spd over!")

    #     send_msg(bus, 0x141, 'position', 200)
    #     send_msg(bus, 0x142, 'position', 200)
    #     time.sleep(2)
    #     print("pos over!")

    #     send_msg(bus, 0x141, 'status', 200)
    #     send_msg(bus, 0x142, 'status', 200)
    #     time.sleep(3)
    #     print("wait!")

    #     send_msg(bus, 0x141, 'speed', 200)
    #     send_msg(bus, 0x142, 'speed', 200)
    #     send_msg(bus, 0x141, 'angle', 200)
    #     send_msg(bus, 0x142, 'angle', 200)
    #     time.sleep(2)
    #     print("angle!")

    #     send_msg(bus, 0x141, 'stop', 200)
    #     send_msg(bus, 0x142, 'stop', 200)
    #     time.sleep(1)
    #     print("stop!")

    #     send_msg(bus, 0x141, 'pos_increment', 200)
    #     send_msg(bus, 0x142, 'pos_increment', 200)
    #     print("INC!")
    #     time.sleep(10)
