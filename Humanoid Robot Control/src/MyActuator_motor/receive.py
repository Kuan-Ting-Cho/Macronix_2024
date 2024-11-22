import can
import math
from can.interfaces.serial.serial_can import SerialBus


def data_process(data):
    pn = 2**15

    # Motion control
    if 0xA0 <= data[0] < 0xAA or data[0] == 0x9C:
        deg = ((data[7] << 8)+data[6]) if ((data[7] << 8)+data[6]
                                    ) < pn else ((data[7])+data[6]-2**16)
  
        cur = ((data[3] << 8)+data[2]) if ((data[3] << 8)+data[2]
                                            ) < pn else ((data[3] << 8)+data[2]-2**16)
        spd = ((data[5] << 8)+data[4]) if ((data[5] << 8)+data[4]
                                            ) < pn else ((data[5] << 8)+data[4]-2**16)
        deg = ((data[7] << 8)+data[6]) if ((data[7] << 8)+data[6]
                                            ) < pn else ((data[7] << 8)+data[6]-2**16)
        
        print("Temptuare: ", data[1])
        print("Current: ", cur*0.01, "A")
        print("Speed: ", spd, "dps (output)")
        print("Degree: ", deg)

        try:
            torque = cur*4320/(math.pi*spd)
            print("Torque: ", torque)
        except:
            print("Torque: ", "unknown")
            

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


def receive():
    """Receive reply message."""
    # Using specific buses works similar:
    bus = SerialBus('/dev/ttyUSB0', baudrate=2000000, timeout=0.1, rtscts=False)

    print("OK")

    while (1):
        msg = bus.recv()
     

        if (msg.arbitration_id > 0x240 and msg.arbitration_id < 0x300) or msg.arbitration_id > 0x500:
            id_str = '{:03X}'.format(msg.arbitration_id)
            print("********ID:", id_str, "********")
            data_process(msg.data)

        else:
            print(msg)


if __name__ == "__main__":
    receive()
