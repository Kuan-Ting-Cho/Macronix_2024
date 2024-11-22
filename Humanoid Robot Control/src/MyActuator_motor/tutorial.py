import can


def send_one():
    """Sends a single message."""

    # Using specific buses works similar:
    bus = can.Bus(interface='socketcan', channel='can0', bitrate=1000000)

    # Basic message formats
    msg = can.Message(
        arbitration_id=0x141, data=[0xA4, 0x00, 0xF4, 0x01, 0xA0, 0x8C, 0x00, 0x00], extended_id=False
    )

    try:
        bus.send(msg)
        print(f"Message sent on {bus.channel_info}")
    except can.CanError:
        print("Message NOT sent")



if __name__ == "__main__":
    send_one()
    send_one()