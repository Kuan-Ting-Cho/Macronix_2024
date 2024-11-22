# MyActuator_motor
This is the basic control of MyActuator motor using CAN bus communiccation. 
Basic setup and tutorials are in HackMD (https://hackmd.io/TwwChWEaTb-me8Ldjc6JkA?view).

NOTE: If you want to use Waveshare USB-CAN-A, then you neet to modify the file "serial_can.py" in original CAN package.
This is because Waveshare USB-CAN-A has unique frame format that is not supported in original CAN package. The CAN package I used in 2023/12/22 is python_can 4.3.1, there are some differences between each version. Be careful to use. And... good luck!


The latest pre-release can be installed with:
 '''
 pip install --upgrade --pre python-can
 '''