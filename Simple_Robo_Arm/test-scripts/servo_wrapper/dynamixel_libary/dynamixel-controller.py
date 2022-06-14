from __future__ import annotations
import dynio as dynamixel
import json
import time
import pkg_resources


dxl_io = dynamixel.dxl.DynamixelIO(
    device_name='COM4',
    baud_rate=57600,
    ) # your port for U2D2 or other serial device


def new_xm430_W210(dxl_id, communication_obj):
    """Returns a new DynamixelMotor object for an XM430-W210"""
    # obj = pkg_resources.resource_filename(__name__, "../json/XM430-W210.json"),   # 
    obj = './dynamixel_json/XM430-W210.json'
    motor = dynamixel.dxl.DynamixelMotor(
        dxl_id,                                                                 # integer
        communication_obj,                                                      # DynamixelIO object
        obj,
        protocol=2,
        # control_table_protocol=None,                                          # this gets set inside the class later by protocol
        )
    motor.set_position_mode()
    return motor


servo = new_xm430_W210(2, dxl_io)

# servo.read_control_table("Values")

DEGREES_PER_PULSE = 0.088

servo.torque_enable()

"""NOTE set_angle does not seem to be working properly. Tried the below test 
routine and it only seemed to move to 0. But read works ok..."""

try:
    pos = None
    while 1:
        # perhaps better to make one read call and then reference the local control table?
        time.sleep(.25)
        servo.set_position(4095)
        # servo.set_angle(360)
        print(f'Position set to {360} deg')
        while pos is None or pos < 360 -1:
            pos = servo.get_angle()
            print(f'Position: {pos} deg')
            time.sleep(0.0001)

        print('')
        time.sleep(.25)
        servo.set_position(0)
        # servo.set_angle(0)
        print(f'Position set to {0} deg')
        while pos > 1:
            pos = servo.get_angle()
            print(f'Position: {pos} deg')
            time.sleep(0.0001)
        print('')
finally:
    servo.torque_disable()

