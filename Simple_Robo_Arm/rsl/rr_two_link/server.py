#!/usr/bin/env python
"""
@author Bruce Iverson
"""

# https://www.includehelp.com/python/implementation-of-websocket-using-socket-io-in-python.aspx
from __future__ import annotations
import aiohttp
from roboticstoolbox import robot
import socketio         # new versions threw errors, worked with 4.3.1
# engine is 3.9.3
import os
import asyncio
import logging
import math
import traceback
import numpy as np
from socketio import server

try:
    from . import camera
except:
    import camera
import time
from typing import Optional

import sys
sys.path.append(sys.path[0] + '/../..')

import robotics.robot as robotics
from robotics.controllers import ControlTypes
import util

# https://medium.com/knerd/best-practices-for-python-dependency-management-cc8d1913db82
# https://pip.pypa.io/en/stable/user_guide/#requirements-files
# https://note.nkmk.me/en/python-pip-install-requirements/
# https://docs.python-guide.org/writing/structure/

# create a new aysnc socket io server https://stackoverflow.com/questions/57579110/how-to-fix-access-control-allow-origin-error-in-a-python-socket-io-server
socket_io = socketio.AsyncServer(cors_allowed_origins='*')  # async_mode='`aiohttp`')

# create a new Aiohttp aiohttp.web application
web_app = aiohttp.web.Application()
routes = aiohttp.web.RouteTableDef()

# bind the socket.io server to the aiohttp.web application instance
socket_io.attach(web_app)


async def shut_down(app):
    return
    # not sure what's wrong with this, but it threw some errors
    for ws in set(app['websockets']):
        await ws.close(code=aiohttp.WSCloseCode.GOING_AWAY, message="Server shutdown")


##########################################################
########### Set up logging with socket emitting ##########
##########################################################
loop = asyncio.get_event_loop()

socket_logging_format = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')


class SocketIOHandler(logging.Handler):
    def emit(self, record:logging.LogRecord):
        log_msg = socket_logging_format.format(record)      # this is necessary for some reason
        global loop
        loop.create_task(socket_io.emit(
            'log', 
            (record.asctime,
            record.name,
            record.levelname,
            record.getMessage())
        ))


server_logger = util.get_simple_logger('server', verbosity=logging.INFO)
socket_logging_handler = SocketIOHandler()
socket_logging_handler.setFormatter(socket_logging_format)  # this didn't seem to work
server_logger.addHandler(socket_logging_handler)


@routes.get('/')
async def landing(request):
    path_to_this_file = os.path.dirname(os.path.abspath(__file__))
    filename = os.path.join(path_to_this_file, 'web/landing.html')
    with open(filename) as file_obj:
        return aiohttp.web.Response(text = file_obj.read(), content_type='text/html')


@routes.get('/master')
async def master(request):
    path_to_this_file = os.path.dirname(os.path.abspath(__file__))
    filename = os.path.join(path_to_this_file, 'web/master.html')
    with open(filename) as file_obj:
        return aiohttp.web.Response(text = file_obj.read(), content_type='text/html')


@routes.get('/assignment_1')
async def assignment_1(request):
    path_to_this_file = os.path.dirname(os.path.abspath(__file__))
    filename = os.path.join(path_to_this_file, 'web/assignment_1.html')
    with open(filename) as file_obj:
        return aiohttp.web.Response(text = file_obj.read(), content_type='text/html')

@routes.get('/exercise_1')
async def exercise_1(request):
    path_to_this_file = os.path.dirname(os.path.abspath(__file__))
    filename = os.path.join(path_to_this_file, 'web/pages/exercise_1.html')
    with open(filename) as file_obj:
        return aiohttp.web.Response(text = file_obj.read(), content_type='text/html')

@routes.get('/exercise_2')
async def exercise_2(request):
    path_to_this_file = os.path.dirname(os.path.abspath(__file__))
    filename = os.path.join(path_to_this_file, 'web/pages/exercise_2.html')
    with open(filename) as file_obj:
        return aiohttp.web.Response(text = file_obj.read(), content_type='text/html')

@socket_io.event
async def connect(id, information):
    server_logger.info(f'Server connected to client id {id}.')
    await send_telemtry()


@socket_io.event
async def disconnect(id):
    server_logger.info(f'Server detected client {id} has disconnected.')


@socket_io.event
async def home(id):
    server_logger.info(f'Received HOME command from id {id}')
    robot_arm.set_joint_pose(np.array([0,0]))


@socket_io.event
async def controls_refresher(id):
    server_logger.info(f'Received CONTROLS REFRESHER DEMO 1 command from id {id}')
    if robot_arm.doing_demo:
        server_logger.warning('Robot is already doing a demo.')
        return
    robot_arm.doing_demo = True

    robot_arm.set_control_mode(ControlTypes.ACTUATOR_POSITION_CONTROL)
    
    robot_arm.set_motor_gains(0, 125, 0)
    robot_arm.set_motor_gains(1, 125, 0)
    if not robot_arm.on:
        robot_arm.torque_enable()

    robot_arm.set_joint_pose((0, 0))
    await asyncio.sleep(1)

    robot_arm.set_joint_pose((math.pi/2, 0))
    await asyncio.sleep(1)
    robot_arm.set_joint_pose((0, 0))
    await asyncio.sleep(1)

    robot_arm.set_motor_gains(0, 80, 0)
    robot_arm.set_joint_pose((math.pi/2, 0))
    await asyncio.sleep(1)
    robot_arm.set_joint_pose((0, 0))
    await asyncio.sleep(1)
    
    robot_arm.set_motor_gains(0, 40, 0)
    robot_arm.set_joint_pose((math.pi/2, 0))
    await asyncio.sleep(1)
    robot_arm.set_joint_pose((0, 0))
    await asyncio.sleep(1)

    robot_arm.doing_demo = False


@socket_io.event
async def workspace_demo(id, theta_1_min, theta_1_max, theta_2_min, theta_2_max):
    server_logger.info(f'Received WORKSPACE_DEMO command from id {id}')

    if robot_arm.doing_demo:
        robot_arm.logger.error(f'Cannot do WORKSPACE_DEMO while another demo is active.')
        return
    robot_arm.logger.info('Begin workspace demo')
    robot_arm.doing_demo = True
    robot_arm.set_control_mode(ControlTypes.ACTUATOR_POSITION_CONTROL)
    robot_arm.set_motor_gains(0, 80, 0)
    robot_arm.set_motor_gains(1, 80, 0)
    theta_1 = math.radians(float(theta_1_min))
    resolution = math.pi/8
    sleep_time = 0.125
    try:
        robot_arm.set_joint_pose((theta_1, -theta_2_min))
        await asyncio.sleep(1.5)
        direction = 1
        while not robot_arm._actuators[0].near_max_position and theta_1 < robot_arm._actuators[0].max_theta:
            await robot_arm._actuators[1].discrete_sweep(direction, resolution, sleep_time)
            robot_arm._actuators[0].set_theta(theta_1)
            await asyncio.sleep(sleep_time)
            theta_1 += resolution
            direction *= -1

        robot_arm._actuators[0].set_theta(robot_arm._actuators[0].max_theta)
        await robot_arm._actuators[1].discrete_sweep(direction, resolution, sleep_time)
    finally:
        robot_arm.logger.info("Workspace demo has concluded.")
        await asyncio.sleep(0.5)
        robot_arm.set_joint_pose((0, 0))
        robot_arm.doing_demo = False


@socket_io.event
async def dynamics_coupling_demo(id):
    server_logger.info(f'Received DYNAMICS_COUPLING_DEMO command from id {id}')
    if robot_arm.doing_demo:
        robot_arm.logger.error(f'Cannot do DYNAMICS_DEMO while another demo is active.')
        return
    robot_arm.set_control_mode(ControlTypes.ACTUATOR_POSITION_CONTROL)
    robot_arm.doing_demo = True
    robot_arm.logger.info('Begin dynamics demo')
    robot_arm.set_joint_pose((0, 0))
    await asyncio.sleep(1)
    robot_arm._actuators[1].torque_disable()
    original_kp = robot_arm._actuators[0].servo.read_control_table('Position_P_Gain')
    robot_arm._actuators[0].set_motor_gains(kp=300)

    for _ in range(2):
        robot_arm._actuators[0].set_theta(math.pi/2)
        await asyncio.sleep(1)
    
        robot_arm._actuators[0].set_theta(-math.pi/2)
        await asyncio.sleep(1)

    robot_arm._actuators[0].set_theta(0)
    await asyncio.sleep(1)
    robot_arm.torque_enable()

    # clean up
    robot_arm._actuators[0].set_motor_gains(kp=original_kp)
    robot_arm.set_joint_pose((0, 0))
    robot_arm.doing_demo = False
    robot_arm.logger.info(f'End dynamics demo')


@socket_io.event
async def dynamics_time_varying_demo(id):
    server_logger.info(f'Received DYNAMICS_TIME_VARYING_DEMO command from id {id}')
    if robot_arm.doing_demo:
        robot_arm.logger.error(f'Cannot do DYNAMICS_DEMO while another demo is active.')
        return
    robot_arm.set_control_mode(ControlTypes.ACTUATOR_POSITION_CONTROL)
    robot_arm.doing_demo = True
    robot_arm.logger.info('Begin dynamics time varying demo')
    robot_arm.set_joint_pose((0, 0))
    await asyncio.sleep(1)
    robot_arm._actuators[1].torque_disable()
    original_kp = robot_arm._actuators[0].servo.read_control_table('Position_P_Gain')
    robot_arm._actuators[0].set_motor_gains(kp=300)

    for _ in range(2):
        robot_arm._actuators[0].set_theta(math.pi/2)
        await asyncio.sleep(1)
    
        robot_arm._actuators[0].set_theta(-math.pi/2)
        await asyncio.sleep(1)

    robot_arm._actuators[0].set_theta(0)
    await asyncio.sleep(1)
    robot_arm.torque_enable()

    # clean up
    robot_arm._actuators[0].set_motor_gains(kp=original_kp)
    robot_arm.set_joint_pose((0, 0))
    robot_arm.doing_demo = False
    robot_arm.logger.info(f'End dynamics demo')


@socket_io.event
async def set_joint_position(id, use_traj:bool, traj_time:str, t1:str, t2:str, x_guess:Optional[str]=None, y_guess:Optional[str]=None):
    server_logger.info(f'Received command MOVE_JOINTS. (t1, t2): ({t1}, {t2}) deg time: {traj_time} from {id}')
    q = [math.radians(float(theta)) for theta in (t1, t2)]
    if traj_time == "":
        robot_arm.set_joint_pose(q, use_traj)   # calculates a default trajectory time
    else:
        robot_arm.set_joint_pose(q, use_traj, float(traj_time))
    if x_guess is not None and y_guess is not None:
        solution = robot_arm._forward_kinematics(q)
        x_correct = abs(solution[0] - float(x_guess)) < 2
        y_correct = abs(solution[1] - float(y_guess)) < 2
        if x_correct and y_correct:
            server_logger.info(f'Forward kinematics solution guess was correct! Solution: {solution}')
        else:
            server_logger.info(f'Forward kinematics solution guess was incorrect! Correct solution {solution}')
    await send_telemtry()


@socket_io.event
async def set_cartesian_position(id, use_traj:bool, traj_time:str, x_str:str, y_str:str, theta1_guess:Optional[str]=None, theta2_guess:Optional[str]=None):
    server_logger.info(f'Received command MOVE_CARTESIAN. (x, y): ({x_str}, {y_str}) mm from {id}')
    x = float(x_str)
    y = float(y_str)
    if traj_time == "":
        robot_arm.set_cartesian_goal_position((x, y), use_traj)
    else:
        robot_arm.set_cartesian_goal_position((x, y), use_traj, float(traj_time))
    
    if theta1_guess is not None and theta2_guess is not None:
        solution = robot_arm._ikine_analytic((x, y))[0]
        theta1, theta2 = float(theta1_guess), float(theta2_guess)               # these are in degrees
        solution = [int(round(math.degrees(i))) for i in solution]              # convert to degrees
        theta1_correct = abs(solution[0] - theta1) < 2
        theta2_correct = abs(solution[1] - theta2) < 2
        if theta1_correct and theta2_correct:
            server_logger.info(f'Inverse kinematics solution guess was correct! Solution: {solution}')
        else:
            server_logger.info(f'Inverse kinematics solution guess was incorrect! Correct solution {solution}')

    await send_telemtry()


@socket_io.event
async def set_active_controller(id, controller_name:str):
    controller = robotics.ControlTypes(int(controller_name))
    server_logger.info(f'Received command SET_ACTIVE_CONTROLLER. Controller: {controller.name}')
    try:
        robot_arm.set_control_mode(controller)
    except:
        server_logger.error(traceback.print_exc())


@socket_io.event
async def set_controller_gains(id, pid_id, kp, ki, kd):
    """Change the gains of whatever controller is currently active."""
    server_logger.info(f'Received command SET_CONTROLLER_GAINS for motor: {pid_id} kp: {kp}, ki: {ki}, kd: {kd}')
    # try:
    gains = (kp, ki, kd)
    pid_id = int(pid_id)
    if pid_id not in (0, 1): raise ValueError
    if robot_arm.control_manager.active_controller_name == ControlTypes.ACTUATOR_POSITION_CONTROL:
        robot_arm.set_motor_gains(pid_id, float(kp), float(kd))
    else:
        robot_arm.control_manager.active_controller.set_gains_for_one_pid(pid_id, *gains)
        server_logger.debug('done')
    # except:
        
    #     server_logger.error(traceback.print_exc())


@socket_io.event
async def torque_enable(id):
    server_logger.info(f'Received command TORQUE_ENABLE.')
    robot_arm.torque_enable()


@socket_io.event
async def torque_disable(id):
    server_logger.info(f'Received command TORQUE_DISABLE.')
    robot_arm.torque_disable()


@socket_io.event
async def reconnect_dynamixel(id):
    server_logger.info(f'Received command RECONNECT.')
    robot_arm.reconnect()


# VIDEO STREAMING
# https://github.com/miguelgrinberg/Flask-SocketIO/issues/778
# https://www.pyimagesearch.com/2019/09/02/opencv-stream-video-to-web-browser-html-page/
@routes.get('/video_feed')
async def video_feed(request):
    response = aiohttp.web.StreamResponse()
    response.content_type = 'multipart/x-mixed-replace; boundary=frame'
    await response.prepare(request)
    global cam
    # get the encoded video feed as bytes and write to cients
    try:
        async for encoded_frame in cam.encoded_video_generator():
            await response.write(encoded_frame)
    except ConnectionResetError:    # the client has disconnected
        pass
    return response


########################
# Telemetry
########################
async def send_telemtry():
    t1, t2 = [round(math.degrees(theta)) for theta in robot_arm.Q]
    x, y = robot_arm.X
    # controller = robot_arm.control_manager.
    controller = robot_arm.control_manager.active_controller_name.name
    if robot_arm.control_manager.active_controller_name == ControlTypes.ACTUATOR_POSITION_CONTROL:
        gains_0, gains_1 = [list(a.gains.values()) for a in robot_arm._actuators]
    else:
        gains_0, gains_1 = robot_arm.control_manager.active_controller.get_gains()
    motor_0_power, motor_1_power = [str(motor.torque_is_enabled).upper() for motor in robot_arm._actuators]
    await socket_io.emit('telemetry', (motor_0_power, motor_1_power, controller, gains_0, gains_1, t1, t2, x, y))


async def telemetry_service():
    while 1: 
        await send_telemtry()
        await asyncio.sleep(0.25)


telemetry_task:Optional[asyncio.Task] = None


def start_telemetry():
    global loop
    global telemetry_task
    if telemetry_task is not None:
        telemetry_task.cancel()
    telemetry_task = loop.create_task(telemetry_service())


if __name__ == '__main__':
    #bind the aiohttp endpoint to the web_application
    import argparse
    parser = argparse.ArgumentParser()
    # Add an argument
    parser.add_argument('--log_level', type=str, required=False)
    # Parse the argument
    args = parser.parse_args()
    try:
        log_level_arg:str = args.log_level.upper()
        if log_level_arg == 'DEBUG':
            log_level = logging.DEBUG
        elif log_level_arg == 'INFO':
            log_level = logging.INFO
        elif log_level_arg == 'WARNING':
            log_level = logging.WARNING
        elif log_level_arg == 'ERROR':
            log_level = logging.ERROR
    except:
        log_level = logging.INFO

    # set up the robot
    server_logger.level = log_level
    robot_arm = robotics.RRTwoLink(socket_logging_handler, log_level)
    robot_arm.start_service(loop)
    start_telemetry()
    try:


        web_app.add_routes(routes)

        # web_app.add_routes([aiohttp.web.get('/', landing)])
        # web_app.router.add_get('/', landing)
        # web_app.router.add_get('/', master)
        # web_app.router.add_get('/', assignment_1)
        # web_app.router.add_get('/video_feed', video_feed)
        cam = camera.CameraWrapper(framerate=32)
        cam.start_video()
        aiohttp.web.run_app(web_app)
    finally:
        try:
            cam.stop_video()
            cam.clean_up()
        except: pass

        try:
            loop = asyncio.get_running_loop()
        except:
            loop = asyncio.get_event_loop()
        server_logger.info(f'Shutting down.')
        loop.run_until_complete(robot_arm.shutdown())
        time.sleep(0.5)
        loop.run_until_complete(shut_down(web_app))
        time.sleep(0.5)
        loop.close()
