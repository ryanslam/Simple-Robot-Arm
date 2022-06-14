from __future__ import annotations
# from dataclasses imor

@dataclass
class ControlTableItem:
    address:int
    size:int                                            # in bytes
    data_name:str
    access:str                                          # R -> read, W -> write
    initial_value:Optional[int] = field(default=None)
    range:Optional[tuple[int]] = field(default=None)
    unit:Optional[str] = field(default=None)
    scale:Optional[float] = field(default=None)         # some of the "unit" things are


class ControlTable:
    # https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/
    def __init__(self):
        ########################################################
        # EEPROM SECTION OF THE CONTROL TABLE
        ########################################################
        self.model_number = ControlTableItem(0, 2, 'Model Number', 'R', 1030)
        self.model_information = ControlTableItem(2, 4, 'Model Information', 'R')
        self.firmware_version = ControlTableItem(6, 1, 'Firmware Version', 'R')
        self.ID = ControlTableItem(7, 1, 'ID', 'RW', 1, (0, 252))
        self.baud_rate = ControlTableItem(8, 1, 'Baud Rate', 'RW', 1, (0, 7))
        self.return_delay_time = ControlTableItem(9, 1, 'Return Delay Time', 'RW', 250, (0, 254), 'usec', 2)
        self.drive_mode = ControlTableItem(10, 1, 'Drive Mode', 'RW', 0, (0, 5))
        self.operating_mode = ControlTableItem(11, 1, 'Operating Mode', 'RW', 3, (0, 16))
        self.secondary_id = ControlTableItem(12, 1, 'Secondary ID', 'RW', 255, (0, 252))
        self.protocol_type = ControlTableItem(13, 1, 'Protocol Type', 'RW', 2, (1, 2))
        self.homing_offset = ControlTableItem(20, 4, 'Homing Offset', 'RW', 0, (-1044479, 1044479), 'pulse', 1)
        self.moving_threshold = ControlTableItem(24, 4, 'Moving Threshold', 'RW', 10, (0, 1023), 'rev/min', .229)
        self.temperature_limit = ControlTableItem(31, 1, 'Temperature Limit', 'RW', 80, (0, 100), 'C', 1)
        self.max_voltage_limit = ControlTableItem(32, 2, 'Max Voltage Limit', 'RW', 160, (95, 160), 'V', .1)
        self.min_voltage_limit = ControlTableItem(34, 2, 'Min Voltage Limit', 'RW', 95, (95, 160), 'V', .1)
        self.pwm_limit = ControlTableItem(36, 2, 'PWM Limit', 'RW', 885, (0, 885), '%', .113)
        self.current_limit = ControlTableItem(38, 2, 'Current Limit', 'RW', 1193, (0, 1193), 'mA', 2.69)
        self.velocity_limit = ControlTableItem(44, 4, 'Velocity Limit', 'RW', 330, (0, 1023), 'rev/min', .229)
        self.max_position_limit = ControlTableItem(48, 4, 'Max Position Limit', 'RW', 4095, (0, 4095), 'pulse', 1)
        self.min_position_limit = ControlTableItem(52, 4, 'Min Position Limit', 'RW', 0, (0, 4095), 'pulse', 1)
        self.shutdown = ControlTableItem(63, 1, 'Shutdown', 'RW', 52)

        ########################################################
        # RAM SECTION OF THE CONTROL TABLE
        ########################################################
        self.torque_enable = ControlTableItem(64, 1, 'Torque Enable', 'RW', 0, (0, 1))
        self.LED = ControlTableItem(65, 1, 'LED', 'RW', 0, (0, 1))
        self.status_return_level = ControlTableItem(68, 2, 'Status Return Level', 'RW', 2, (0, 2))
        self.registered_instruction = ControlTableItem(69, 2, 'Registered Instruction', 'R', 0, (0, 1))
        self.hardware_error_status = ControlTableItem(70, 2, 'Hardware Error Status', 'R', 0)
        self.velocity_i_gain = ControlTableItem(76, 2, 'Velocity I Gain', 'RW', 1920, (0, 16383))
        self.velocity_p_gain = ControlTableItem(78, 2, 'Velocity P Gain', 'RW', 100, (0, 16383))
        self.position_d_gain = ControlTableItem(80, 2, 'Position D Gain', 'RW', 0, (0, 16383))
        self.position_i_gain = ControlTableItem(82, 2, 'Position I Gain', 'RW', 0, (0, 16383))
        self.position_p_gain = ControlTableItem(84, 2, 'Position P Gain', 'RW', 800, (0, 16383))
        self.feedforward_2nd_gain = ControlTableItem(88, 2, 'Feedforward 2nd Gain', 'RW', 0, (0, 16383))
        self.feedforward_1st_gain = ControlTableItem(90, 2, 'Feedforward 1st Gain', 'RW', 0, (0, 16383))
        self.bus_watchdog = ControlTableItem(98, 1, 'Bus Watchdog', 'RW', 0, (1, 127), 'msec', 20)
        self.goal_pwm = ControlTableItem(100, 2, 'Goal PWM', 'RW', None, None, '%', .113)                       # limits here are set explicitly at PWM Limit
        self.goal_current = ControlTableItem(102, 2, 'Goal Current', 'RW', None, None, 'mA', 2.69)              # Limits set at current_limit
        self.goal_velocity = ControlTableItem(104, 4, 'Goal Velocity', 'RW', None, None, 'rev/min', 0.229)      # limits set explitcitly
        self.profile_acceleration = ControlTableItem(108, 4, 'Profile Acceleration', 'RW', 0, (0, 32767), 'rev/min^2', 214.577)
        self.profile_velocity = ControlTableItem(112, 4, 'Profile Velocity', 'RW', 0, (0, 32767), 'rev/min', .229)
        self.goal_position = ControlTableItem(116, 4, 'Goal Position', 'RW', None, None, 'pulse', 1)            # limits set explicitly
        self.realtime_tick = ControlTableItem(120, 2, 'Realtime Tick', 'R', None, (0, 32767), 'msec', 1)
        self.moving = ControlTableItem(122, 1, 'Moving', 'R', 0, (0, 1))
        self.moving_status = ControlTableItem(123, 1, 'Moving Status', 'R', 0)
        self.present_pwm = ControlTableItem(124, 2, 'Present PWM', 'R', None, None)
        self.present_current = ControlTableItem(126, 2, 'Present Current', 'R', None, None, )
        self.present_velocity = ControlTableItem(128, 2, 'Present Velocity', 'R', None, None, )
        self.present_position = ControlTableItem(132, 2, 'Present Position', 'R', None, None, )
        self.velocity_trajectory = ControlTableItem(136, 2, 'Velocity Trajectory', 'R', None, None, )
        self.position_trajectory = ControlTableItem(140, 2, 'Position Trajectory', 'R', None, None, )
        self.present_input_voltage = ControlTableItem(144, 2, 'Present Input Voltage', 'R', None, None, )
        self.present_temperature = ControlTableItem(146, 2, 'Present Temperature', 'R', None, None, )
        # self.indirect_address_1 = ControlTableItem(, 2, '', 'RW', 0, (0, 160))

