import enum
import threading
import time

import numpy as np
from xarm.wrapper import XArmAPI

from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError


class TorqueMode(enum.Enum):
    ENABLED = 1
    DISABLED = 0


class XArmWrapper:
    """Wrapper for the xArm Python SDK"""

    def __init__(
        self,
        port: str,
        motors: dict[str, tuple[int, str]],
        mock=False,
    ):
        print("Initializing XArmWrapper")  # Debug print
        self.port = port
        self.motors = motors
        self.mock = mock

        self.calibration = None
        self.is_connected = False
        self.logs = {}

        self.api = None

        self.MAX_SPEED_LIMIT = None
        self.MAX_ACC_LIMIT = None

        self.MOTION_RANGE_MM = 5

        self.last_position = None

        # x 200 - 450
        # y -330 - 280
        # z 110 - 300
        self.cartezian_limits_down = np.array([200, -330, 110]) 
        self.cartezian_limits_up = np.array([450, 280, 300]) 

    @property
    def motor_names(self) -> list[str]:
        return list(self.motors.keys())

    @property
    def motor_models(self) -> list[str]:
        return [model for _, model in self.motors.values()]

    @property
    def motor_indices(self) -> list[int]:
        return [idx for idx, _ in self.motors.values()]

    def connect(self):
        print("Connecting to xArm")  # Debug print
        if self.is_connected:
            raise DeviceAlreadyConnectedError(
                f"DynamixelMotorsBus({self.port}) is already connected. Do not call `motors_bus.connect()` twice."
            )

        if self.mock:
            print("Mock mode, not connecting to real device")  # Debug print
            return
        else:
            self.api = XArmAPI(self.port)

        try:
            if not self.api.connected:
                raise OSError(f"Failed to connect to xArm API @ '{self.port}'.")
            print("Successfully connected to xArm")  # Debug print
        except Exception as e:
            print(f"Exception while connecting in XArmWrapper: {e}")
            raise

        # Allow to read and write
        self.is_connected = True

        self.initialize_limits()

    def write(self, data_name, values: int | float | np.ndarray, motor_names: str | list[str] | None = None):
        pass  # TODO (@vmayoral): implement if of interest

    def read(self, data_name, motor_names: str | list[str] | None = None):
        pass  # TODO (@vmayoral): implement if of interest

    def enable(self):
        self.api.motion_enable(enable=True)
        self.api.clean_error()
        self.api.set_mode(1)
        self.api.set_state(state=0)
        #
        self.api.set_gripper_mode(0)
        self.api.set_gripper_enable(True)
        self.api.set_gripper_speed(2000)  # default speed, as there's no way to fetch gripper speed from API

        # # Initialize the global speed and acceleration limits
        # self.initialize_limits()  # not acting as expected

        self.last_position = self.api.get_position()[1]

    def disconnect(self):
        print("Disconnecting from xArm")  # Debug print
        if not self.is_connected:
            raise DeviceNotConnectedError(
                f"FeetechMotorsBus({self.port}) is not connected. Try running `motors_bus.connect()` first."
            )

        # Turn off manual mode after recording
        self.api.set_mode(0)
        self.api.set_state(0)
        # Light down the digital output 2 (button), to signal manual mode
        # self.api.set_tgpio_digital(ionum=2, value=0)
        # Disconnect both arms
        self.api.disconnect()

        # Signal as disconnected
        self.is_connected = False

    def __del__(self):
        if getattr(self, "is_connected", False):
            self.disconnect()

    def initialize_limits(self):
        # heuristic: 1/3 of the max speed and acceleration limits
        #  for testing purposes
        self.MAX_SPEED_LIMIT = max(self.api.joint_speed_limit) / 3
        self.MAX_ACC_LIMIT = max(self.api.joint_acc_limit) / 3

    def get_position(self):
        # code, angles = self.api.get_servo_angle()
        code, cartezians = self.api.get_position()
        code_gripper, pos_gripper = self.api.get_gripper_position()
        pos = cartezians + [pos_gripper]  # discard rotations
        # pos = angles[:-2] + [pos_gripper]  # discard 6th and 7th dof, which is not present in xArm 5
        # pos = angles + [pos_gripper]
        return pos

    def set_position(self, position: np.ndarray):
        current_pos = np.array(self.last_position[:3])
        cartezian_delta = position[:-1]
        goal_pos = current_pos + cartezian_delta * self.MOTION_RANGE_MM
        # goal_pos = cartezian_delta * self.MOTION_RANGE_MM
        # print(goal_pos)
        # if np.any(goal_pos < self.cartezian_limits_down) or np.any(goal_pos > self.cartezian_limits_up):
        #     return
        gripper_pos = int(position[-1])

        # joints
        # self.api.set_position(x=goal_pos[0], 
        #                       y=goal_pos[1], 
        #                       z=goal_pos[2], 
        #                       roll=180, 
        #                       pitch=0, 
        #                       yaw=0, 
        #                       is_radian=False, 
        #                       wait=False, 
        #                       relative=False)
        mvpose = list(goal_pos) + [180, 0., 0.]
        # print(mvpose)
        # self.api.set_mode(1)
        # self.api.set_state(0)
        
        self.api.set_servo_cartesian(
                              mvpose=mvpose,
                              is_radian=False,
        )

        self.last_position = mvpose
        # self.last_position = goal_pos

        # self.api.set_mode(0)
        # self.api.set_state(0)

        # gripper
        if gripper_pos == 2:
            self.api.set_gripper_position(pos=600, wait=False)
        elif gripper_pos == 0:
            self.api.set_gripper_position(pos=0, wait=False)

    def robot_reset(self):
        """Reset the robot to a safe state"""
        self.api.set_mode(0)
        self.api.set_state(state=0)
        self.api.move_gohome(wait=False)
        self.enable()


