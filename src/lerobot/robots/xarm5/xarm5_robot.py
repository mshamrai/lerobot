from lerobot.cameras import make_cameras_from_configs
from lerobot.robots import Robot
from arm.xarm5_robot_config import XArm5RobotConfig
from arm.xarm5_bus import XArmWrapper
from typing import Any
import numpy as np

class XArm5Robot(Robot):
    config_class = XArm5RobotConfig
    name = "xarm5_robot"

    def __init__(self, config: XArm5RobotConfig):
        super().__init__(config)
        self.config = config
        self.bus = XArmWrapper(
            port=self.config.port,
            motors={
                "joint1": (1, "ufactory-xarm5"),
                "joint2": (2, "ufactory-xarm5"),
                "joint3": (3, "ufactory-xarm5"),
                "joint4": (4, "ufactory-xarm5"),
                "joint5": (5, "ufactory-xarm5"),
                "gripper": (6, "ufactory-xarm5"),
            }
        )
        self.cameras = make_cameras_from_configs(config.cameras)
    
    @property
    def _motors_ft(self) -> dict[str, type]:
        return {
            "joint1.angle": float,
            "joint2.angle": float,
            "joint3.angle": float,
            "joint4.angle": float,
            "joint5.angle": float,
            "gripper.pos": float,
        }

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.cameras[cam].height, self.cameras[cam].width, 3) for cam in self.cameras
        }

    @property
    def observation_features(self) -> dict:
        return {**self._motors_ft, **self._cameras_ft}
    
    @property
    def action_features(self) -> dict:
        return {
            "delta_x": float,
            "delta_y": float,
            "delta_z": float,
            "gripper": int
        }
    
    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise ConnectionError(f"{self} is not connected.")

        # Read arm position
        obs_dict = self.bus.get_position()
        # print(f"Current arm position: {obs_dict}")
        obs_dict = { feat:val for feat, val in zip(self.observation_features.keys(), obs_dict) }

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.async_read()

        return obs_dict
    
    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        goal_pos = np.array(list(action.values()))
        # print(f"Sending action to XArm5: {goal_pos}")
        self.bus.set_position(goal_pos)
        return action
    
    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass    
        
    def configure(self) -> None:
        if not self.is_connected:
            raise ConnectionError(f"{self} is not connected.")
        
        self.bus.enable()
        self.bus.robot_reset()

    def connect(self, calibrate: bool = True) -> None:
        self.bus.connect()
        if not self.is_calibrated and calibrate:
            self.calibrate()

        for cam in self.cameras.values():
            cam.connect()

        self.configure()

    def disconnect(self) -> None:
        self.bus.disconnect()
        for cam in self.cameras.values():
            cam.disconnect()
