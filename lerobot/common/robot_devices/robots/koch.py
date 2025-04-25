import pickle
import time
from dataclasses import dataclass, field, replace
from pathlib import Path
import os
import ctypes

import numpy as np
import torch

from lerobot.common.robot_devices.cameras.utils import Camera
from lerobot.common.robot_devices.motors.dynamixel import (
    DriveMode,
    DynamixelMotorsBus,
    OperatingMode,
    TorqueMode,
)
from lerobot.common.robot_devices.motors.utils import MotorsBus
from lerobot.common.robot_devices.utils import RobotDeviceAlreadyConnectedError, RobotDeviceNotConnectedError
from concurrent.futures import ThreadPoolExecutor
from Robotic_Arm.rm_robot_interface import *


URL_HORIZONTAL_POSITION = {
    "follower": "https://raw.githubusercontent.com/huggingface/lerobot/main/media/koch/follower_horizontal.png",
    "leader": "https://raw.githubusercontent.com/huggingface/lerobot/main/media/koch/leader_horizontal.png",
}
URL_90_DEGREE_POSITION = {
    "follower": "https://raw.githubusercontent.com/huggingface/lerobot/main/media/koch/follower_90_degree.png",
    "leader": "https://raw.githubusercontent.com/huggingface/lerobot/main/media/koch/leader_90_degree.png",
}

########################################################################
# Calibration logic
########################################################################


TARGET_HORIZONTAL_POSITION = np.array([0, 0, 0, 0, 0, 0,0,0])
TARGET_90_DEGREE_POSITION = np.array([0, 0, 0, 0, 0, 0,0,0])
GRIPPER_OPEN = np.array([-400])

FLAG_FOLLOW=0 #1为高跟随,0为低跟随

def apply_homing_offset(values: np.array, homing_offset: np.array) -> np.array:
    for i in range(len(values)):
        if values[i] is not None:
            values[i] += homing_offset[i]
    return values


def apply_drive_mode(values: np.array, drive_mode: np.array) -> np.array:
    for i in range(len(values)):
        if values[i] is not None and drive_mode[i]:
            values[i] = -values[i]
    return values


def apply_calibration(values: np.array, homing_offset: np.array, drive_mode: np.array) -> np.array:
    values = apply_drive_mode(values, drive_mode)
    values = apply_homing_offset(values, homing_offset)
    return values


def revert_calibration(values: np.array, homing_offset: np.array, drive_mode: np.array) -> np.array:
    """
    Transform working position into real position for the robot.
    """
    values = apply_homing_offset(
        values,
        np.array([-homing_offset if homing_offset is not None else None for homing_offset in homing_offset]),
    )
    values = apply_drive_mode(values, drive_mode)
    return values


def revert_appropriate_positions(positions: np.array, drive_mode: list[bool]) -> np.array:
    for i, revert in enumerate(drive_mode):
        if not revert and positions[i] is not None:
            positions[i] = -positions[i]
    return positions


def compute_corrections(positions: np.array, drive_mode: list[bool], target_position: np.array) -> np.array:
    correction = revert_appropriate_positions(positions, drive_mode)

    for i in range(len(positions)):
        if correction[i] is not None:
            if drive_mode[i]:
                correction[i] -= target_position[i]
            else:
                correction[i] += target_position[i]

    return correction


def compute_nearest_rounded_positions(positions: np.array) -> np.array:
    return np.array(
        [
            round(positions[i] / 1024) * 1024 if positions[i] is not None else None
            for i in range(len(positions))
        ]
    )


#用于计算机械臂的归零偏差
def compute_homing_offset(
    arm: DynamixelMotorsBus, drive_mode: list[bool], target_position: np.array
) -> np.array:
    # Get the present positions of the servos
    #从 arm 读取当前伺服电机的位置（“Present_Position”）。apply_calibration 函数使用 drive_mode 
    # 和一个初始零位置数组对这些位置进行校准。
    present_positions = apply_calibration(
        arm.read("Present_Position"), np.array([0, 0, 0, 0, 0, 0,0,0]), drive_mode
    )

    nearest_positions = compute_nearest_rounded_positions(present_positions)
    correction = compute_corrections(nearest_positions, drive_mode, target_position)
    return correction


def compute_drive_mode(arm: DynamixelMotorsBus, offset: np.array):
    
    # Get current positions
    present_positions = apply_calibration(
        arm.read("Present_Position"), offset, np.array([False, False, False, False, False, False,False,False])
    )

    nearest_positions = compute_nearest_rounded_positions(present_positions)

    # construct 'drive_mode' list comparing nearest_positions and TARGET_90_DEGREE_POSITION
    drive_mode = []
    for i in range(len(nearest_positions)):
        drive_mode.append(nearest_positions[i] != TARGET_90_DEGREE_POSITION[i])
    return drive_mode

#重置电机的状态，禁用扭矩模式，设置伺服电机的模式
def reset_arm(arm: MotorsBus):
    # To be configured, all servos must be in "torque disable" mode
    arm.write("Torque_Enable", TorqueMode.DISABLED.value)#将所有伺服电机的扭矩模式设置为禁用，以确保电机在重置过程中不会施加力量。

    # Use 'extended position mode' for all motors except gripper, because in joint mode the servos can't
    # rotate more than 360 degrees (from 0 to 4095) And some mistake can happen while assembling the arm,
    # you could end up with a servo with a position 0 or 4095 at a crucial point See [
    # https://emanual.robotis.com/docs/en/dxl/x/x_series/#operating-mode11]
    all_motors_except_gripper = [name for name in arm.motor_names if name != "gripper"]
    arm.write("Operating_Mode", OperatingMode.EXTENDED_POSITION.value, all_motors_except_gripper)

    # TODO(rcadene): why?
    # Use 'position control current based' for gripper
    arm.write("Operating_Mode", OperatingMode.CURRENT_CONTROLLED_POSITION.value, "gripper")

    # Make sure the native calibration (homing offset abd drive mode) is disabled, since we use our own calibration layer to be more generic
    arm.write("Homing_Offset", 0)
    arm.write("Drive_Mode", DriveMode.NON_INVERTED.value)


def run_arm_calibration(arm: MotorsBus, name: str, arm_type: str):
    """Example of usage:
    ```python
    run_arm_calibration(arm, "left", "follower")
    ```
    """
    #先重置机械臂
    reset_arm(arm)

    #提示用户放置到水平位置
    # TODO(rcadene): document what position 1 mean
    print(
        f"Please move the '{name} {arm_type}' arm to the horizontal position (gripper fully closed, see {URL_HORIZONTAL_POSITION[arm_type]})"
    )
    input("Press Enter to continue...")
    #调用 compute_homing_offset 函数来计算在水平位置时的归零偏差，TARGET_HORIZONTAL_POSITION 是预设的水平目标位置。
    #TARGET_HORIZONTAL_POSITION 是预设的水平目标位置。
    horizontal_homing_offset = compute_homing_offset(
        arm, [False, False, False, False, False, False,False,False], TARGET_HORIZONTAL_POSITION
    )

    # TODO(rcadene): document what position 2 mean
    #显示一条消息，提示用户将机械臂移动到90度位置，并要求用户在调整后按回车继续。
    print(
        f"Please move the '{name} {arm_type}' arm to the 90 degree position (gripper fully open, see {URL_90_DEGREE_POSITION[arm_type]})"
    )
    input("Press Enter to continue...")
    
    #计算驱动模式和归零偏差
    drive_mode = compute_drive_mode(arm, horizontal_homing_offset)
    homing_offset = compute_homing_offset(arm, drive_mode, TARGET_90_DEGREE_POSITION)

    # Invert offset for all drive_mode servos
    for i in range(len(drive_mode)):
        if drive_mode[i]:
            homing_offset[i] = -homing_offset[i]

    print("Calibration is done!")

    print("=====================================")
    print("      HOMING_OFFSET: ", " ".join([str(i) for i in homing_offset]))
    print("      DRIVE_MODE: ", " ".join([str(i) for i in drive_mode]))
    print("=====================================")

    return homing_offset, drive_mode


########################################################################
# Alexander Koch robot arm
########################################################################

@dataclass
class KochRobotConfig:
    """
    Example of usage:
    ```python
    KochRobotConfig()
    ```
    """

    # Define all components of the robot
    leader_arms: dict[str, MotorsBus] = field(default_factory=lambda: {})
    #follower_arms: dict[str, MotorsBus] = field(default_factory=lambda: {})
    cameras: dict[str, Camera] = field(default_factory=lambda: {})


class KochRobot:
    # TODO(rcadene): Implement force feedback
    """Tau Robotics: https://tau-robotics.com

    Example of highest frequency teleoperation without camera:
    ```python
    # Defines how to communicate with the motors of the leader and follower arms
    leader_arms = {
        "main": DynamixelMotorsBus(
            port="/dev/tty.usbmodem575E0031751",
            motors={
                # name: (index, model)
                "shoulder_pan": (1, "xl330-m077"),
                "shoulder_lift": (2, "xl330-m077"),
                "elbow_flex": (3, "xl330-m077"),
                "wrist_flex": (4, "xl330-m077"),
                "wrist_roll": (5, "xl330-m077"),
                "gripper": (6, "xl330-m077"),
            },
        ),
    }
    follower_arms = {
        "main": DynamixelMotorsBus(
            port="/dev/tty.usbmodem575E0032081",
            motors={
                # name: (index, model)
                "shoulder_pan": (1, "xl430-w250"),
                "shoulder_lift": (2, "xl430-w250"),
                "elbow_flex": (3, "xl330-m288"),
                "wrist_flex": (4, "xl330-m288"),
                "wrist_roll": (5, "xl330-m288"),
                "gripper": (6, "xl330-m288"),
            },
        ),
    }
    robot = KochRobot(leader_arms, follower_arms)

    # Connect motors buses and cameras if any (Required)
    robot.connect()

    while True:
        robot.teleop_step()
    ```

    Example of highest frequency data collection without camera:
    ```python
    # Assumes leader and follower arms have been instantiated already (see first example)
    robot = KochRobot(leader_arms, follower_arms)
    robot.connect()
    while True:
        observation, action = robot.teleop_step(record_data=True)
    ```

    Example of highest frequency data collection with cameras:
    ```python
    # Defines how to communicate with 2 cameras connected to the computer.
    # Here, the webcam of the mackbookpro and the iphone (connected in USB to the macbookpro)
    # can be reached respectively using the camera indices 0 and 1. These indices can be
    # arbitrary. See the documentation of `OpenCVCamera` to find your own camera indices.
    cameras = {
        "macbookpro": OpenCVCamera(camera_index=0, fps=30, width=640, height=480),
        "iphone": OpenCVCamera(camera_index=1, fps=30, width=640, height=480),
    }

    # Assumes leader and follower arms have been instantiated already (see first example)
    robot = KochRobot(leader_arms, follower_arms, cameras)
    robot.connect()
    while True:
        observation, action = robot.teleop_step(record_data=True)
    ```

    Example of controlling the robot with a policy (without running multiple policies in parallel to ensure highest frequency):
    ```python
    # Assumes leader and follower arms + cameras have been instantiated already (see previous example)
    robot = KochRobot(leader_arms, follower_arms, cameras)
    robot.connect()
    while True:
        # Uses the follower arms and cameras to capture an observation
        observation = robot.capture_observation()

        # Assumes a policy has been instantiated
        with torch.inference_mode():
            action = policy.select_action(observation)

        # Orders the robot to move
        robot.send_action(action)
    ```

    Example of disconnecting which is not mandatory since we disconnect when the object is deleted:
    ```python
    robot.disconnect()
    ```
    """

    def __init__(
        self,
        config: KochRobotConfig | None = None,
        calibration_path: Path = ".cache/calibration/koch.pkl",
        **kwargs,
    ):
        if config is None:
            config = KochRobotConfig()
        # Overwrite config arguments using kwargs
        self.config = replace(config, **kwargs)
        self.calibration_path = Path(calibration_path)
        self.leader_arms = self.config.leader_arms
        self.cameras = self.config.cameras
        self.is_connected = False
        self.logs = {}
        #------------------connect gen72 API------------------
        self.robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        self.arm=self.robot.rm_create_robot_arm("192.168.1.18", 8080)
        #------------------init gen72 API----------------------
        self.arm.rm_set_arm_run_mode(2)
        self.arm.rm_set_tool_voltage(3) 
        self.arm.rm_set_modbus_mode(1,115200,2)   
        self.write_params = rm_peripheral_read_write_params_t(1, 40000, 1)
        self.arm.rm_write_single_register(self.write_params,100) 
        self.arm.rm_change_tool_frame("lebai2")
        #-------------------
        self.gipflag=1
        self.gipflag_send=1
        #远程操控部分-读取领导臂的目标位置
        self.leader_pos = {}
        #远程操控部分-写入gen72 API的关节数据
        float_joint = ctypes.c_float*7
        self.joint_teleop_write = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        #远程操控部分-读取gen72 API的关节数据
        self.joint_teleop_read = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        #数据观察部分-读取gen72 API的关节数据
        self.joint_obs_read=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        #数据观察部分-上传关节以及夹爪开合度
        self.joint_obs_present=[0.0]*8
        #推理输出部分-接收模型推理的关节角度，写入gen72 API中
        self.joint_send=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        #----------------------move initial position----------------
        joint_base = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.arm.right_arm.rm_movej(joint_base, 20, 0, 0, 1)




    def connect(self):
        if self.is_connected:
            raise RobotDeviceAlreadyConnectedError(
                "KochRobot is already connected. Do not run `robot.connect()` twice."
            )
        if not self.leader_arms and not self.cameras:
            raise ValueError(
                "KochRobot doesn't have any device to connect. See example of usage in docstring of the class."
            )
        # Connect the arms
        #for name in self.follower_arms:
        for name in self.leader_arms:
            #self.follower_arms[name].connect()
            self.leader_arms[name].connect()

        # Reset the arms and load or run calibration
        #初始化电机的位置，先读取是否存在.cache文件，如果没有就单独创建
        if self.calibration_path.exists():
            # Reset all arms before setting calibration
            # for name in self.follower_arms:
            #     reset_arm(self.follower_arms[name])
            for name in self.leader_arms:
                reset_arm(self.leader_arms[name])

            with open(self.calibration_path, "rb") as f:
                calibration = pickle.load(f)
        else:
            # Run calibration process which begins by reseting all arms
            #创建初始化的路径
            calibration = self.run_calibration()

            self.calibration_path.parent.mkdir(parents=True, exist_ok=True)
            with open(self.calibration_path, "wb") as f:
                pickle.dump(calibration, f)

        # Set calibration
        # for name in self.follower_arms:
        #     self.follower_arms[name].set_calibration(calibration[f"follower_{name}"])
        for name in self.leader_arms:
            self.leader_arms[name].set_calibration(calibration[f"leader_{name}"])

        # Set better PID values to close the gap between recored states and actions
        # TODO(rcadene): Implement an automatic procedure to set optimial PID values for each motor
        for name in self.leader_arms:
            self.leader_arms[name].write("Torque_Enable", 1, "gripper")
            self.leader_arms[name].write("Goal_Position", GRIPPER_OPEN, "gripper")

        # Connect the cameras
        for name in self.cameras:
            self.cameras[name].connect()

        self.is_connected = True

    def run_calibration(self):
        calibration = {}

        for name in self.leader_arms:
            homing_offset, drive_mode = run_arm_calibration(self.leader_arms[name], name, "leader")

            calibration[f"leader_{name}"] = {}
            for idx, motor_name in enumerate(self.leader_arms[name].motor_names):
                calibration[f"leader_{name}"][motor_name] = (homing_offset[idx], drive_mode[idx])

        return calibration


    def teleop_step(
        self, record_data=False
    ) -> None | tuple[dict[str, torch.Tensor], dict[str, torch.Tensor]]:
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                "KochRobot is not connected. You need to run `robot.connect()`."
            )
        # Prepare to assign the positions of the leader to the follower
        for name in self.leader_arms:
            now = time.perf_counter()
            #读取当领导臂电机数据
            self.leader_pos[name] = self.leader_arms[name].read("Present_Position")
            #电机数据到关节角度的转换
            for i in range(7):
                processed_value = round(((self.leader_pos[name][i]) / 1024) * 90, 2)
                self.joint_teleop_write[i] = processed_value
            # for i in range(8):
            #   print(self.leader_pos[name][i])
            # self.joint_teleop_write[1] = 0-self.joint_teleop_write[1]
            self.joint_teleop_write[3] = 0-self.joint_teleop_write[3]
            self.joint_teleop_write[5] = 0-self.joint_teleop_write[5]
            self.logs[f"read_leader_{name}_pos_dt_s"] = time.perf_counter() - now
        # for i in range(7):
        #     print(self.joint_teleop_write[i])
        
        for name in self.leader_arms:
            now = time.perf_counter()
            #电机角度到夹爪开合度的换算
            giper_trans = (((self.leader_pos[name][7]) / 1024) * 90 + 98)/ 62 * 100
            #调用透传API，控制gen72移动到目标位置
            giper_trans=round(giper_trans, 2)
            self.arm.rm_movej_canfd(self.joint_teleop_write, True, 0, 2, 100)
            #状态为张开，且需要关闭夹爪
            if (giper_trans < 21) and (self.gipflag == 1):
                self.arm.rm_write_single_register(self.write_params,10)
                self.gipflag=0
            #状态为闭合，且需要张开夹爪
            if (giper_trans > 79) and (self.gipflag == 0):
                self.arm.rm_write_single_register(self.write_params,100)
                self.gipflag=1
            self.logs[f"write_follower_{name}_goal_pos_dt_s"] = time.perf_counter() - now

        if not record_data:
            return

        for name in self.leader_arms:
            #调用API获取gen72当前关节角度
            now = time.perf_counter()
            self.pDll.Get_Joint_Degree(self.nSocket,self.joint_teleop_read) 
            # for i in range(7):
            #     self.joint_teleop_read[i] = 0
            #获取夹爪开合度
            eight_byte_array = np.zeros(8, dtype=np.float32)
            eight_byte_array[:7] = self.joint_teleop_read[:]
            giper_read=ctypes.c_int()
            if self.gipflag == 0:
                eight_byte_array[7] = 10
            elif self.gipflag == 1:
                eight_byte_array[7] = 100
            eight_byte_array = np.round(eight_byte_array, 2)
            self.logs[f"read_follower_{name}_pos_dt_s"] = time.perf_counter() - now

        #记录当前关节角度
        state = []
        for name in self.leader_arms:
            state.append(eight_byte_array)
        state = np.concatenate(state)#将当前位置合并为一个NumPy数组
        # print('当前夹爪状态',eight_byte_array[7])

        #将关节目标位置添加到 action 列表中
        eight_byte_array[:7] = self.joint_teleop_write[:]
        action = []
        for name in self.leader_arms:
            action.append(eight_byte_array)
        action = np.concatenate(action)
        # print('目标夹爪状态',eight_byte_array[7])

        # Capture images from cameras
        images = {}
        for name in self.cameras:
            now = time.perf_counter()
            images[name] = self.cameras[name].async_read()
            self.logs[f"read_camera_{name}_dt_s"] = self.cameras[name].logs["delta_timestamp_s"]
            self.logs[f"async_read_camera_{name}_dt_s"] = time.perf_counter() - now

        # Populate output dictionnaries and format to pytorch
        obs_dict, action_dict = {}, {}
        obs_dict["observation.state"] = torch.from_numpy(state)
        action_dict["action"] = torch.from_numpy(action)
        for name in self.cameras:
            obs_dict[f"observation.images.{name}"] = torch.from_numpy(images[name])
        return obs_dict, action_dict



    def capture_observation(self):
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                "KochRobot is not connected. You need to run `robot.connect()`."
            )

        #调用从臂api获取当前关节角度 
        for name in self.leader_arms:
            now = time.perf_counter()
            self.pDll.Get_Joint_Degree(self.nSocket,self.joint_obs_read)  
 
            self.joint_obs_present[:7]=self.joint_obs_read[:]
            if self.gipflag_send==1:
                self.joint_obs_present[7]=100
            elif self.gipflag_send==0:
                self.joint_obs_present[7]=10
            self.logs[f"read_follower_{name}_pos_dt_s"] = time.perf_counter() - now

        # Create state by concatenating follower current position
        state = []
        self.joint_obs_present = np.round(self.joint_obs_present, 2)
        joint_array_np = np.array( self.joint_obs_present)
        state = np.array([joint_array_np], dtype=np.float32)
        state = np.concatenate(state, dtype=np.float32)

        # Capture images from cameras
        images = {}
        for name in self.cameras:
            now = time.perf_counter()
            images[name] = self.cameras[name].async_read()
            self.logs[f"read_camera_{name}_dt_s"] = self.cameras[name].logs["delta_timestamp_s"]
            self.logs[f"async_read_camera_{name}_dt_s"] = time.perf_counter() - now

        # Populate output dictionnaries and format to pytorch
        obs_dict = {}
        obs_dict["observation.state"] = torch.from_numpy(state)
        for name in self.cameras:
            # Convert to pytorch format: channel first and float32 in [0,1]
            img = torch.from_numpy(images[name])
            img = img.type(torch.float32) / 255
            img = img.permute(2, 0, 1).contiguous()
            obs_dict[f"observation.images.{name}"] = img
        return obs_dict


    def send_action(self, action: torch.Tensor):
        """The provided action is expected to be a vector."""
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                "KochRobot is not connected. You need to run `robot.connect()`."
            )
        from_idx = 0
        to_idx = 8
        follower_goal_pos_array= action[from_idx:to_idx].numpy()

        follower_goal_pos_array = np.round(follower_goal_pos_array, 2)
        for i in range(7):
            self.joint_send[i]=follower_goal_pos_array[i]   
        self.pDll.Movej_CANFD(self.nSocket,self.joint_send,FLAG_FOLLOW,0)
        if (follower_goal_pos_array[7] <21 ) and (self.gipflag_send == 1):
            ret_giper = self.pDll.Write_Single_Register(self.nSocket, 1, 40000, 10 , 1, 1)
            self.gipflag_send=0
        #状态为闭合，且需要张开夹爪
        if (follower_goal_pos_array[7]>79) and (self.gipflag_send == 0):
            ret_giper = self.pDll.Write_Single_Register(self.nSocket, 1, 40000, 100, 1, 1)
            self.gipflag_send=1

    def disconnect(self):
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                "KochRobot is not connected. You need to run `robot.connect()` before disconnecting."
            )
        for name in self.leader_arms:
            self.leader_arms[name].disconnect()
        for name in self.cameras:
            self.cameras[name].disconnect()
        
        self.arm.rm_delete_robot_arm()
        self.is_connected = False

    def __del__(self):
        if getattr(self, "is_connected", False):
            self.disconnect()