
from eureka.drivers.base import (
    RobotStatus,
    AbstractIndustrialRobot,
    DeviceCommandResult,
    ErrorCode,
)
from optics_handling_perception.comp_graph.transform import Transform
from optics_handling_perception.comp_graph.robot_pose_type import PoseType
import numpy as np
import common_utility.typing_compat as T

from trajectory_msgs.msg import JointTrajectory

class RobotController(AbstractIndustrialRobot):
    def __init__(
        self,
        timeout: float = 1,
        trajectory_blending_radius: int = 1,
        **kargs,
    ):
        super().__init__()
        self.timeout = timeout

    @property
    def _speed_level(self):
        if self.config is None:
            return 1.0
        return self.config.speed_level

    # CONNECTION HANDLING
    def _switch_motors(self, switch) -> bool:
        pass
    def _connect(self, switch: bool) -> bool:
        pass

    # MONITOR HANDLING
    def _create_monitor_handler(self) -> bool:
        """
        The monitor handler is written in C++ for performance reasons
        namely it requires continuous polling the status of the robot
        """
        pass

    def _clear_monitor_handler(self):
        """Stop the monitor thread and clear resources"""
        pass

    def _robot_state_changed(self, state: int):
        pass

    def get_joint_positions(self) -> np.ndarray:
        pass

    def get_pose(self) -> PoseType:
        pass

    def _check_joints(self, target: np.ndarray):
        pass

    # COMMAND HANDLING
    def _move(
        self,
        interpolation: int,
        target: T.Union[np.ndarray, PoseType],
        pass_start: int = 0,
        speed: float = 1,
    ) -> bool:
        pass

    def _move_offset(
        self,
        interpolation,
        ref_joints: np.ndarray,
        offset: Transform,
        pass_start: int = 0,
        speed: float = 1.0,
    ) -> bool:
        pass
    def shift(
        self, interpolation, offset: Transform, pass_start: int = 0, speed: float = 1
    ) -> bool:

        pass

    def approach(
        self,
        interpolation,
        ref_joints: np.ndarray,
        dist: float,
        pass_start: int = 0,
        speed: float = 1.0,
    ) -> bool:
        pass

    def j2p(self, joints: np.ndarray) -> PoseType:
        """Convert joint confiuration to pose

        Args:
            joints (np.ndarray): Joint configuration for which pose is required

        Returns:
            PoseType: Corresponding pose
        """
        pass

    def p2j(self, pose: PoseType) -> np.ndarray:
        """Convert pose to joint configuration

        Args:
            pose (PoseType): pose for which joints are required

        Returns:
            np.ndarray: Corresponding joints
        """
        pass

    def change_frame(
        self,
        tool_frame_idx: T.Optional[int] = None,
        work_frame_idx: T.Optional[int] = None,
    ) -> bool:
        pass

    def set_tool_def(self, idx, transform) -> bool:
        """Set the tool frame definition.

        Args:
          idx: Tool frame index.
          transform: The transform from the tool frame to the robot flange.

        Returns:
          Whether the call is successful.
        """
        pass

    def set_work_def(self, idx, transform):
        """Set the work frame definition.

        Args:
          idx: Work frame index.
          transform: The transform from the work frame to the robot base frame.

        Returns:
          Whether the call is successful.
        """
        pass

    def set_acceleration(self, accel, decel) -> bool:
        """Set acceleration as a perecentage 1-100% of it's internal value

        Args:
            accel : acceleration as a percentage of it's maximum value

        Returns:
          Whether the call is successful.
        """
        pass

    def get_cur_tool(self) -> T.Optional[int]:
        """Return the current tool frame index."""
        pass

    def get_cur_work(self) -> T.Optional[int]:
        """Return the current work frame index."""
        pass

    def _get_cur_frame(
        self, is_tool: bool = False, is_work: bool = False
    ) -> T.Optional[int]:
        """Get the current frame index"""
        pass

    def _set_frame_def(
        self,
        transform: Transform,
        tool_frame_idx: T.Optional[int] = None,
        work_frame_idx: T.Optional[int] = None,
    ) -> bool:
        """Set the selected tool frame transformation."""
        pass

    def _execute_trajectory(self, trajectory: JointTrajectory) -> DeviceCommandResult:
        """Execute the trajectory"""

        pass

    def stop_trajectory(self):
        """Stop the current movement."""
        pass

    def set_motion_supervision(
        self,
        switch: bool,
        tune_value: T.Optional[int] = None,
        no_backup: T.Optional[bool] = None,
    ) -> bool:
        """Set motion supervision for ABB Robot

        Args:
            switch (bool): Turn on/off motion supervision
            tune_value (T.Optional[int], optional): Tuning the motion supervision sensitivity level in percent (1 - 300%) of system parameter level. A higher level gives more robust sensitivity. Defaults to None, a value must be provided to enable supervision.
            no_backup (T.Optional[bool], optional): If this switch is used, the robot does not back off at a motion collision. Defaults to None, a value must be provided to enable supervision.

        Returns:
            bool: True if motion supervision was set successfully
        """
        pass

    def _read_bit(self, port_str: str) -> int:
        pass

    def _write_bit(self, port_str: str, value: bool) -> None:
        pass
