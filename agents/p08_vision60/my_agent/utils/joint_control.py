"""
ジョイント制御モジュール

PD制御によるジョイント角度制御
"""
import pybullet as p
from typing import Dict, List
from .config import config
from .state import Vision60State
from .robot_model import RobotModel
from .logging_config import get_logger

logger = get_logger('joint_control')


class JointController:
    """ジョイント制御クラス"""
    
    def __init__(self, robot_model: RobotModel, state: Vision60State):
        """
        ジョイントコントローラーを初期化
        
        Args:
            robot_model: ロボットモデル
            state: 状態管理
        """
        self.robot_model = robot_model
        self.state = state
    
    def set_initial_pose(self):
        """初期姿勢を設定"""
        logger.info("🦵 立つ姿勢を設定中...")
        for leg_name, joint_indices in self.robot_model.leg_joints.items():
            angles = self.state.standing_angles.get(leg_name, [0.0, 0.0, 1.5])
            for i, joint_idx in enumerate(joint_indices):
                try:
                    p.resetJointState(
                        self.state.robot_id,
                        joint_idx,
                        targetValue=angles[i],
                        targetVelocity=0.0
                    )
                    p.setJointMotorControl2(
                        bodyIndex=self.state.robot_id,
                        jointIndex=joint_idx,
                        controlMode=p.VELOCITY_CONTROL,
                        force=0.0
                    )
                except:
                    logger.warning(f"  警告: ジョイント {joint_idx} の設定に失敗しました")
    
    def apply_joint_control(self, is_standing_up: bool = False):
        """
        ジョイント制御を適用
        
        Args:
            is_standing_up: 立ち上がり中かどうか
        """
        # 力とゲインの決定
        # 物理的に現実的な力に制限（15kgのロボットに対して適切な値）
        if is_standing_up:
            current_force = config.STANDING_UP_FORCE
            current_position_gain = config.POSITION_GAIN * config.STANDING_UP_POSITION_GAIN_MULTIPLIER
            current_velocity_gain = config.VELOCITY_GAIN * config.STANDING_UP_VELOCITY_GAIN_MULTIPLIER
        else:
            # 通常の力: 15kgのロボットに対して、各ジョイントで20-30N程度が適切
            # 200Nは大きすぎて、物理法則を無視した動きを強制してしまう
            current_force = 30.0  # 物理的に現実的な力に制限
            current_position_gain = config.POSITION_GAIN
            current_velocity_gain = config.VELOCITY_GAIN
        
        # 各ジョイントを制御
        for leg_name, joint_indices in self.robot_model.leg_joints.items():
            angles = self.state.standing_angles.get(leg_name, [0.0, 0.0, 0.5])
            for i, joint_idx in enumerate(joint_indices):
                try:
                    p.setJointMotorControl2(
                        bodyIndex=self.state.robot_id,
                        jointIndex=joint_idx,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=angles[i],
                        positionGain=current_position_gain,
                        velocityGain=current_velocity_gain,
                        force=current_force
                    )
                except:
                    pass
