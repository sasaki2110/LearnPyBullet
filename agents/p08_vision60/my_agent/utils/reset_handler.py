"""
リセット機能モジュール

姿勢チェックとリセット処理
"""
import pybullet as p
import math
from .config import config
from .state import Vision60State
from .robot_model import RobotModel
from .logging_config import get_logger

logger = get_logger('reset_handler')


class ResetHandler:
    """リセット処理クラス"""
    
    def __init__(self, robot_model: RobotModel, state: Vision60State):
        """
        リセットハンドラーを初期化
        
        Args:
            robot_model: ロボットモデル
            state: 状態管理
        """
        self.robot_model = robot_model
        self.state = state
    
    def check_and_reset(self, step: int, is_standing_up: bool = False) -> bool:
        """
        リセットが必要かチェックし、必要ならリセット
        
        Args:
            step: 現在のステップ
            is_standing_up: 立ち上がり中かどうか
            
        Returns:
            リセットが発生したかどうか
        """
        if step % config.RESET_CHECK_INTERVAL != 0:
            return False
        
        if self.state.target_height is None:
            return False
        
        # 現在の姿勢を取得
        current_pos, current_orn = self.state.get_current_pose()
        current_euler = self.state.get_current_euler()
        current_pitch = math.degrees(current_euler[1])
        current_roll = math.degrees(current_euler[0])
        
        # リセット判定
        reset_needed = False
        reset_reason = None
        
        # 位置移動チェック
        if self.state.initial_pos_after_setup:
            pos_diff = math.sqrt(
                (current_pos[0] - self.state.initial_pos_after_setup[0])**2 +
                (current_pos[1] - self.state.initial_pos_after_setup[1])**2
            )
            if pos_diff > config.RESET_POSITION_THRESHOLD:
                reset_needed = True
                reset_reason = "位置移動"
        
        # 高さ低下チェック
        height_ratio = current_pos[2] / self.state.target_height
        if height_ratio < (1.0 - config.RESET_HEIGHT_THRESHOLD_RATIO):
            reset_needed = True
            reset_reason = "高さ低下"
        
        # 姿勢傾きチェック
        orientation_threshold = (config.RESET_ORIENTATION_THRESHOLD_STANDING_UP 
                                if is_standing_up 
                                else config.RESET_ORIENTATION_THRESHOLD)
        if abs(current_pitch) > orientation_threshold or abs(current_roll) > orientation_threshold:
            reset_needed = True
            reset_reason = "姿勢傾き"
        
        # リセット実行
        if reset_needed:
            self._reset_pose(current_pos, current_orn)
            self.state.increment_reset_count(reset_reason)
            logger.info(f"  🔄 リセット発生 #{self.state.reset_count} (ステップ{step}): {reset_reason}")
            return True
        
        return False
    
    def _reset_pose(self, target_pos, target_orn):
        """姿勢をリセット"""
        # 初期位置（X, Y座標）に戻し、高さ（Z座標）は現在の高さを維持
        if self.state.initial_pos_after_setup:
            # X, Y座標を初期位置に戻し、Z座標（高さ）は現在の高さを維持
            reset_pos = (
                self.state.initial_pos_after_setup[0],  # X: 初期位置
                self.state.initial_pos_after_setup[1],  # Y: 初期位置
                target_pos[2]  # Z: 現在の高さを維持
            )
            # 位置と姿勢をリセット
            p.resetBasePositionAndOrientation(
                self.state.robot_id,
                reset_pos,
                target_orn
            )
            # 速度と角速度をゼロにリセット（物理法則を無視した動きを防ぐため）
            p.resetBaseVelocity(
                self.state.robot_id,
                linearVelocity=[0, 0, 0],
                angularVelocity=[0, 0, 0]
            )
            # すべてのジョイントの速度もリセット（角度は現在の値を維持）
            num_joints = p.getNumJoints(self.state.robot_id)
            for i in range(num_joints):
                joint_info = p.getJointInfo(self.state.robot_id, i)
                if joint_info[2] == p.JOINT_REVOLUTE or joint_info[2] == p.JOINT_PRISMATIC:
                    # 現在のジョイント角度を取得
                    joint_state = p.getJointState(self.state.robot_id, i)
                    current_angle = joint_state[0]
                    # 角度は維持し、速度だけをゼロにリセット
                    p.resetJointState(self.state.robot_id, i, targetValue=current_angle, targetVelocity=0)
