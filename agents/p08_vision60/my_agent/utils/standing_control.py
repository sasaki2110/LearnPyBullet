"""
立ち上がり制御モジュール

段階的な立ち上がり動作の制御
"""
import math
import pybullet as p
from typing import Dict, List, Optional, Tuple
from .config import config
from .state import Vision60State
from .robot_model import RobotModel
from .logging_config import get_logger
from .logger import Logger

logger = get_logger('standing_control')


class StandingController:
    """立ち上がり制御クラス"""
    
    def __init__(self, robot_model: RobotModel, state: Vision60State):
        """
        立ち上がりコントローラーを初期化
        
        Args:
            robot_model: ロボットモデル
            state: 状態管理
        """
        self.robot_model = robot_model
        self.state = state
    
    def check_stability(self, step: int) -> bool:
        """
        安定確認
        
        Args:
            step: 現在のステップ
            
        Returns:
            安定が確認されたかどうか
        """
        if not self.state.stability_confirmed and self.state.reset_count == 0:
            if self.state.stability_check_start is None:
                self.state.stability_check_start = step
            elif step - self.state.stability_check_start >= config.STABILITY_CHECK_STEPS:
                self.state.stability_confirmed = True
                self.state.standing_up_start_step = step
                self.state.initial_standing_angles = {
                    leg_name: list(angles) for leg_name, angles in self.state.standing_angles.items()
                }
                logger.info(f"  ✅ 安定確認完了（ステップ{step}）: リセット無しで{config.STABILITY_CHECK_STEPS}ステップ経過")
                
                # 立ち上がる角度を設定
                self.state.standing_up_angles = config.STANDING_UP_ANGLES.copy()
                # 立ち上がり開始時に色を変更（オレンジ系）
                self.robot_model.change_robot_color(config.ROBOT_COLOR_STANDING_UP)
                logger.info(f"  🦵 立ち上がります（膝を大きく開き、hipを調整 - {config.STANDING_UP_DURATION}ステップかけてゆっくりと）...")
                return True
        return False
    
    def update_standing_angles(self, step: int, plane_id: int):
        """
        立ち上がり角度を更新
        
        Args:
            step: 現在のステップ
            plane_id: 地面のID（接地判定用）
        """
        if (self.state.stability_confirmed and 
            self.state.standing_up_angles is not None and 
            self.state.initial_standing_angles is not None and 
            self.state.standing_up_start_step is not None):
            
            # 進行度を計算
            elapsed_steps = step - self.state.standing_up_start_step
            progress = min(1.0, elapsed_steps / config.STANDING_UP_DURATION)
            
            # 立ち上がり完了を検知
            if self.state.standing_up_completed_step is None and progress >= 1.0:
                self.state.standing_up_completed_step = step
                # 立ち上がり完了時に通常色に戻す（足踏み開始時に緑色に変わる）
                self.robot_model.change_robot_color(config.ROBOT_COLOR_NORMAL)
                logger.info(f"  ✅ 立ち上がり完了 (ステップ{step}): 目標角度への到達完了")
            
            # 立ち上がり中の進行度をログ出力（100ステップごと）
            if elapsed_steps % config.STANDING_UP_LOG_INTERVAL == 0 and elapsed_steps < config.STANDING_UP_DURATION:
                self._log_standing_up_progress(step, progress, plane_id)
            
            # 線形補間で角度を更新
            for leg_name in self.robot_model.leg_joints.keys():
                initial = self.state.initial_standing_angles[leg_name]
                target = self.state.standing_up_angles[leg_name]
                self.state.standing_angles[leg_name] = [
                    initial[0] + (target[0] - initial[0]) * progress,
                    initial[1] + (target[1] - initial[1]) * progress,
                    initial[2] + (target[2] - initial[2]) * progress
                ]
    
    def _log_standing_up_progress(self, step: int, progress: float, plane_id: int):
        """立ち上がり進行度をログ出力"""
        # 現在の姿勢を取得
        current_pos, current_orn = self.state.get_current_pose()
        current_euler = self.state.get_current_euler()
        current_roll = self.state.get_current_roll_deg()
        current_pitch = self.state.get_current_pitch_deg()
        
        # 現在のknee角度を取得
        current_knee_angles = {}
        for leg_name, joint_indices in self.robot_model.leg_joints.items():
            knee_joint = joint_indices[2]
            knee_state = p.getJointState(self.state.robot_id, knee_joint)
            current_knee_angles[leg_name] = math.degrees(knee_state[0])
        
        # 目標knee角度
        target_knee_deg = math.degrees(self.state.standing_up_angles['front_left'][2])
        
        # 足先位置を取得
        toe_positions = {}
        leg_index_map = self.robot_model.get_leg_index_map()
        for leg_name, leg_idx in leg_index_map.items():
            toe_link_idx = self.robot_model.get_toe_link_index(leg_idx)
            if toe_link_idx is not None and toe_link_idx >= 0:
                try:
                    toe_state = p.getLinkState(self.state.robot_id, toe_link_idx)
                    toe_positions[leg_name] = toe_state[0]
                except:
                    toe_positions[leg_name] = None
            else:
                toe_positions[leg_name] = None
        
        # 接地状態を取得
        contact_states = {}
        for leg_name, leg_idx in leg_index_map.items():
            toe_link_idx = self.robot_model.get_toe_link_index(leg_idx)
            if toe_link_idx is not None and toe_link_idx >= 0:
                try:
                    contact_points = p.getContactPoints(self.state.robot_id, plane_id, linkIndexA=toe_link_idx)
                    contact_count = len(contact_points)
                    total_force = contact_count * 10.0  # 簡易的な推定
                    contact_states[leg_name] = (contact_count, total_force)
                except:
                    contact_states[leg_name] = (0, 0.0)
            else:
                contact_states[leg_name] = (0, 0.0)
        
        # 目標角度と実際の角度の誤差を計算
        angle_errors = {}
        for leg_name, joint_indices in self.robot_model.leg_joints.items():
            target_angles = self.state.standing_angles.get(leg_name, [0.0, 0.0, 0.5])
            actual_angles = []
            for j, joint_idx in enumerate(joint_indices):
                try:
                    joint_state = p.getJointState(self.state.robot_id, joint_idx)
                    actual_angle = math.degrees(joint_state[0])
                    actual_angles.append(actual_angle)
                except:
                    actual_angles.append(0.0)
            
            # 各ジョイントの誤差を計算（度単位）
            errors = []
            for j in range(len(joint_indices)):
                target_deg = math.degrees(target_angles[j])
                actual_deg = actual_angles[j]
                error = actual_deg - target_deg
                errors.append(error)
            angle_errors[leg_name] = errors
        
        # 前回値との差分を計算
        prev_roll = self.state.prev_roll
        prev_pitch = self.state.prev_pitch
        prev_knee_angles = self.state.prev_knee_angles
        prev_base_pos = self.state.prev_base_pos
        
        # 足先の高さを取得
        toe_heights = {}
        if all(pos is not None for pos in toe_positions.values()):
            for leg_name in leg_index_map.keys():
                toe_heights[leg_name] = toe_positions[leg_name][2]
            avg_toe_height = sum(toe_heights.values()) / 4.0
        else:
            avg_toe_height = None
            toe_heights = None
        
        # ログ出力
        Logger.log_standing_up_progress(
            step=step,
            progress=progress,
            current_roll=current_roll,
            logger=logger,
            current_pitch=current_pitch,
            base_pos=current_pos,
            knee_angles=current_knee_angles,
            target_knee_deg=target_knee_deg,
            contact_states=contact_states,
            toe_positions=toe_positions,
            angle_errors=angle_errors,
            prev_roll=prev_roll,
            prev_pitch=prev_pitch,
            prev_knee_angles=prev_knee_angles,
            prev_base_pos=prev_base_pos,
            prev_toe_heights=self.state.prev_toe_heights
        )
        
        # 前回値を更新
        self.state.prev_roll = current_roll
        self.state.prev_pitch = current_pitch
        self.state.prev_knee_angles = current_knee_angles.copy()
        self.state.prev_toe_heights = toe_heights.copy() if toe_heights else None
        self.state.prev_base_pos = list(current_pos)
    
    def is_standing_up(self) -> bool:
        """立ち上がり中かどうか"""
        return (self.state.stability_confirmed and 
                self.state.standing_up_start_step is not None and
                self.state.standing_up_completed_step is None)
