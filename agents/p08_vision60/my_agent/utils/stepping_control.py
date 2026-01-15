"""
足踏み動作制御モジュール

対角線の脚を交互に上げ下げする動作の制御
"""
import math
import pybullet as p
from typing import Dict, List
from .config import config
from .state import Vision60State
from .robot_model import RobotModel
from .logging_config import get_logger
from .logger import Logger

logger = get_logger('stepping_control')


class SteppingController:
    """足踏み動作制御クラス"""
    
    def __init__(self, robot_model: RobotModel, state: Vision60State):
        """
        足踏みコントローラーを初期化
        
        Args:
            robot_model: ロボットモデル
            state: 状態管理
        """
        self.robot_model = robot_model
        self.state = state
        self.phase_names = {
            0: "左前足(FL)と右後ろ足(BR)を上げる",
            1: "左前足(FL)と右後ろ足(BR)を戻す",
            2: "右前足(FR)と左後ろ足(BL)を上げる",
            3: "右前足(FR)と左後ろ足(BL)を戻す"
        }
    
    def start_stepping(self, step: int):
        """
        足踏み動作を開始
        
        Args:
            step: 現在のステップ
        """
        if self.state.stabilization_detected and not self.state.stepping_started:
            self.state.stepping_started = True
            self.state.stepping_phase_start_step = step
            self.state.base_standing_angles_for_stepping = self.state.standing_angles.copy()
            # 足踏み開始時に色を変更（緑系）
            self.robot_model.change_robot_color(config.ROBOT_COLOR_STEPPING)
            logger.info(f"\n  🦶 足踏み動作を開始します（ステップ{step}）...")
    
    def update_stepping(self, step: int):
        """
        足踏み動作を更新
        
        Args:
            step: 現在のステップ
        """
        # 歩行が開始されたら足踏みを停止
        if not self.state.stepping_started or self.state.walking_started:
            return
        
        # 現在のフェーズの経過ステップ数を計算
        phase_elapsed = step - self.state.stepping_phase_start_step if self.state.stepping_phase_start_step is not None else 0
        
        # フェーズが終了したら次のフェーズに移行
        if phase_elapsed >= config.STEPPING_PHASE_DURATION:
            prev_phase = self.state.stepping_phase
            self.state.stepping_phase = (self.state.stepping_phase + 1) % 4
            self.state.stepping_phase_start_step = step
            phase_elapsed = 0
            
            # 各フェーズの開始時に色を変更（足踏みのタイミングと同期）
            # フェーズ0, 2（足を上げる）: 緑色
            # フェーズ1, 3（足を戻す）: オリジナルの色（グレー）
            if self.state.stepping_phase == 0 or self.state.stepping_phase == 2:
                self.robot_model.change_robot_color(config.ROBOT_COLOR_STEPPING)
            else:  # フェーズ1, 3
                self.robot_model.restore_original_colors()
            
            # フェーズ3から0に戻ったら1サイクル完了
            if prev_phase == 3 and self.state.stepping_phase == 0:
                self.state.stepping_cycles_completed += 1
                logger.info(f"  🦶 足踏み{self.state.stepping_cycles_completed}サイクル完了 (ステップ{step})")
            
            logger.info(f"  🦶 足踏みフェーズ変更 (ステップ{step}): {self.phase_names[self.state.stepping_phase]}")
        
        # 足踏み動作中の状態を定期的にログ出力（100ステップごと）
        if step % config.STEPPING_LOG_INTERVAL == 0:
            self._log_stepping_status(step, phase_elapsed)
        
        # 各フェーズでの角度調整（簡易版）
        if self.state.base_standing_angles_for_stepping is not None:
            # 基本姿勢角度をコピー
            stepping_angles = {}
            for leg_name in self.robot_model.leg_joints.keys():
                stepping_angles[leg_name] = list(self.state.base_standing_angles_for_stepping.get(leg_name, [0.0, 0.0, 1.7]))
            
            # フェーズに応じて脚を上げる（簡易実装）
            phase_half = config.STEPPING_PHASE_DURATION * 0.5
            if phase_elapsed < phase_half:
                lift_progress = phase_elapsed / phase_half
            else:
                lift_progress = 1.0 - (phase_elapsed - phase_half) / phase_half
            
            # フェーズに応じた角度調整（足を上げるために調整量を増加）
            # 実機の動作に合わせて、両脚とも股関節を後ろ側に回し、膝を閉じることで足先を垂直に上げる
            # ログ分析により、水平方向の移動を抑え、垂直方向の上昇を増やすため、膝の角度を増やす
            if self.state.stepping_phase == 0:  # FL+BRを上げる
                stepping_angles['front_left'][1] += 0.3 * lift_progress  # hip: 約17度（後ろ向きに）- 0.4から0.3に減らして水平移動を抑制
                stepping_angles['front_left'][2] -= 0.5 * lift_progress   # knee: 約29度（曲げる）- 0.3から0.5に増やして垂直上昇を増やす
                stepping_angles['back_right'][1] += 0.3 * lift_progress  # hip: 約17度（後ろ向きに）
                stepping_angles['back_right'][2] -= 0.5 * lift_progress  # knee: 約29度（曲げる）
            elif self.state.stepping_phase == 2:  # FR+BLを上げる
                stepping_angles['front_right'][1] += 0.3 * lift_progress  # hip: 約17度（後ろ向きに）
                stepping_angles['front_right'][2] -= 0.5 * lift_progress  # knee: 約29度（曲げる）
                stepping_angles['back_left'][1] += 0.3 * lift_progress   # hip: 約17度（後ろ向きに）
                stepping_angles['back_left'][2] -= 0.5 * lift_progress  # knee: 約29度（曲げる）
            
            self.state.standing_angles = stepping_angles
    
    def _log_stepping_status(self, step: int, phase_elapsed: int):
        """足踏み動作の状態をログ出力"""
        phase_half = config.STEPPING_PHASE_DURATION * 0.5
        if phase_elapsed < phase_half:
            action_status = f"上げる動作中 (進行度: {phase_elapsed/phase_half*100:.1f}%)"
        else:
            action_status = f"戻す動作中 (進行度: {(phase_elapsed-phase_half)/phase_half*100:.1f}%)"
        
        # 現在の姿勢と位置を取得
        base_pos = self.state.get_current_position()
        current_roll = self.state.get_current_roll_deg()
        current_pitch = self.state.get_current_pitch_deg()
        
        # 各脚の膝角度を取得
        knee_angles = {}
        for leg_name, joint_indices in self.robot_model.leg_joints.items():
            knee_joint = joint_indices[2]
            joint_state = p.getJointState(self.state.robot_id, knee_joint)
            knee_angles[leg_name] = math.degrees(joint_state[0])
        
        # 接地状態を取得
        contact_states = {}
        plane_id = 0  # 地面のID（通常は0）
        leg_index_map = {
            'front_left': 0,
            'front_right': 1,
            'back_left': 2,
            'back_right': 3
        }
        
        # 足先位置を取得
        toe_positions = {}
        for leg_name, leg_idx in leg_index_map.items():
            toe_link_idx = self.robot_model.get_toe_link_index(leg_idx)
            if toe_link_idx is not None and toe_link_idx >= 0:
                try:
                    toe_state = p.getLinkState(self.state.robot_id, toe_link_idx)
                    toe_positions[leg_name] = toe_state[0]  # ワールド座標での位置
                except:
                    toe_positions[leg_name] = None
            else:
                toe_positions[leg_name] = None
        
        for leg_name, leg_idx in leg_index_map.items():
            toe_link_idx = self.robot_model.get_toe_link_index(leg_idx)
            if toe_link_idx is not None and toe_link_idx >= 0:
                try:
                    contact_points = p.getContactPoints(self.state.robot_id, plane_id, linkIndexA=toe_link_idx)
                    contact_count = len(contact_points)
                    total_force = sum([cp[9] for cp in contact_points]) if contact_points else 0.0
                    contact_states[leg_name] = (contact_count, total_force)
                except:
                    contact_states[leg_name] = (0, 0.0)
            else:
                contact_states[leg_name] = (0, 0.0)
        
        Logger.log_stepping_status(
            step=step,
            phase=self.state.stepping_phase,
            phase_names=self.phase_names,
            action_status=action_status,
            current_roll=current_roll,
            current_pitch=current_pitch,
            base_pos=base_pos,
            knee_angles=knee_angles,
            logger=logger,
            contact_states=contact_states,
            toe_positions=toe_positions
        )
