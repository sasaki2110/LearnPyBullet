"""
歩行動作制御モジュール

対角線の脚を交互に上げ下げしながら前進する動作の制御
"""
import math
import pybullet as p
from typing import Dict, List
from .config import config
from .state import Vision60State
from .robot_model import RobotModel
from .logging_config import get_logger
from .logger import Logger

logger = get_logger('walking_control')


class WalkingController:
    """歩行動作制御クラス"""
    
    def __init__(self, robot_model: RobotModel, state: Vision60State):
        """
        歩行コントローラーを初期化
        
        Args:
            robot_model: ロボットモデル
            state: 状態管理
        """
        self.robot_model = robot_model
        self.state = state
        self.phase_names = {
            0: "左前足(FL)と右後ろ足(BR)を上げる→前に出す→着地 + 右前足(FR)と左後ろ足(BL)を後ろに蹴る",
            1: "左前足(FL)と右後ろ足(BR)を戻す",
            2: "右前足(FR)と左後ろ足(BL)を上げる→前に出す→着地 + 左前足(FL)と右後ろ足(BR)を後ろに蹴る",
            3: "右前足(FR)と左後ろ足(BL)を戻す"
        }
    
    def should_start_walking(self, step: int) -> bool:
        """
        歩行を開始すべきかどうかを判定
        
        Args:
            step: 現在のステップ
            
        Returns:
            歩行を開始すべき場合はTrue
        """
        # 足踏みが開始されていて、指定されたサイクル数を完了したら歩行を開始
        if (self.state.stepping_started and 
            not self.state.walking_started and
            self.state.stepping_cycles_completed >= config.STEPPING_CYCLES_BEFORE_WALKING):
            return True
        return False
    
    def start_walking(self, step: int):
        """
        歩行動作を開始
        
        Args:
            step: 現在のステップ
        """
        if self.should_start_walking(step):
            self.state.walking_started = True
            self.state.walking_start_step = step
            self.state.walking_start_position = self.state.get_current_position()
            self.state.walking_phase = 0
            self.state.walking_phase_start_step = step
            self.state.base_standing_angles_for_walking = self.state.standing_angles.copy()
            
            # 歩行開始時にオレンジ色に変更（立ち上がり中の色と同じ）
            self.robot_model.change_robot_color(config.ROBOT_COLOR_STANDING_UP)
            
            distance = 0.0
            logger.info(f"\n  🚶 歩行動作を開始します（ステップ{step}）...")
            logger.info(f"  目標距離: {config.WALKING_TARGET_DISTANCE}m")
            logger.info(f"  現在位置: ({self.state.walking_start_position[0]:.3f}, {self.state.walking_start_position[1]:.3f}, {self.state.walking_start_position[2]:.3f})")
    
    def update_walking(self, step: int):
        """
        歩行動作を更新
        
        Args:
            step: 現在のステップ
        """
        if not self.state.walking_started:
            return
        
        # 現在の位置を取得して歩行距離を計算（X方向のみ、前進距離）
        current_pos = self.state.get_current_position()
        if self.state.walking_start_position is not None:
            # X方向のみの前進距離を計算（Y方向の移動は無視）
            distance = current_pos[0] - self.state.walking_start_position[0]
            
            # 目標距離に到達したら歩行を終了
            if distance >= config.WALKING_TARGET_DISTANCE:
                logger.info(f"\n  ✅ 歩行完了（ステップ{step}）: {distance:.3f}m前進しました")
                self.state.walking_started = False
                # 歩行完了時に終了待機色（紫系）に変更
                self.robot_model.change_robot_color(config.ROBOT_COLOR_FINISHED)
                return
        
        # 現在のフェーズの経過ステップ数を計算
        phase_elapsed = step - self.state.walking_phase_start_step if self.state.walking_phase_start_step is not None else 0
        
        # フェーズが終了したら次のフェーズに移行
        if phase_elapsed >= config.WALKING_PHASE_DURATION:
            prev_phase = self.state.walking_phase
            self.state.walking_phase = (self.state.walking_phase + 1) % 4
            self.state.walking_phase_start_step = step
            phase_elapsed = 0
            
            logger.info(f"  🚶 歩行フェーズ変更 (ステップ{step}): {self.phase_names[self.state.walking_phase]}")
        
        # 歩行動作中の状態を定期的にログ出力（100ステップごと）
        if step % config.WALKING_LOG_INTERVAL == 0:
            self._log_walking_status(step, phase_elapsed)
        
        # 各フェーズでの角度調整
        if self.state.base_standing_angles_for_walking is not None:
            # 基本姿勢角度をコピー
            walking_angles = {}
            for leg_name in self.robot_model.leg_joints.keys():
                walking_angles[leg_name] = list(self.state.base_standing_angles_for_walking.get(leg_name, [0.0, 0.0, 1.7]))
            
            # フェーズに応じて脚を上げる/前に出す/着地の動作の進行度を計算（3段階）
            phase_third = config.WALKING_PHASE_DURATION / 3.0
            if phase_elapsed < phase_third:
                # 第1段階（0-33%）：足を上げる
                lift_progress = phase_elapsed / phase_third
                forward_progress = 0.0
                land_progress = 0.0
            elif phase_elapsed < phase_third * 2:
                # 第2段階（33-66%）：足を前に出す + 反対の対角を後ろに蹴る
                lift_progress = 1.0
                forward_progress = (phase_elapsed - phase_third) / phase_third
                land_progress = 0.0
            else:
                # 第3段階（66-100%）：足を伸ばして着地
                lift_progress = 1.0
                forward_progress = 1.0
                land_progress = (phase_elapsed - phase_third * 2) / phase_third
            
            # フェーズ0: FL+BRを上げる → 前に出す → 着地、FR+BLを後ろに蹴る
            if self.state.walking_phase == 0:
                # FL+BRを上げる（第1段階）
                walking_angles['front_left'][1] += config.WALKING_HIP_LIFT_ANGLE * lift_progress  # hip: 後ろ向きに
                walking_angles['front_left'][2] -= config.WALKING_KNEE_LIFT_ANGLE * lift_progress  # knee: 曲げる
                walking_angles['back_right'][1] += config.WALKING_HIP_LIFT_ANGLE * lift_progress  # hip: 後ろ向きに
                walking_angles['back_right'][2] -= config.WALKING_KNEE_LIFT_ANGLE * lift_progress  # knee: 曲げる
                
                # FL+BRを前に出す（第2段階：hipを前向きに回す）
                walking_angles['front_left'][1] -= config.WALKING_HIP_FORWARD_ANGLE * forward_progress  # hip: 前向きに（前に出す）
                walking_angles['back_right'][1] -= config.WALKING_HIP_FORWARD_ANGLE * forward_progress  # hip: 前向きに（前に出す）
                
                # FL+BRを伸ばして着地（第3段階：kneeを伸ばす）
                walking_angles['front_left'][2] += config.WALKING_KNEE_LAND_ANGLE * land_progress  # knee: 伸ばす（着地）
                walking_angles['back_right'][2] += config.WALKING_KNEE_LAND_ANGLE * land_progress  # knee: 伸ばす（着地）
                
                # FR+BLを後ろに蹴る（第2段階と第3段階：反作用で前進、着地時も継続）
                # 第2段階と第3段階の両方で蹴る動作を実行することで、より長い時間前進力を維持
                push_progress = forward_progress + land_progress  # 第2段階と第3段階の進行度を合計
                walking_angles['front_right'][1] -= config.WALKING_HIP_PUSH_ANGLE * push_progress  # hip: 前向きに（後ろに蹴る）
                walking_angles['front_right'][2] += config.WALKING_KNEE_PUSH_ANGLE * push_progress  # knee: 伸ばす（後ろに蹴る）
                walking_angles['back_left'][1] -= config.WALKING_HIP_PUSH_ANGLE * push_progress  # hip: 前向きに（後ろに蹴る）
                walking_angles['back_left'][2] += config.WALKING_KNEE_PUSH_ANGLE * push_progress  # knee: 伸ばす（後ろに蹴る）
            
            # フェーズ1: FL+BRを後ろに引っ張って重心を前に移動、FR+BLを後ろに蹴る（前進を維持）
            elif self.state.walking_phase == 1:
                # 進行度を計算（フェーズ1全体で基本姿勢に戻す）
                return_progress = phase_elapsed / config.WALKING_PHASE_DURATION
                
                # FL+BRを後ろに引っ張る（重心を前に移動させる）
                # 前に出した足を後ろに引っ張ることで、重心を前に移動させる
                walking_angles['front_left'][1] += config.WALKING_HIP_PULL_ANGLE * return_progress  # hip: 後ろ向きに（後ろに引っ張る）
                walking_angles['back_right'][1] += config.WALKING_HIP_PULL_ANGLE * return_progress  # hip: 後ろ向きに（後ろに引っ張る）
                
                # FR+BLを後ろに蹴る（前進を維持、強度を維持）
                walking_angles['front_right'][1] -= config.WALKING_HIP_PUSH_ANGLE  # hip: 前向きに（後ろに蹴る）
                walking_angles['front_right'][2] += config.WALKING_KNEE_PUSH_ANGLE  # knee: 伸ばす（後ろに蹴る）
                walking_angles['back_left'][1] -= config.WALKING_HIP_PUSH_ANGLE  # hip: 前向きに（後ろに蹴る）
                walking_angles['back_left'][2] += config.WALKING_KNEE_PUSH_ANGLE  # knee: 伸ばす（後ろに蹴る）
            
            # フェーズ2: FR+BLを上げる → 前に出す → 着地、FL+BRを後ろに蹴る
            elif self.state.walking_phase == 2:
                # FR+BLを上げる（第1段階）
                walking_angles['front_right'][1] += config.WALKING_HIP_LIFT_ANGLE * lift_progress  # hip: 後ろ向きに
                walking_angles['front_right'][2] -= config.WALKING_KNEE_LIFT_ANGLE * lift_progress  # knee: 曲げる
                walking_angles['back_left'][1] += config.WALKING_HIP_LIFT_ANGLE * lift_progress  # hip: 後ろ向きに
                walking_angles['back_left'][2] -= config.WALKING_KNEE_LIFT_ANGLE * lift_progress  # knee: 曲げる
                
                # FR+BLを前に出す（第2段階：hipを前向きに回す）
                walking_angles['front_right'][1] -= config.WALKING_HIP_FORWARD_ANGLE * forward_progress  # hip: 前向きに（前に出す）
                walking_angles['back_left'][1] -= config.WALKING_HIP_FORWARD_ANGLE * forward_progress  # hip: 前向きに（前に出す）
                
                # FR+BLを伸ばして着地（第3段階：kneeを伸ばす）
                walking_angles['front_right'][2] += config.WALKING_KNEE_LAND_ANGLE * land_progress  # knee: 伸ばす（着地）
                walking_angles['back_left'][2] += config.WALKING_KNEE_LAND_ANGLE * land_progress  # knee: 伸ばす（着地）
                
                # FL+BRを後ろに蹴る（第2段階と第3段階：反作用で前進、着地時も継続）
                # 第2段階と第3段階の両方で蹴る動作を実行することで、より長い時間前進力を維持
                push_progress = forward_progress + land_progress  # 第2段階と第3段階の進行度を合計
                walking_angles['front_left'][1] -= config.WALKING_HIP_PUSH_ANGLE * push_progress  # hip: 前向きに（後ろに蹴る）
                walking_angles['front_left'][2] += config.WALKING_KNEE_PUSH_ANGLE * push_progress  # knee: 伸ばす（後ろに蹴る）
                walking_angles['back_right'][1] -= config.WALKING_HIP_PUSH_ANGLE * push_progress  # hip: 前向きに（後ろに蹴る）
                walking_angles['back_right'][2] += config.WALKING_KNEE_PUSH_ANGLE * push_progress  # knee: 伸ばす（後ろに蹴る）
            
            # フェーズ3: FR+BLを後ろに引っ張って重心を前に移動、FL+BRを後ろに蹴る（前進を維持）
            elif self.state.walking_phase == 3:
                # 進行度を計算（フェーズ3全体で基本姿勢に戻す）
                return_progress = phase_elapsed / config.WALKING_PHASE_DURATION
                
                # FR+BLを後ろに引っ張る（重心を前に移動させる）
                # 前に出した足を後ろに引っ張ることで、重心を前に移動させる
                walking_angles['front_right'][1] += config.WALKING_HIP_PULL_ANGLE * return_progress  # hip: 後ろ向きに（後ろに引っ張る）
                walking_angles['back_left'][1] += config.WALKING_HIP_PULL_ANGLE * return_progress  # hip: 後ろ向きに（後ろに引っ張る）
                
                # FL+BRを後ろに蹴る（前進を維持、強度を維持）
                walking_angles['front_left'][1] -= config.WALKING_HIP_PUSH_ANGLE  # hip: 前向きに（後ろに蹴る）
                walking_angles['front_left'][2] += config.WALKING_KNEE_PUSH_ANGLE  # knee: 伸ばす（後ろに蹴る）
                walking_angles['back_right'][1] -= config.WALKING_HIP_PUSH_ANGLE  # hip: 前向きに（後ろに蹴る）
                walking_angles['back_right'][2] += config.WALKING_KNEE_PUSH_ANGLE  # knee: 伸ばす（後ろに蹴る）
            
            self.state.standing_angles = walking_angles
    
    def _log_walking_status(self, step: int, phase_elapsed: int):
        """歩行動作の状態をログ出力"""
        # フェーズ0, 2: 上げる→前に出す→着地
        # フェーズ1, 3: 基本姿勢に戻す + 反対の対角を後ろに蹴る
        if self.state.walking_phase == 0 or self.state.walking_phase == 2:
            phase_third = config.WALKING_PHASE_DURATION / 3.0
            if phase_elapsed < phase_third:
                action_status = f"上げる動作中 (進行度: {phase_elapsed/phase_third*100:.1f}%)"
            elif phase_elapsed < phase_third * 2:
                action_status = f"前に出す/蹴る動作中 (進行度: {(phase_elapsed-phase_third)/phase_third*100:.1f}%)"
            else:
                action_status = f"着地動作中 (進行度: {(phase_elapsed-phase_third*2)/phase_third*100:.1f}%)"
        else:  # フェーズ1, 3
            return_progress = phase_elapsed / config.WALKING_PHASE_DURATION
            action_status = f"基本姿勢に戻す/蹴る動作中 (進行度: {return_progress*100:.1f}%)"
        
        # 現在の姿勢と位置を取得
        base_pos = self.state.get_current_position()
        current_roll = self.state.get_current_roll_deg()
        current_pitch = self.state.get_current_pitch_deg()
        
        # 歩行距離を計算（X方向のみ、前進距離）
        distance = 0.0
        if self.state.walking_start_position is not None:
            # X方向のみの前進距離を計算（Y方向の移動は無視）
            distance = base_pos[0] - self.state.walking_start_position[0]
        
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
        
        Logger.log_walking_status(
            step=step,
            phase=self.state.walking_phase,
            phase_names=self.phase_names,
            action_status=action_status,
            current_roll=current_roll,
            current_pitch=current_pitch,
            base_pos=base_pos,
            distance=distance,
            knee_angles=knee_angles,
            logger=logger,
            contact_states=contact_states
        )
