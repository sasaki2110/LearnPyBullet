"""
前方転倒チェックモジュール

4本の足を同時に後ろに蹴って、前に倒れる（重心移動）ことができるかをチェック
"""
import math
import pybullet as p
from typing import Dict, List
from .config import config
from .state import Vision60State
from .robot_model import RobotModel
from .logging_config import get_logger
from .logger import Logger

logger = get_logger('falling_forward_check')


class FallingForwardCheck:
    """前方転倒チェッククラス"""
    
    def __init__(self, robot_model: RobotModel, state: Vision60State):
        """
        前方転倒テストを初期化
        
        Args:
            robot_model: ロボットモデル
            state: 状態管理
        """
        self.robot_model = robot_model
        self.state = state
        self.check_started = False
        self.check_start_step = None
        self.check_start_position = None
        self.base_standing_angles_for_check = None
        
        # チェックパラメータ
        self.CHECK_DURATION = 2000  # チェック継続時間（ステップ数）
        # 前脚を前に出す（重心を前に移動）
        self.FRONT_LEG_FORWARD_HIP = 0.8  # 前脚を前に出す際のhip角度調整量（ラジアン、後ろ向きに回す = 前に出す）
        self.FRONT_LEG_DOWN_KNEE = -0.2  # 前脚を下げる際のknee角度調整量（ラジアン、曲げる）
        # 後脚を後ろに蹴る（反作用で前進）
        self.BACK_LEG_KICK_HIP = 1.0  # 後脚を後ろに蹴る際のhip角度調整量（ラジアン、前向きに回す = 後ろに蹴る）
        self.BACK_LEG_KICK_KNEE = 0.5  # 後脚を後ろに蹴る際のknee角度調整量（ラジアン、伸ばす）
    
    def should_start_check(self, step: int) -> bool:
        """
        チェックを開始すべきかどうかを判定
        
        Args:
            step: 現在のステップ
            
        Returns:
            チェックを開始すべき場合はTrue
        """
        # 安定化検知後に自動的に開始
        if (self.state.stabilization_detected and 
            not self.check_started and
            not self.state.stepping_started):
            return True
        return False
    
    def start_check(self, step: int):
        """
        前方転倒チェックを開始
        
        Args:
            step: 現在のステップ
        """
        if self.should_start_check(step):
            self.check_started = True
            self.check_start_step = step
            self.check_start_position = self.state.get_current_position()
            self.base_standing_angles_for_check = self.state.standing_angles.copy()
            
            # チェック開始時に赤色（透明度低め）に変更
            self.robot_model.change_robot_color(config.ROBOT_COLOR_FALLING_FORWARD_CHECK)
            
            logger.info(f"\n  🧪 前方転倒チェックを開始します（ステップ{step}）...")
            logger.info(f"  現在位置: ({self.check_start_position[0]:.3f}, {self.check_start_position[1]:.3f}, {self.check_start_position[2]:.3f})")
            logger.info(f"  前脚を前に出して重心を前に移動、後脚を後ろに蹴って前に倒れる動作をチェックします")
            # 基本姿勢角度をログ出力
            if self.base_standing_angles_for_check is not None:
                logger.info(f"  基本姿勢角度:")
                for leg_name, angles in self.base_standing_angles_for_check.items():
                    hip_deg = math.degrees(angles[1])
                    knee_deg = math.degrees(angles[2])
                    logger.info(f"    {leg_name}: hip={hip_deg:.1f}°, knee={knee_deg:.1f}°")
    
    def update_check(self, step: int):
        """
        前方転倒チェックを更新
        
        Args:
            step: 現在のステップ
        """
        if not self.check_started:
            return
        
        # チェック継続時間をチェック
        check_elapsed = step - self.check_start_step if self.check_start_step is not None else 0
        if check_elapsed >= self.CHECK_DURATION:
            logger.info(f"\n  ✅ 前方転倒チェック完了（ステップ{step}）")
            self.check_started = False
            # チェック完了時に終了待機色（紫系）に変更
            self.robot_model.change_robot_color(config.ROBOT_COLOR_FINISHED)
            return
        
        # 現在の位置を取得
        current_pos = self.state.get_current_position()
        if self.check_start_position is not None:
            # X方向のみの前進距離を計算
            distance = current_pos[0] - self.check_start_position[0]
            
            # 定期的にログ出力（100ステップごと）
            if step % 100 == 0:
                self._log_check_status(step, check_elapsed, current_pos, distance)
        
        # 前脚を前に出して重心を前に移動、後脚を後ろに蹴る
        if self.base_standing_angles_for_check is not None:
            # 基本姿勢角度をコピー
            check_angles = {}
            for leg_name in self.robot_model.leg_joints.keys():
                check_angles[leg_name] = list(self.base_standing_angles_for_check.get(leg_name, [0.0, 0.0, 1.7]))
            
            # チェックの進行度を計算（0.0～1.0）
            check_progress = check_elapsed / self.CHECK_DURATION
            
            # 前に倒れる動作：前脚を前に出して重心を前に移動、後脚を後ろに蹴る
            # 進行度に応じて、徐々に強く動作する（最初は弱く、最後は強く）
            action_progress = check_progress  # 0.0～1.0
            
            # 前脚を前に出す（重心を前に移動）
            # 座標系の非対称性を考慮：左足と右足で符号を反転する必要がある可能性
            # ログを見ると、左前脚（FL）のhip角度がほとんど変化していないため、
            # 左足と右足で符号を反転する必要がある可能性がある
            # 前脚を前に出す：hipを後ろ向きに回す（左足は`+=`、右足は`-=`で符号を反転）
            check_angles['front_left'][1] += self.FRONT_LEG_FORWARD_HIP * action_progress
            check_angles['front_left'][2] += self.FRONT_LEG_DOWN_KNEE * action_progress
            check_angles['front_right'][1] -= self.FRONT_LEG_FORWARD_HIP * action_progress  # 右足は符号を反転
            check_angles['front_right'][2] += self.FRONT_LEG_DOWN_KNEE * action_progress
            
            # 後脚を後ろに蹴る（反作用で前進）
            # 座標系の非対称性を考慮：左足と右足で符号を反転する必要がある可能性
            # 後脚を後ろに蹴る：hipを前向きに回す（左足は`-=`、右足は`+=`で符号を反転）
            check_angles['back_left'][1] -= self.BACK_LEG_KICK_HIP * action_progress
            check_angles['back_left'][2] += self.BACK_LEG_KICK_KNEE * action_progress
            check_angles['back_right'][1] += self.BACK_LEG_KICK_HIP * action_progress  # 右足は符号を反転
            check_angles['back_right'][2] += self.BACK_LEG_KICK_KNEE * action_progress
            
            # デバッグ：100ステップごとに角度をログ出力
            if step % 100 == 0:
                logger.debug(f"  角度調整後（進行度{action_progress*100:.1f}%）:")
                for leg_name in ['front_left', 'front_right', 'back_left', 'back_right']:
                    hip_deg = math.degrees(check_angles[leg_name][1])
                    knee_deg = math.degrees(check_angles[leg_name][2])
                    logger.debug(f"    {leg_name}: hip={hip_deg:.1f}°, knee={knee_deg:.1f}°")
            
            self.state.standing_angles = check_angles
    
    def _log_check_status(self, step: int, check_elapsed: int, current_pos: List[float], distance: float):
        """チェックの状態をログ出力"""
        # 現在の姿勢を取得
        current_roll = self.state.get_current_roll_deg()
        current_pitch = self.state.get_current_pitch_deg()
        
        # 各脚の膝角度とhip角度を取得
        knee_angles = {}
        hip_angles = {}
        for leg_name, joint_indices in self.robot_model.leg_joints.items():
            hip_joint = joint_indices[1]
            knee_joint = joint_indices[2]
            hip_state = p.getJointState(self.state.robot_id, hip_joint)
            knee_state = p.getJointState(self.state.robot_id, knee_joint)
            hip_angles[leg_name] = math.degrees(hip_state[0])
            knee_angles[leg_name] = math.degrees(knee_state[0])
        
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
        
        check_progress_pct = (check_elapsed / self.CHECK_DURATION) * 100
        
        logger.info(f"  🧪 前方転倒チェック中 (ステップ{step}, 進行度: {check_progress_pct:.1f}%):")
        logger.info(f"      姿勢: roll={current_roll:.1f}°, pitch={current_pitch:.1f}°")
        logger.info(f"      位置: X={current_pos[0]:.3f}, Y={current_pos[1]:.3f}, Z={current_pos[2]:.3f}")
        logger.info(f"      前進距離: {distance:.3f}m")
        logger.info(f"      前脚hip角度: FL={hip_angles['front_left']:.1f}°, FR={hip_angles['front_right']:.1f}° (前に出す)")
        logger.info(f"      後脚hip角度: BL={hip_angles['back_left']:.1f}°, BR={hip_angles['back_right']:.1f}° (後ろに蹴る)")
        logger.info(f"      前脚膝角度: FL={knee_angles['front_left']:.1f}°, FR={knee_angles['front_right']:.1f}°")
        logger.info(f"      後脚膝角度: BL={knee_angles['back_left']:.1f}°, BR={knee_angles['back_right']:.1f}°")
        
        contact_str = ", ".join([f"{leg}={'接地' if count > 0 else '浮上'}({count}点, {force:.1f}N)" 
                                 for leg, (count, force) in contact_states.items()])
        logger.info(f"      接地状態: {contact_str}")
