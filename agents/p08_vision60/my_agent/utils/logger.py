"""
ログ出力モジュール

Vision60エージェントのログ出力を統一管理
"""
from typing import Dict, Optional, List
import math
import pybullet as p
from .config import config
from .logging_config import get_logger

logger = get_logger('logger')


class Logger:
    """ログ出力クラス"""
    
    def __init__(self):
        """ロガーを初期化"""
        pass
    
    @classmethod
    def log_standing_up_progress(
        cls,
        step: int,
        progress: float,
        current_roll: float,
        current_pitch: float,
        base_pos: tuple,
        knee_angles: Dict[str, float],
        target_knee_deg: float,
        contact_states: Dict[str, tuple],
        toe_positions: Dict[str, Optional[tuple]],
        angle_errors: Dict[str, List[float]],
        prev_roll: Optional[float] = None,
        prev_pitch: Optional[float] = None,
        prev_knee_angles: Optional[Dict[str, float]] = None,
        prev_base_pos: Optional[tuple] = None,
        prev_toe_heights: Optional[Dict[str, float]] = None
    ):
        """
        立ち上がり進行度をログ出力
        
        Args:
            step: 現在のステップ
            progress: 進行度（0.0～1.0）
            current_roll: 現在のroll（度）
            current_pitch: 現在のpitch（度）
            base_pos: ベース位置 (x, y, z)
            knee_angles: 各脚の膝角度（度）
            target_knee_deg: 目標膝角度（度）
            contact_states: 各脚の接地状態 {(leg_name): (contact_points, force)}
            toe_positions: 各脚の足先位置 {(leg_name): (x, y, z) or None}
            prev_roll: 前回のroll（度）
            prev_pitch: 前回のpitch（度）
            prev_knee_angles: 前回の膝角度
            prev_base_pos: 前回のベース位置
        """
        # 変化量を計算
        roll_change = current_roll - prev_roll if prev_roll is not None else 0.0
        pitch_change = current_pitch - prev_pitch if prev_pitch is not None else 0.0
        
        base_pos_change = (
            base_pos[0] - prev_base_pos[0] if prev_base_pos else 0.0,
            base_pos[1] - prev_base_pos[1] if prev_base_pos else 0.0,
            base_pos[2] - prev_base_pos[2] if prev_base_pos else 0.0
        )
        
        knee_angle_changes = {}
        if prev_knee_angles:
            for leg_name in knee_angles:
                prev_angle = prev_knee_angles.get(leg_name, 0.0)
                knee_angle_changes[leg_name] = knee_angles[leg_name] - prev_angle
        else:
            for leg_name in knee_angles:
                knee_angle_changes[leg_name] = 0.0
        
        # 膝角度差を計算
        left_knee_avg = (knee_angles.get('front_left', 0.0) + knee_angles.get('back_left', 0.0)) / 2.0
        right_knee_avg = (knee_angles.get('front_right', 0.0) + knee_angles.get('back_right', 0.0)) / 2.0
        front_knee_avg = (knee_angles.get('front_left', 0.0) + knee_angles.get('front_right', 0.0)) / 2.0
        back_knee_avg = (knee_angles.get('back_left', 0.0) + knee_angles.get('back_right', 0.0)) / 2.0
        
        knee_lr_diff = left_knee_avg - right_knee_avg
        knee_fb_diff = front_knee_avg - back_knee_avg
        
        # 足先位置の変化量
        toe_height_changes = {}
        if prev_base_pos:
            for leg_name, toe_pos in toe_positions.items():
                if toe_pos:
                    # 前回の値は計算できないので、0.0とする
                    toe_height_changes[leg_name] = 0.0
                else:
                    toe_height_changes[leg_name] = 0.0
        else:
            for leg_name in toe_positions:
                toe_height_changes[leg_name] = 0.0
        
        # 平均足先高さ
        toe_heights = [pos[2] for pos in toe_positions.values() if pos]
        avg_toe_height = sum(toe_heights) / len(toe_heights) if toe_heights else 0.0
        
        # 足先の高さの変化を計算
        toe_height_diffs = {}
        if prev_toe_heights:
            for leg_name in ['front_left', 'front_right', 'back_left', 'back_right']:
                if toe_positions.get(leg_name) and prev_toe_heights.get(leg_name) is not None:
                    toe_height_diffs[leg_name] = toe_positions[leg_name][2] - prev_toe_heights[leg_name]
                else:
                    toe_height_diffs[leg_name] = 0.0
        else:
            for leg_name in ['front_left', 'front_right', 'back_left', 'back_right']:
                toe_height_diffs[leg_name] = 0.0
        
        logger.info(f"  📈 立ち上がり進行中 (ステップ{step}, 進行度{progress*100:.1f}%):")
        logger.info(f"     姿勢: roll={current_roll:.1f}° (変化: {roll_change:+.1f}°), pitch={current_pitch:.1f}° (変化: {pitch_change:+.1f}°)")
        logger.info(f"     ベース位置: X={base_pos[0]:.3f} ({base_pos_change[0]:+.3f}), Y={base_pos[1]:.3f} ({base_pos_change[1]:+.3f}), Z={base_pos[2]:.3f} ({base_pos_change[2]:+.3f})")
        logger.info(f"     膝角度: FL={knee_angles.get('front_left', 0.0):.1f}° ({knee_angle_changes.get('front_left', 0.0):+.1f}°), "
              f"FR={knee_angles.get('front_right', 0.0):.1f}° ({knee_angle_changes.get('front_right', 0.0):+.1f}°), "
              f"BL={knee_angles.get('back_left', 0.0):.1f}° ({knee_angle_changes.get('back_left', 0.0):+.1f}°), "
              f"BR={knee_angles.get('back_right', 0.0):.1f}° ({knee_angle_changes.get('back_right', 0.0):+.1f}°) (目標={target_knee_deg:.1f}°)")
        logger.info(f"     膝角度差: 左右差={knee_lr_diff:+.1f}° (左-右), 前後差={knee_fb_diff:+.1f}° (前-後)")
        
        # 角度誤差
        logger.info(f"     角度誤差: FL(abd={angle_errors.get('front_left', [0,0,0])[0]:+.1f}°, hip={angle_errors.get('front_left', [0,0,0])[1]:+.1f}°, knee={angle_errors.get('front_left', [0,0,0])[2]:+.1f}°), "
              f"FR(abd={angle_errors.get('front_right', [0,0,0])[0]:+.1f}°, hip={angle_errors.get('front_right', [0,0,0])[1]:+.1f}°, knee={angle_errors.get('front_right', [0,0,0])[2]:+.1f}°)")
        logger.info(f"             BL(abd={angle_errors.get('back_left', [0,0,0])[0]:+.1f}°, hip={angle_errors.get('back_left', [0,0,0])[1]:+.1f}°, knee={angle_errors.get('back_left', [0,0,0])[2]:+.1f}°), "
              f"BR(abd={angle_errors.get('back_right', [0,0,0])[0]:+.1f}°, hip={angle_errors.get('back_right', [0,0,0])[1]:+.1f}°, knee={angle_errors.get('back_right', [0,0,0])[2]:+.1f}°)")
        
        # 接地状態
        fl_contact = contact_states.get('front_left', (0, 0.0))
        fr_contact = contact_states.get('front_right', (0, 0.0))
        bl_contact = contact_states.get('back_left', (0, 0.0))
        br_contact = contact_states.get('back_right', (0, 0.0))
        logger.info(f"     接地状態: FL={'接地' if fl_contact[0] > 0 else '浮上'}({fl_contact[0]}点, {fl_contact[1]:.1f}N), "
              f"FR={'接地' if fr_contact[0] > 0 else '浮上'}({fr_contact[0]}点, {fr_contact[1]:.1f}N), "
              f"BL={'接地' if bl_contact[0] > 0 else '浮上'}({bl_contact[0]}点, {bl_contact[1]:.1f}N), "
              f"BR={'接地' if br_contact[0] > 0 else '浮上'}({br_contact[0]}点, {br_contact[1]:.1f}N)")
        
        # 足先位置（詳細版）
        if toe_positions:
            fl_pos = toe_positions.get('front_left')
            fr_pos = toe_positions.get('front_right')
            bl_pos = toe_positions.get('back_left')
            br_pos = toe_positions.get('back_right')
            
            if fl_pos and fr_pos and bl_pos and br_pos:
                logger.info(f"     足先位置: FL=({fl_pos[0]:.3f}, {fl_pos[1]:.3f}, {fl_pos[2]:.3f} ({toe_height_diffs.get('front_left', 0.0):+.3f})), "
                      f"FR=({fr_pos[0]:.3f}, {fr_pos[1]:.3f}, {fr_pos[2]:.3f} ({toe_height_diffs.get('front_right', 0.0):+.3f}))")
                logger.info(f"                BL=({bl_pos[0]:.3f}, {bl_pos[1]:.3f}, {bl_pos[2]:.3f} ({toe_height_diffs.get('back_left', 0.0):+.3f})), "
                      f"BR=({br_pos[0]:.3f}, {br_pos[1]:.3f}, {br_pos[2]:.3f} ({toe_height_diffs.get('back_right', 0.0):+.3f}))")
                logger.info(f"     平均足先高さ: {avg_toe_height:.3f}m")
            else:
                logger.info(f"     足先位置: 取得失敗（一部のリンクが見つかりませんでした）")
    
    @classmethod
    def log_stepping_status(
        cls,
        step: int,
        phase: int,
        phase_names: Dict[int, str],
        action_status: str,
        current_roll: float,
        current_pitch: float,
        base_pos: tuple,
        knee_angles: Dict[str, float]
    ):
        """
        足踏み動作の状態をログ出力
        
        Args:
            step: 現在のステップ
            phase: 現在のフェーズ
            phase_names: フェーズ名のマッピング
            action_status: 動作状態（"上げる動作中"など）
            current_roll: 現在のroll（度）
            current_pitch: 現在のpitch（度）
            base_pos: ベース位置
            knee_angles: 各脚の膝角度（度）
        """
        logger.info(f"  🦶 足踏み動作中 (ステップ{step}, フェーズ{phase}: {phase_names.get(phase, 'Unknown')}, {action_status}):")
        logger.info(f"     姿勢: roll={current_roll:.1f}°, pitch={current_pitch:.1f}°")
        logger.info(f"     位置: X={base_pos[0]:.3f}, Y={base_pos[1]:.3f}, Z={base_pos[2]:.3f}")
        logger.info(f"     膝角度: FL={knee_angles.get('front_left', 0.0):.1f}°, "
              f"FR={knee_angles.get('front_right', 0.0):.1f}°, "
              f"BL={knee_angles.get('back_left', 0.0):.1f}°, "
              f"BR={knee_angles.get('back_right', 0.0):.1f}°")
    
    @classmethod
    def log_reset_stats(cls, reset_count: int, reset_reasons_count: Dict[str, int], total_steps: int):
        """
        リセット統計をログ出力
        
        Args:
            reset_count: 総リセット回数
            reset_reasons_count: 原因別リセット回数
            total_steps: 総ステップ数
        """
        logger.info(f"\n📊 リセット統計:")
        logger.info(f"  総リセット回数: {reset_count}回 ({total_steps}ステップ中)")
        logger.info(f"  原因別内訳:")
        for reason, count in reset_reasons_count.items():
            if count > 0:
                logger.info(f"    - {reason}: {count}回")
    
    @classmethod
    def log_final_state(cls, robot_id: int, robot_model):
        """
        最終状態をログ出力
        
        Args:
            robot_id: ロボットID
            robot_model: ロボットモデル
        """
        pos, orn = p.getBasePositionAndOrientation(robot_id)
        euler = p.getEulerFromQuaternion(orn)
        
        logger.info(f"\n📊 現在の姿勢:")
        logger.info(f"  位置: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
        logger.info(f"  姿勢: roll={math.degrees(euler[0]):.1f}°, pitch={math.degrees(euler[1]):.1f}°, yaw={math.degrees(euler[2]):.1f}°")
