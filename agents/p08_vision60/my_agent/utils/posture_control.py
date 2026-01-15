"""
姿勢フィードバック制御モジュール

roll/pitchに基づく姿勢フィードバック制御
"""
from .config import config
from .state import Vision60State
from .robot_model import RobotModel


class PostureController:
    """姿勢フィードバック制御クラス"""
    
    def __init__(self, robot_model: RobotModel, state: Vision60State):
        """
        姿勢コントローラーを初期化
        
        Args:
            robot_model: ロボットモデル
            state: 状態管理
        """
        self.robot_model = robot_model
        self.state = state
    
    def apply_posture_feedback(self, is_standing_up: bool = False, is_stepping: bool = False):
        """
        姿勢フィードバック制御を適用
        
        Args:
            is_standing_up: 立ち上がり中かどうか
            is_stepping: 足踏み中または歩行中かどうか
        """
        # 足踏み中または歩行中は姿勢フィードバック制御を無効化（動作と競合しないように）
        if is_stepping:
            return
        
        # ゲインの倍率を決定
        gain_multiplier = 2.0 if is_standing_up else 1.0
        
        # 現在の姿勢を取得
        roll = self.state.get_current_roll_deg()
        pitch = self.state.get_current_pitch_deg()
        
        # rollフィードバック（左右バランス）
        roll_error = roll  # 目標roll=0度
        if abs(roll_error) > 0.1:  # 0.1度以上の誤差がある場合
            roll_adjustment = roll_error * config.ROLL_FEEDBACK_GAIN * gain_multiplier
            # 左に傾いている場合、左側の脚のabductionを増やす
            if roll_error < 0:  # 左に傾いている
                self.state.standing_angles['front_left'][0] += abs(roll_adjustment)
                self.state.standing_angles['back_left'][0] += abs(roll_adjustment)
            else:  # 右に傾いている
                self.state.standing_angles['front_right'][0] -= abs(roll_adjustment)
                self.state.standing_angles['back_right'][0] -= abs(roll_adjustment)
        
        # pitchフィードバック（前後バランス）
        pitch_error = pitch  # 目標pitch=0度
        if abs(pitch_error) > 0.1:  # 0.1度以上の誤差がある場合
            pitch_adjustment = pitch_error * config.PITCH_FEEDBACK_GAIN * gain_multiplier
            # 前のめりの場合、前脚のhipを後ろ向きに、後脚のhipを前向きに
            if pitch_error > 0:  # 前のめり
                self.state.standing_angles['front_left'][1] += abs(pitch_adjustment)
                self.state.standing_angles['front_right'][1] += abs(pitch_adjustment)
                self.state.standing_angles['back_left'][1] -= abs(pitch_adjustment)
                self.state.standing_angles['back_right'][1] -= abs(pitch_adjustment)
            else:  # 後ろのめり
                self.state.standing_angles['front_left'][1] -= abs(pitch_adjustment)
                self.state.standing_angles['front_right'][1] -= abs(pitch_adjustment)
                self.state.standing_angles['back_left'][1] += abs(pitch_adjustment)
                self.state.standing_angles['back_right'][1] += abs(pitch_adjustment)
