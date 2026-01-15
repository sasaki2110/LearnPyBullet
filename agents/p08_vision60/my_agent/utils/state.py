"""
状態管理モジュール

Vision60エージェントの状態を一元管理
"""
from typing import Dict, List, Optional, Tuple
import math
import pybullet as p
from .config import config


class Vision60State:
    """Vision60エージェントの状態管理クラス"""
    
    def __init__(self, robot_id: int):
        """
        状態を初期化
        
        Args:
            robot_id: PyBulletのロボットID
        """
        self.robot_id = robot_id
        
        # 現在の姿勢角度（各脚の[abduction, hip, knee]）
        self.standing_angles: Dict[str, List[float]] = config.INITIAL_STANDING_ANGLES.copy()
        
        # リセット関連
        self.reset_count: int = 0
        self.reset_reasons_count: Dict[str, int] = {
            '位置移動': 0,
            '高さ低下': 0,
            '姿勢傾き': 0
        }
        self.initial_pos_after_setup: Optional[Tuple[float, float, float]] = None
        self.target_height: Optional[float] = None
        
        # 立ち上がり制御関連
        self.stability_confirmed: bool = False
        self.stability_check_start: Optional[int] = None
        self.standing_up_angles: Optional[Dict[str, List[float]]] = None
        self.initial_standing_angles: Optional[Dict[str, List[float]]] = None
        self.standing_up_start_step: Optional[int] = None
        self.standing_up_completed_step: Optional[int] = None
        
        # 安定化検知関連
        self.stabilization_detected: bool = False
        self.prev_stability_metrics: Optional[Dict] = None
        
        # 足踏み動作関連
        self.stepping_started: bool = False
        self.stepping_phase: int = 0  # 0: FL+BR上げ, 1: FL+BR戻す, 2: FR+BL上げ, 3: FR+BL戻す
        self.stepping_phase_start_step: Optional[int] = None
        self.base_standing_angles_for_stepping: Optional[Dict[str, List[float]]] = None
        
        # 前回のログ値（差分計算用）
        self.prev_roll: Optional[float] = None
        self.prev_pitch: Optional[float] = None
        self.prev_knee_angles: Optional[Dict[str, float]] = None
        self.prev_toe_heights: Optional[Dict[str, float]] = None
        self.prev_base_pos: Optional[Tuple[float, float, float]] = None
    
    def get_current_pose(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
        """
        現在の姿勢を取得
        
        Returns:
            (位置, 姿勢クォータニオン)のタプル
        """
        return p.getBasePositionAndOrientation(self.robot_id)
    
    def get_current_euler(self) -> Tuple[float, float, float]:
        """
        現在の姿勢をオイラー角で取得
        
        Returns:
            (roll, pitch, yaw)のタプル（ラジアン）
        """
        _, orn = self.get_current_pose()
        return p.getEulerFromQuaternion(orn)
    
    def get_current_roll_deg(self) -> float:
        """現在のrollを度で取得"""
        euler = self.get_current_euler()
        return math.degrees(euler[0])
    
    def get_current_pitch_deg(self) -> float:
        """現在のpitchを度で取得"""
        euler = self.get_current_euler()
        return math.degrees(euler[1])
    
    def get_current_yaw_deg(self) -> float:
        """現在のyawを度で取得"""
        euler = self.get_current_euler()
        return math.degrees(euler[2])
    
    def get_current_position(self) -> Tuple[float, float, float]:
        """現在の位置を取得"""
        pos, _ = self.get_current_pose()
        return pos
    
    def update_initial_height(self):
        """初期高さを記録（リセットチェック用）"""
        if self.initial_pos_after_setup is None:
            self.initial_pos_after_setup = self.get_current_position()
            self.target_height = self.initial_pos_after_setup[2]
    
    def increment_reset_count(self, reason: str):
        """
        リセット回数を増加
        
        Args:
            reason: リセット理由
        """
        self.reset_count += 1
        if reason in self.reset_reasons_count:
            self.reset_reasons_count[reason] += 1
