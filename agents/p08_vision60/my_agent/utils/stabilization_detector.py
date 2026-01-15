"""
安定化検知モジュール

立ち上がり完了後の安定化を検知
"""
import math
import pybullet as p
from typing import Dict, Optional
from .config import config
from .state import Vision60State
from .robot_model import RobotModel
from .logging_config import get_logger

logger = get_logger('stabilization_detector')


class StabilizationDetector:
    """安定化検知クラス"""
    
    def __init__(self, robot_model: RobotModel, state: Vision60State):
        """
        安定化検知器を初期化
        
        Args:
            robot_model: ロボットモデル
            state: 状態管理
        """
        self.robot_model = robot_model
        self.state = state
    
    def check_stabilization(self, step: int) -> bool:
        """
        安定化をチェック
        
        Args:
            step: 現在のステップ
            
        Returns:
            安定化が検知されたかどうか
        """
        # 立ち上がり完了後のみチェック
        if self.state.standing_up_completed_step is None:
            return False
        
        # 立ち上がり完了から一定ステップ経過後からチェック開始
        elapsed_since_completion = step - self.state.standing_up_completed_step
        if elapsed_since_completion < config.STABILITY_CHECK_WINDOW:
            return False
        
        # チェック間隔
        if elapsed_since_completion % config.STABILITY_CHECK_INTERVAL != 0:
            return False
        
        # 安定性指標を計算（簡易版）
        # 実際の実装では、姿勢変化率、位置変化率などを計算
        
        # 簡易版：立ち上がり完了から一定時間経過したら安定化とみなす
        if elapsed_since_completion >= 200:  # 約0.8秒後
            if not self.state.stabilization_detected:
                self.state.stabilization_detected = True
                logger.info(f"  🎯 安定化検知！ (ステップ{step}, 立ち上がり完了から{elapsed_since_completion}ステップ後)")
                return True
        
        return False
