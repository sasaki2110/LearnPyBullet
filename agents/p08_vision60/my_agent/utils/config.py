"""
Vision60エージェントの設定・定数

すべての設定値と定数を一元管理
"""
from dataclasses import dataclass, field
from typing import Dict, List


@dataclass
class Config:
    """Vision60エージェントの設定クラス"""
    
    # PyBullet設定
    GUI_MODE: bool = True
    TIME_STEP: float = 1.0 / 480.0  # より細かいタイムステップで物理シミュレーションの精度を向上
    GRAVITY: tuple = (0, 0, -9.81)
    
    # ロボット初期位置
    INITIAL_POS: List[float] = None  # [0, 0, 0.1]
    
    # 物理パラメータ
    BASE_MASS: float = 15.0  # ベースリンクの質量（kg）
    BASE_LINEAR_DAMPING: float = 0.5
    BASE_ANGULAR_DAMPING: float = 0.5
    BASE_RESTITUTION: float = 0.1
    BASE_LATERAL_FRICTION: float = 1.0
    BASE_SPINNING_FRICTION: float = 0.5
    BASE_ROLLING_FRICTION: float = 0.5
    
    LINK_LINEAR_DAMPING: float = 0.3
    LINK_ANGULAR_DAMPING: float = 0.3
    LINK_RESTITUTION: float = 0.1
    LINK_LATERAL_FRICTION: float = 1.0
    LINK_SPINNING_FRICTION: float = 0.5
    LINK_ROLLING_FRICTION: float = 0.5
    
    JOINT_DAMPING: float = 0.5
    
    # 地面の物理パラメータ
    PLANE_RESTITUTION: float = 0.1
    PLANE_LATERAL_FRICTION: float = 1.0
    PLANE_SPINNING_FRICTION: float = 0.5
    PLANE_ROLLING_FRICTION: float = 0.5
    
    # カメラ設定
    CAMERA_DISTANCE: float = 2.0
    #CAMERA_YAW: float = 45.0
    CAMERA_YAW: float = 45.0
    CAMERA_PITCH: float = -30.0
    
    # ジョイント構造
    LEG_JOINTS: Dict[str, List[int]] = None  # 後で初期化
    
    # 初期姿勢角度（脚を閉じた状態）
    INITIAL_STANDING_ANGLES: Dict[str, List[float]] = None  # 後で初期化
    
    # PD制御パラメータ
    POSITION_GAIN: float = 0.2  # 位置ゲイン（Kp）
    VELOCITY_GAIN: float = 2.0  # 速度ゲイン（Kd）
    
    # 姿勢フィードバック制御のゲイン
    ROLL_FEEDBACK_GAIN: float = 0.01
    PITCH_FEEDBACK_GAIN: float = 0.005
    
    # 位置フィードバック制御のゲイン
    POSITION_FEEDBACK_GAIN: float = 0.001
    
    # 接地状態フィードバック制御のゲイン
    CONTACT_FEEDBACK_GAIN: float = 0.05
    CONTACT_HIP_FEEDBACK_GAIN: float = 0.02
    
    # 傾き修正用のゲイン
    TILT_CORRECTION_KNEE_GAIN: float = 0.1
    
    # 立ち上がり制御パラメータ
    STANDING_UP_DURATION: int = 2000  # 立ち上がりにかけるステップ数
    STABILITY_CHECK_STEPS: int = 20  # 安定確認に必要なステップ数
    STABILITY_SETTLE_STEPS: int = 100  # 姿勢設定後の安定化ステップ数
    
    # 立ち上がり目標角度
    STANDING_UP_ANGLES: Dict[str, List[float]] = None  # 後で初期化
    
    # 立ち上がり中の力とゲイン
    STANDING_UP_FORCE: float = 50.0
    STANDING_UP_POSITION_GAIN_MULTIPLIER: float = 0.5
    STANDING_UP_VELOCITY_GAIN_MULTIPLIER: float = 0.5
    
    # リセット設定
    RESET_CHECK_INTERVAL: int = 50  # リセットチェックの間隔（ステップ）
    RESET_POSITION_THRESHOLD: float = 0.20  # 位置移動の閾値（m）
    RESET_HEIGHT_THRESHOLD_RATIO: float = 0.30  # 高さ低下の閾値（比率）
    RESET_ORIENTATION_THRESHOLD: float = 15.0  # 姿勢傾きの閾値（度）
    RESET_ORIENTATION_THRESHOLD_STANDING_UP: float = 25.0  # 立ち上がり中の姿勢傾きの閾値（度）
    
    # 安定化検知設定
    STABILITY_CHECK_WINDOW: int = 20  # 安定性チェックのウィンドウサイズ（ステップ数）
    STABILITY_CHECK_INTERVAL: int = 10  # 安定性チェックの間隔（ステップ数）
    
    # 足踏み動作設定
    STEPPING_PHASE_DURATION: int = 240  # 各フェーズの継続時間（ステップ数、0.5秒 = 240ステップ @ 1/480秒、1サイクル=2秒）
    STEPPING_CYCLES_BEFORE_WALKING: int = 2  # 歩行開始前に必要な足踏みサイクル数
    LEG_LIFT_HEIGHT: float = 0.05  # 足を上げる高さ（m、未使用）
    
    # 歩行動作設定
    WALKING_PHASE_DURATION: int = 240  # 各フェーズの継続時間（ステップ数、足踏みと同じ）
    WALKING_TARGET_DISTANCE: float = 3.0  # 目標歩行距離（m）
    # 足を上げる動作
    WALKING_HIP_LIFT_ANGLE: float = 0.3  # 足を上げる際のhip角度調整量（ラジアン、後ろ向きに）
    WALKING_KNEE_LIFT_ANGLE: float = 0.5  # 足を上げる際のknee角度調整量（ラジアン、曲げる）
    # 足を前に出す動作
    WALKING_HIP_FORWARD_ANGLE: float = 0.5  # 前に出す際のhip角度調整量（ラジアン、前向きに、0.4→0.5に増加）
    # 足を着地させる動作
    WALKING_KNEE_LAND_ANGLE: float = 0.4  # 着地時にkneeを伸ばす角度調整量（ラジアン、0.3→0.4に増加）
    # 後ろに蹴る動作
    WALKING_HIP_PUSH_ANGLE: float = 0.6  # 後ろに蹴る際のhip角度調整量（ラジアン、前向きに、0.5→0.6に増加）
    WALKING_KNEE_PUSH_ANGLE: float = 0.3  # 後ろに蹴る際のknee角度調整量（ラジアン、kneeを伸ばして後ろに蹴る、0.2→0.3に増加）
    # 足を後ろに引っ張る動作（重心を前に移動）
    WALKING_HIP_PULL_ANGLE: float = 0.4  # 後ろに引っ張る際のhip角度調整量（ラジアン、後ろ向きに、0.3→0.4に増加）
    
    # ロボットの色設定（RGBA、各値は0.0～1.0）
    # アルファ値は0.5（半透明）に設定（change_robot_colorメソッドで自動的に適用される）
    ROBOT_COLOR_NORMAL: List[float] = field(default_factory=lambda: [0.3, 0.5, 0.8, 1.0])  # 通常時: 青系
    ROBOT_COLOR_STEPPING: List[float] = field(default_factory=lambda: [0.2, 0.8, 0.3, 1.0])  # 足踏み中: 緑系
    ROBOT_COLOR_STANDING_UP: List[float] = field(default_factory=lambda: [0.9, 0.6, 0.2, 1.0])  # 立ち上がり中: オレンジ系
    ROBOT_COLOR_FINISHED: List[float] = field(default_factory=lambda: [0.8, 0.2, 0.8, 1.0])  # 終了待機中: 紫系
    ROBOT_COLOR_FALLING_FORWARD_CHECK: List[float] = field(default_factory=lambda: [0.9, 0.1, 0.1, 0.3])  # 前方転倒チェック中: 赤系（透明度低め）
    
    # シミュレーション設定
    # 足踏み開始（約ステップ2220）から0.5秒×4フェーズ（2秒/サイクル）を複数サイクル実行
    # その後、歩行を3m実行（約20-30サイクル必要と仮定）
    # 計算: 2220（足踏み開始） + 240×4×2（2サイクル×2秒） + 240×4×30（30サイクル×2秒） = 約31000ステップ → 35000ステップ（余裕を持たせる）
    TOTAL_SIMULATION_STEPS: int = 35000  # 総シミュレーションステップ数
    
    # ログ出力設定
    STANDING_UP_LOG_INTERVAL: int = 100  # 立ち上がり進行度ログの間隔（ステップ数）
    STEPPING_LOG_INTERVAL: int = 100  # 足踏み動作ログの間隔（ステップ数）
    WALKING_LOG_INTERVAL: int = 100  # 歩行動作ログの間隔（ステップ数）
    
    def __post_init__(self):
        """初期化後の処理"""
        if self.INITIAL_POS is None:
            self.INITIAL_POS = [0, 0, 0.1]
        
        if self.LEG_JOINTS is None:
            self.LEG_JOINTS = {
                'front_left': [0, 1, 2],      # abduction, hip, knee
                'front_right': [4, 5, 6],
                'back_left': [8, 9, 10],
                'back_right': [12, 13, 14]
            }
        
        if self.INITIAL_STANDING_ANGLES is None:
            self.INITIAL_STANDING_ANGLES = {
                'front_left': [0.0, 0.0, 0.5],
                'front_right': [0.0, 0.0, 0.5],
                'back_left': [0.0, 0.0, 0.5],
                'back_right': [0.0, 0.0, 0.5]
            }
        
        if self.STANDING_UP_ANGLES is None:
            self.STANDING_UP_ANGLES = {
                'front_left': [0.0, 0.5, 1.7],      # abduction, hip(後ろ向き), knee(約97度)
                'front_right': [0.0, 0.5, 1.7],
                'back_left': [0.0, 0.5, 1.7],
                'back_right': [0.0, 0.5, 1.7]
            }


# グローバル設定インスタンス
config = Config()
