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
    CAMERA_YAW: float = 0.0
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
    LEG_LIFT_HEIGHT: float = 0.05  # 足を上げる高さ（m、未使用）
    
    # ロボットの色設定（RGBA、各値は0.0～1.0）
    # アルファ値は0.5（半透明）に設定（change_robot_colorメソッドで自動的に適用される）
    ROBOT_COLOR_NORMAL: List[float] = field(default_factory=lambda: [0.3, 0.5, 0.8, 1.0])  # 通常時: 青系
    ROBOT_COLOR_STEPPING: List[float] = field(default_factory=lambda: [0.2, 0.8, 0.3, 1.0])  # 足踏み中: 緑系
    ROBOT_COLOR_STANDING_UP: List[float] = field(default_factory=lambda: [0.9, 0.6, 0.2, 1.0])  # 立ち上がり中: オレンジ系
    ROBOT_COLOR_FINISHED: List[float] = field(default_factory=lambda: [0.8, 0.2, 0.8, 1.0])  # 終了待機中: 紫系
    
    # シミュレーション設定
    # 足踏み開始（約ステップ2220）から0.5秒×4フェーズ（2秒/サイクル）を複数サイクル実行
    # 計算: 2220（足踏み開始） + 240×4×5（5サイクル×2秒） = 7020ステップ → 12000ステップ（余裕を持たせる）
    TOTAL_SIMULATION_STEPS: int = 12000  # 総シミュレーションステップ数
    
    # ログ出力設定
    STANDING_UP_LOG_INTERVAL: int = 100  # 立ち上がり進行度ログの間隔（ステップ数）
    STEPPING_LOG_INTERVAL: int = 100  # 足踏み動作ログの間隔（ステップ数）
    
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
