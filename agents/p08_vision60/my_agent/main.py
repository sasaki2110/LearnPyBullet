"""
Vision60（四足ロボット）のメインエントリーポイント

PyBulletを使用したVision60ロボットのシミュレーション制御
"""
import pybullet as p
import time
import math
from .utils.config import config
from .utils.pybullet_env import PyBulletEnvironment
from .utils.robot_model import RobotModel
from .utils.state import Vision60State
from .utils.joint_control import JointController
from .utils.standing_control import StandingController
from .utils.stepping_control import SteppingController
from .utils.posture_control import PostureController
from .utils.reset_handler import ResetHandler
from .utils.stabilization_detector import StabilizationDetector
from .utils.logger import Logger
from .utils.logging_config import setup_logging, get_logger, get_log_level

# ロギングをセットアップ
log_level = get_log_level()
setup_logging(log_level=log_level, initialize=True)
logger = get_logger('main')


def main():
    """メインエントリーポイント"""
    logger.info("🚀 [MAIN] Vision60エージェントの初期化を開始します")
    
    # 環境初期化
    env = PyBulletEnvironment()
    env.create_environment()
    env.load_plane()
    env.load_robot()
    env.setup_camera()
    env.print_joint_info()
    
    # ロボットモデル
    robot_model = RobotModel(env.robot_id)
    
    # 状態管理
    state = Vision60State(env.robot_id)
    
    # 各コントローラー
    joint_controller = JointController(robot_model, state)
    standing_controller = StandingController(robot_model, state)
    stepping_controller = SteppingController(robot_model, state)
    posture_controller = PostureController(robot_model, state)
    reset_handler = ResetHandler(robot_model, state)
    stabilization_detector = StabilizationDetector(robot_model, state)
    log_writer = Logger()
    
    # 初期姿勢を設定
    joint_controller.set_initial_pose()
    
    # 姿勢を安定化
    logger.info("⚙️ 姿勢を安定化中（脚を閉じた状態でそのまま放置）...")
    for _ in range(config.STABILITY_SETTLE_STEPS):
        p.stepSimulation()
        time.sleep(config.TIME_STEP)
    
    # 初期高さを記録
    state.update_initial_height()
    
    # メインループ
    logger.info("⚡ 姿勢を維持・安定化中（PD制御と姿勢フィードバック制御）...")
    for step in range(config.TOTAL_SIMULATION_STEPS):
        # 安定確認
        standing_controller.check_stability(step)
        
        # 立ち上がり制御
        standing_controller.update_standing_angles(step, env.plane_id)
        is_standing_up = standing_controller.is_standing_up()
        
        # 安定化検知
        stabilization_detector.check_stabilization(step)
        
        # 足踏み動作
        stepping_controller.start_stepping(step)
        stepping_controller.update_stepping(step)
        is_stepping = state.stepping_started
        
        # 姿勢フィードバック制御
        posture_controller.apply_posture_feedback(is_standing_up, is_stepping)
        
        # ジョイント制御
        joint_controller.apply_joint_control(is_standing_up)
        
        # リセットチェック
        reset_handler.check_and_reset(step, is_standing_up)
        
        # シミュレーションステップ
        p.stepSimulation()
        time.sleep(config.TIME_STEP)
    
    # 最終状態をログ出力
    log_writer.log_reset_stats(state.reset_count, state.reset_reasons_count, config.TOTAL_SIMULATION_STEPS, logger)
    log_writer.log_final_state(env.robot_id, robot_model, logger)
    
    # クリーンアップ
    logger.info("\n✅ ロボットの登場が完了しました。")
    # 終了待機中であることを示すために色を変更（紫系）
    robot_model.change_robot_color(config.ROBOT_COLOR_FINISHED)
    logger.info("   5秒間停止して終了します...")
    
    # 5秒間、シミュレーションを停止して待機（GUIは表示し続ける）
    for _ in range(500):  # 500回 × 0.01秒 = 5秒
        time.sleep(0.01)
        # シミュレーションは動かさない（p.stepSimulation()を呼ばない）
    
    env.disconnect()
    logger.info("✅ [MAIN] Vision60エージェントの実行が完了しました")


if __name__ == "__main__":
    main()
