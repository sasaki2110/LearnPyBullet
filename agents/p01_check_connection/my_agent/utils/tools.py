"""
PyBulletツール関数

LangGraphからPyBulletを呼び出すためのツール関数
"""
import pybullet as p
import pybullet_data
import time
from .logging_config import get_logger

logger = get_logger('tools')


def run_pybullet_simulation(steps: int = 50) -> str:
    """
    PyBulletシミュレーションを実行するツール関数（DIRECTモード専用）
    
    注意: この関数はDIRECTモード（GUIなし）でのみ動作します。
    LangGraph経由ではGUIモードは使用できません。
    
    Args:
        steps: シミュレーションステップ数
    
    Returns:
        シミュレーション結果の文字列
    """
    logger.info(f"🚀 [TOOLS] PyBulletシミュレーションを開始します (mode=DIRECT, steps={steps})")
    
    try:
        # DIRECTモードで接続（LangGraph経由ではGUIモードは使用不可）
        client_id = p.connect(p.DIRECT)
        if client_id < 0:
            error_msg = "物理サーバーへの接続に失敗しました。"
            logger.error(f"❌ [TOOLS] {error_msg}")
            return error_msg
        
        logger.info(f"✅ [TOOLS] PyBullet接続成功 (Client ID: {client_id})")
        
        # PyBulletに付属している標準データのパスを追加
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # 重力を地球（-9.81）に設定
        p.setGravity(0, 0, -9.81)
        
        # 床(plane)をロード
        plane_id = p.loadURDF("plane.urdf")
        logger.debug(f"📦 [TOOLS] 床をロードしました (ID: {plane_id})")
        
        # ロボット(R2D2)を空中(高さ1m)にロード
        robot_id = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 1.0])
        logger.debug(f"🤖 [TOOLS] R2D2ロボットをロードしました (ID: {robot_id})")
        
        # シミュレーション結果を記録
        results = []
        results.append(f"接続成功！ Client ID: {client_id}")
        results.append(f"標準データパス: {pybullet_data.getDataPath()}")
        
        # 物理シミュレーションの進展
        for i in range(steps):
            p.stepSimulation()
            if i % 10 == 0:
                pos, _ = p.getBasePositionAndOrientation(robot_id)
                height = pos[2]
                results.append(f"ステップ {i}: ロボットの現在の高さ = {height:.4f}m")
                logger.debug(f"📊 [TOOLS] ステップ {i}: 高さ = {height:.4f}m")
        
        # 最終位置を取得
        final_pos, _ = p.getBasePositionAndOrientation(robot_id)
        final_height = final_pos[2]
        results.append(f"最終ステップ: ロボットの最終高さ = {final_height:.4f}m")
        
        # 切断
        p.disconnect()
        logger.info("✅ [TOOLS] シミュレーションを正常に終了しました")
        
        result_str = "\n".join(results)
        return result_str
        
    except Exception as e:
        error_msg = f"❌ [TOOLS] シミュレーション実行中にエラーが発生しました: {e}"
        logger.error(error_msg, exc_info=True)
        return error_msg
