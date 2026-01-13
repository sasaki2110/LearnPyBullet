"""
PyBullet版ロボットアーム用のツール関数

PyBullet環境を使用して実際の物理シミュレーションを実行
"""
from typing import Dict, Tuple, Optional
from .logging_config import get_logger
from .pybullet_env import get_environment, PyBulletEnvironment

logger = get_logger('tools')

# 物体名とURDFパスのマッピング
# 日本語名で指定された物体をPyBullet環境内でロードする
OBJECT_URDF_MAPPING: Dict[str, str] = {
    "赤いコップ": "tray/tray.urdf",  # 暫定：トレイをコップとして使用
    "青いトレイ": "tray/tray.urdf",
    "トレイ": "tray/tray.urdf",
    "コップ": "tray/tray.urdf",
    # 英語名もサポート
    "red cup": "tray/tray.urdf",
    "blue tray": "tray/tray.urdf",
    "tray": "tray/tray.urdf",
    "cup": "tray/tray.urdf",
}

# 物体のデフォルト位置（物体名 -> 位置）
OBJECT_DEFAULT_POSITIONS: Dict[str, Tuple[float, float, float]] = {
    "赤いコップ": (0.5, 0.0, 0.01),
    "青いトレイ": (0.3, 0.2, 0.01),
    "トレイ": (0.5, 0.0, 0.01),
    "コップ": (0.5, 0.0, 0.01),
    "red cup": (0.5, 0.0, 0.01),
    "blue tray": (0.3, 0.2, 0.01),
    "tray": (0.5, 0.0, 0.01),
    "cup": (0.5, 0.0, 0.01),
}


def get_object_position(item_name: str) -> Tuple[float, float, float]:
    """
    指定した物体の3次元座標を取得（PyBullet環境内から）
    
    Args:
        item_name: 物体の名前（日本語または英語）
        
    Returns:
        物体の位置 (x, y, z)
    """
    logger.info(f"🔍 [TOOL] 物体の位置を取得: {item_name}")
    
    # 新しい環境を作成
    env = get_environment()
    
    try:
        # 物体が既にロードされているか確認
        position = env.get_object_position(item_name)
        
        if position is None:
            # 物体がロードされていない場合、ロードを試みる
            if item_name in OBJECT_URDF_MAPPING:
                urdf_path = OBJECT_URDF_MAPPING[item_name]
                default_pos = OBJECT_DEFAULT_POSITIONS.get(item_name, (0.5, 0.0, 0.01))
                env.load_object(item_name, urdf_path, default_pos)
                position = env.get_object_position(item_name)
                
                if position is None:
                    logger.warning(f"⚠️ [TOOL] 物体 '{item_name}' の位置取得に失敗しました。デフォルト位置を返します。")
                    return OBJECT_DEFAULT_POSITIONS.get(item_name, (0.0, 0.0, 0.0))
            else:
                logger.warning(f"⚠️ [TOOL] 物体 '{item_name}' のURDFマッピングが見つかりません。デフォルト位置を返します。")
                return OBJECT_DEFAULT_POSITIONS.get(item_name, (0.0, 0.0, 0.0))
        
        logger.info(f"✅ [TOOL] 物体 '{item_name}' の位置: ({position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f})")
        return position
        
    except Exception as e:
        logger.error(f"❌ [TOOL] 物体位置取得中にエラーが発生しました: {e}", exc_info=True)
        # エラー時はデフォルト位置を返す
        return OBJECT_DEFAULT_POSITIONS.get(item_name, (0.0, 0.0, 0.0))
    finally:
        # 環境をクリーンアップ（毎回新しい環境を作成するため）
        env.cleanup()


def move_arm_to(x: float, y: float, z: float) -> bool:
    """
    アームの先端（グリッパー）を指定座標へ移動させる（PyBullet逆運動学を使用）
    
    Args:
        x: X座標
        y: Y座標
        z: Z座標
        
    Returns:
        移動が成功したかどうか
    """
    logger.info(f"🤖 [TOOL] アームを移動: ({x:.4f}, {y:.4f}, {z:.4f})")
    
    # 新しい環境を作成
    env = get_environment()
    
    try:
        success = env.move_arm_to((x, y, z), steps=100)
        
        if success:
            # 実際の位置を取得してログに記録
            actual_pos = env.get_arm_position()
            if actual_pos:
                logger.info(f"✅ [TOOL] アームの移動が完了しました: 実際の位置=({actual_pos[0]:.4f}, {actual_pos[1]:.4f}, {actual_pos[2]:.4f})")
            else:
                logger.info(f"✅ [TOOL] アームの移動が完了しました")
        else:
            logger.warning(f"⚠️ [TOOL] アームの移動に失敗しました")
        
        return success
        
    except Exception as e:
        logger.error(f"❌ [TOOL] アーム移動中にエラーが発生しました: {e}", exc_info=True)
        return False
    finally:
        # 環境をクリーンアップ
        env.cleanup()


def control_gripper(action: str) -> bool:
    """
    グリッパーの開閉を行う（暫定：状態のみ更新）
    
    Args:
        action: "open" または "close"
        
    Returns:
        操作が成功したかどうか
    """
    logger.info(f"🤖 [TOOL] グリッパーを操作: {action}")
    
    if action not in ["open", "close"]:
        logger.error(f"❌ [TOOL] 無効なアクション: {action} (open または close である必要があります)")
        return False
    
    # 暫定実装：状態のみ更新（実際のグリッパー制御は未実装）
    logger.info(f"✅ [TOOL] グリッパーの操作が完了しました: {action} (状態のみ更新)")
    return True
