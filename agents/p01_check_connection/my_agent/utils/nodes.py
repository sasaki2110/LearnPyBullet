"""
ノード関数の実装
"""
from langchain.messages import HumanMessage, AIMessage
from .state import State
from .tools import run_pybullet_simulation
from .logging_config import get_logger

# ロガーを取得
logger = get_logger('nodes')


def run_simulation_node(state: State):
    """PyBulletシミュレーションを実行するノード（DIRECTモード専用）"""
    logger.info("🎮 [NODES] シミュレーションノードを開始します")
    
    try:
        # DIRECTモード専用（LangGraph経由ではGUIモードは使用不可）
        steps = 50
        
        logger.info(f"📝 [NODES] シミュレーション設定: mode=DIRECT, steps={steps}")
        
        # PyBulletシミュレーションを実行（DIRECTモード）
        result = run_pybullet_simulation(steps=steps)
        
        logger.info("✅ [NODES] シミュレーションが完了しました")
        logger.debug(f"📊 [NODES] 結果: {result[:100]}...")
        
        # 結果をAIMessageとして追加
        response_message = AIMessage(content=f"PyBulletシミュレーションを実行しました。\n\n結果:\n{result}")
        
        return {
            "simulation_result": result,
            "messages": [response_message]
        }
        
    except Exception as e:
        error_msg = f"❌ [NODES] シミュレーションノードでエラーが発生しました: {e}"
        logger.error(error_msg, exc_info=True)
        
        error_message = AIMessage(content=f"エラーが発生しました: {e}")
        return {
            "simulation_result": None,
            "messages": [error_message]
        }
