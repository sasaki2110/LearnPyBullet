"""
LangGraphからPyBulletを呼び出すテストエージェント

このグラフは、PyBulletシミュレーションを実行するシンプルなエージェントです。
LangGraph Studio（`langgraph dev`）で使用できます。
"""
import os
try:
    from dotenv import load_dotenv
    load_dotenv()
except ImportError:
    pass  # dotenvがインストールされていない場合はスキップ
from langgraph.graph import StateGraph, START, END
from .utils.state import State
from .utils.nodes import run_simulation_node
from .utils.logging_config import setup_logging, get_logger, get_log_level

# ロギングをセットアップ
log_level = get_log_level()
setup_logging(log_level=log_level, initialize=True)
logger = get_logger('agent')

logger.info("🚀 [AGENT] エージェントの初期化を開始します")

try:
    # グラフの構築
    logger.debug("📊 [AGENT] グラフの構築を開始します")
    graph = StateGraph(State)
    
    # ノードの追加
    graph.add_node("run_simulation", run_simulation_node)
    logger.info("✅ [AGENT] ノードの追加が完了しました (run_simulation)")
    
    # エッジの追加
    graph.add_edge(START, "run_simulation")
    graph.add_edge("run_simulation", END)
    logger.info("✅ [AGENT] エッジの追加が完了しました (START -> run_simulation -> END)")
    
    # コンパイルしてモジュールレベルの変数に代入
    # langgraph.jsonでは "./my_agent/agent.py:graph" として参照可能
    logger.debug("🔨 [AGENT] グラフをコンパイルしています...")
    graph = graph.compile()
    logger.info("✅ [AGENT] エージェントの初期化が完了しました")
    
except Exception as e:
    logger.error(f"❌ [AGENT] エージェントの初期化中にエラーが発生しました: {e}", exc_info=True)
    raise
