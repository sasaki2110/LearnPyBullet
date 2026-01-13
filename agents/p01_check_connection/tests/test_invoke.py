"""
invokeのテスト

このテストは、グラフが正常にinvokeできることを確認します。
PyBulletシミュレーションが正常に実行されることを確認します。
"""
import sys
from pathlib import Path

# p01_check_connectionディレクトリをパスに追加
p01_dir = Path(__file__).parent.parent
sys.path.insert(0, str(p01_dir))  # agents/p01_check_connectionを追加

# テスト対象のグラフをインポート
from my_agent.agent import graph


def test_invoke():
    """グラフが正常にinvokeできることを確認するテスト"""
    from langchain.messages import HumanMessage
    
    # テスト用の入力
    initial_state = {
        "messages": [HumanMessage(content="PyBulletシミュレーションを実行してください")],
        "simulation_result": None
    }
    
    print("=" * 60)
    print("PyBulletシミュレーションinvokeテスト開始")
    print("=" * 60)
    print(f"\n初期状態:")
    print(f"  messages: {[msg.content for msg in initial_state['messages']]}")
    print(f"  simulation_result: {initial_state['simulation_result']}")
    print("\n" + "-" * 60)
    
    # invokeを実行
    result = graph.invoke(initial_state)
    
    print("\n実行結果:")
    print("-" * 60)
    print(f"  simulation_result:")
    if result.get('simulation_result'):
        # 結果を複数行に分けて表示
        for line in result['simulation_result'].split('\n'):
            print(f"    {line}")
    else:
        print("    None")
    print(f"\n  messages数: {len(result.get('messages', []))}")
    if result.get('messages'):
        print(f"  最後のメッセージ: {result['messages'][-1].content[:100]}...")
    print("\n" + "=" * 60)
    
    # 結果の検証
    assert "simulation_result" in result, "結果に'simulation_result'が含まれている必要があります"
    assert result["simulation_result"] is not None, "シミュレーション結果が設定されている必要があります"
    assert len(result["simulation_result"]) > 0, "シミュレーション結果が空でない必要があります"
    assert "messages" in result, "結果に'messages'が含まれている必要があります"
    assert len(result["messages"]) > 0, "メッセージが生成されている必要があります"
    
    print("✓ すべての検証が成功しました")
    print("=" * 60)


if __name__ == "__main__":
    # 直接実行時もNoneを返す（pytestの警告を避けるため）
    test_invoke()
