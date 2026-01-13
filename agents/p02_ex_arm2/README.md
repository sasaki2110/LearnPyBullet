# p02_ex_arm2 - PyBullet版ロボットアーム制御エージェント

このプロジェクトは、PyBulletを使用して物理シミュレーションを行いながら、ユーザーの指示に基づいてロボットアームを制御するLangGraphエージェントです。
p34_ex_arm1をベースに、PyBulletによる物理演算を統合しました。

## 主な特徴

- **PyBullet物理シミュレーション**: 実際の物理演算を使用したロボットアーム制御
- **Franka Pandaアーム**: 実用的な7自由度ロボットアームを使用
- **DIRECTモード専用**: LangGraph経由での実行に最適化（GUIなし）
- **物体位置の自動取得**: PyBullet環境内の物体から実際の位置を取得
- **逆運動学による制御**: PyBulletの逆運動学を使用した正確なアーム制御

## 構造

```
p02_ex_arm2/
├── my_agent/              # プロジェクトコード
│   ├── utils/             # グラフ用のユーティリティ
│   │   ├── __init__.py
│   │   ├── state.py       # グラフの状態定義
│   │   ├── nodes.py       # グラフ用のノード関数（ログ付き）
│   │   ├── tools.py       # PyBullet版ロボットアーム用のツール関数
│   │   ├── pybullet_env.py # PyBullet環境管理モジュール
│   │   └── logging_config.py  # ロギング設定
│   ├── __init__.py
│   └── agent.py          # グラフを構築するコード（ログ付き）
├── tests/                # テストファイル
│   ├── __init__.py
│   └── test_invoke.py    # invokeのテスト
├── langgraph.json         # LangGraph設定ファイル
└── README.md             # このファイル
```

## 機能

このエージェントは、以下の処理を行います：

1. **Extractor（抽出）**: ユーザーメッセージから指示を取得します
2. **Planner（計画）**: LLMを使用して、全体のタスクリストを生成します（構造化出力を使用）
3. **Task Selector（タスク選択）**: 依存関係が満たされた次のタスクを選択します
4. **Tool Executor（ツール実行）**: 選択されたタスクを実行します
   - `get_object_position(item_name)`: PyBullet環境内から物体の位置を取得
   - `move_arm_to(x, y, z)`: PyBulletの逆運動学を使用してアームを指定座標に移動
   - `control_gripper(action)`: グリッパーを開閉（暫定：状態のみ更新）
5. **Task Updater（タスク更新）**: 実行完了したタスクを完了済みにマークします
6. **Verifier（検証）**: すべてのタスクが完了したかどうかを確認します
7. **Final Answer（最終回答）**: タスクが完了した場合、LLMを使用して最終結果を整形して返却します

## PyBullet環境管理

- **毎回新しい環境**: ツール実行時に毎回新しいPyBullet環境を作成
- **自動クリーンアップ**: ツール実行後に環境を自動的にクリーンアップ
- **DIRECTモード**: GUIなしで物理シミュレーションを実行

## ツール

| 関数名 | 引数 | 説明 |
| --- | --- | --- |
| `get_object_position(item_name)` | `item_name` (str) | PyBullet環境内から指定した物体の3次元座標を取得 |
| `move_arm_to(x, y, z)` | `x, y, z` (float) | PyBulletの逆運動学を使用してアームの先端（グリッパー）を指定座標へ移動 |
| `control_gripper(action)` | `action` ("open" or "close") | グリッパーの開閉を行う（暫定：状態のみ更新） |

## グラフ構造

```
start -> extractor -> planner -> task_selector -> tool_executor -> task_updater -> verifier
                                                                                    ↓
                                                                               条件分岐
                                                                                ↓      ↓
                                                                          final_answer  task_selector
                                                                                ↓
                                                                              end
```

## セットアップ

### 1. 依存関係のインストール

親フォルダで実行済みの場合、それらを使用できます。
必要な依存関係：
- langchain
- langchain-openai
- langgraph
- pybullet
- python-dotenv (オプション)

### 2. 環境変数の設定

親フォルダで実行済み。

### 3. LangGraph Studioで実行

```bash
cd /root/LearnPyBullet/agents/p02_ex_arm2
langgraph dev
```

## テスト

### invokeのテスト

```bash
cd /root/LearnPyBullet/agents/p02_ex_arm2
source ../../venv/bin/activate
pytest tests/test_invoke.py -v
```

## 使用例

```python
from my_agent.agent import graph
from langchain.messages import HumanMessage

initial_state = {
    "messages": [HumanMessage(content="赤いコップの位置を取得して、その位置にアームを移動してください")],
    "gripper_position_x": 0.0,
    "gripper_position_y": 0.0,
    "gripper_position_z": 0.0,
    "gripper_state": "open",
    "instruction": None,
    "task_completed": False
}

result = graph.invoke(initial_state)
print(f"タスク完了: {result['task_completed']}")
```

## p34_ex_arm1との違い

- **物理シミュレーション**: PyBulletを使用した実際の物理演算
- **物体位置取得**: 仮想データではなく、PyBullet環境内の物体から位置を取得
- **アーム制御**: 逆運動学を使用した正確な制御
- **環境管理**: 毎回新しい環境を作成・クリーンアップ

## 注意事項

- **DIRECTモード専用**: GUIモードは使用できません
- **グリッパー制御**: 現在は状態のみ更新（実際のグリッパー制御は未実装）
- **環境の独立性**: 各ツール実行時に新しい環境を作成するため、前回の状態は保持されません
