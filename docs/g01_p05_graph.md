すばらしいです！ログの `actual_pos` を見ると、最初は高さ **0.82m** 付近にあった手が、最終的に目標の **0.2m** （x=0.5）へ誤差数ミリのレベルで到達していますね。

これで、あなたは **「仮想空間に物体を配置し、ロボットに物理計算（逆運動学）を行わせ、目標地点へ動かす」** という一連のプロセスをマスターしました。

---

## ステップ 6：LangGraph との統合に向けた「ツール化」

いよいよ最後の仕上げです。今のスクリプトを、AIエージェント（LangGraph）が「道具」として使えるような関数（Tool）の形に整理しましょう。

LangGraphから呼び出す際に重要なのは、**「1回の関数呼び出しでシミュレーションを必要な分だけ進め、結果を返す」** という設計です。

### 実践：`arm_tool.py`（AI用ツール・プロトタイプ）

このコードは、これまでの学習の集大成です。AIが「トレイの場所へ移動して」と言ったときに実行される内部ロジックを想定しています。

```python
import pybullet as p
import pybullet_data
import time

class RobotArmSimulator:
    def __init__(self, mode=p.DIRECT):
        # 接続設定 (AIの思考中は DIRECT、デバッグ時は GUI)
        self.client = p.connect(mode)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # 環境のセットアップ
        p.loadURDF("plane.urdf")
        self.robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
        self.end_effector_index = 11

    def move_to_position(self, target_xyz):
        """AIから座標(x,y,z)を受け取って腕を動かし、最終的な座標を返す"""
        
        # 240ステップ（約1秒分）シミュレーションを回して安定させる
        for _ in range(240):
            joint_poses = p.calculateInverseKinematics(
                self.robot_id, 
                self.end_effector_index, 
                target_xyz
            )
            for j in range(len(joint_poses)):
                p.setJointMotorControl2(
                    self.robot_id, j, p.POSITION_CONTROL, joint_poses[j]
                )
            p.stepSimulation()
        
        # 到達後の座標を取得
        ls = p.getLinkState(self.robot_id, self.end_effector_index)
        return ls[4]

    def close(self):
        p.disconnect()

# --- AIがツールを叩くシミュレーション ---
def tool_test():
    # 実際にはここで LangGraph が「トレイは 0.5, 0, 0 だな」と判断して送ってくる
    sim = RobotArmSimulator(mode=p.GUI) # 動きを見たいのでGUI
    
    target = [0.5, 0.2, 0.3]
    print(f"AIの指示: {target} へ移動せよ")
    
    final_pos = sim.move_to_position(target)
    print(f"ツール実行結果: 移動完了。現在地は {final_pos} です。")
    
    time.sleep(2)
    sim.close()

if __name__ == "__main__":
    tool_test()

```

---

## 物理エンジンを学んだ後の「次のステップ」

これで PyBullet の基本操作（導入〜可視化〜制御）が完了しました。
LangGraph などの AI エージェント開発に戻る際、以下の **「3つの進化」** が可能になります：

1. **座標のリアリティ**: AIが「トレイは  にある」と嘘をついても、物理エンジン側で「そこには床しかありません」とエラーを返したり、物理的に届かない（IKが解けない）場合に「手が届きません」とAIにフィードバックできます。
2. **干渉の検知**: `p.getContactPoints()` を使えば、アームがトレイにぶつかってしまったかどうかをAIに教えることができます。
3. **状態の巻き戻し**: `p.saveState()` を使えば、AIが失敗したときに「1秒前の状態に戻してやり直す」という強化学習のような動きも可能です。

**次はどうされますか？**

* 「この PyBullet ロジックを、実際に LangGraph の `tools.py` に組み込む具体的な方法」を考えますか？
* それとも、PyBullet 側で「物を掴む（グリッパー）」操作について深掘りしますか？

PyBullet の物理演算ロジックを LangGraph の Tool として統合する具体的な方法を解説します。

LangGraph でロボット操作を扱う際のポイントは、**「シミュレーションの状態（インスタンス）をどのように維持するか」**という点です。AI が「トレイに移動して」の後に「次は掴んで」と連続して命令する場合、シミュレーションを毎回リセットせず、同じ状態を保持する必要があります。

---

## 1. ディレクトリ構成と依存関係

プロジェクトフォルダ（`~/LearnPyBullet`）内に以下のファイルを用意します。

```text
LearnPyBullet/
├── requirements.txt
├── srcs/
│   ├── robot_sim.py   # PyBulletのロジックをカプセル化したクラス
│   └── tools.py       # LangChain/LangGraph用のTool定義
└── main_graph.py      # LangGraphのメインロジック

```

`requirements.txt` には以下を追加してください。

```text
langgraph
langchain-openai  # または使用するモデルのライブラリ

```

---

## 2. PyBullet ロジックのカプセル化 (`srcs/robot_sim.py`)

AI が呼び出しやすいよう、前回の `arm_tool.py` をさらに整理します。

```python
import pybullet as p
import pybullet_data
from typing import List

class PandaSim:
    def __init__(self, mode=p.DIRECT):
        self.client = p.connect(mode)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")
        self.robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
        self.tray_id = p.loadURDF("tray/tray.urdf", basePosition=[0.5, 0.0, 0.01])
        self.ee_index = 11

    def move_arm(self, x: float, y: float, z: float) -> str:
        """指定した座標にアームを移動させ、結果を文字列で返す"""
        target = [x, y, z]
        # 240ステップ（約1秒分）実行
        for _ in range(240):
            joint_poses = p.calculateInverseKinematics(self.robot_id, self.ee_index, target)
            for j in range(len(joint_poses)):
                p.setJointMotorControl2(self.robot_id, j, p.POSITION_CONTROL, joint_poses[j])
            p.stepSimulation()
        
        pos = p.getLinkState(self.robot_id, self.ee_index)[4]
        return f"移動完了。現在の手先位置は x:{pos[0]:.2f}, y:{pos[1]:.2f}, z:{pos[2]:.2f} です。"

    def get_object_pos(self) -> str:
        """トレイの現在位置を取得する"""
        pos, _ = p.getBasePositionAndOrientation(self.tray_id)
        return f"トレイは座標 x:{pos[0]:.2f}, y:{pos[1]:.2f}, z:{pos[2]:.2f} にあります。"

```

---

## 3. LangGraph 用の Tool 定義 (`srcs/tools.py`)

LangChain の `@tool` デコレータを使用して、AI が理解できる形式にします。ここでは**シングルトン・パターン**（またはグローバル変数）を使用して、シミュレーションの状態を維持します。

```python
from langchain_core.tools import tool
from .robot_sim import PandaSim
import pybullet as p

# シミュレーションのインスタンスを保持（DIRECTモード推奨）
sim = PandaSim(mode=p.DIRECT)

@tool
def get_tray_position():
    """シミュレーション空間内のトレイの現在位置を取得します。"""
    return sim.get_object_pos()

@tool
def move_robot_arm(x: float, y: float, z: float):
    """
    ロボットアームを指定された x, y, z 座標に移動させます。
    トレイを触る場合は、トレイの座標の少し上を指定してください。
    """
    return sim.move_arm(x, y, z)

# Toolのリスト
robot_tools = [get_tray_position, move_robot_arm]

```

---

## 4. LangGraph への組み込み (`main_graph.py`)

最後に、これらを LangGraph のグラフとして組み立てます。

```python
from typing import Annotated, TypedDict
from langgraph.graph import StateGraph, START, END
from langgraph.graph.message import add_messages
from langchain_openai import ChatOpenAI # 使用するモデルに変更してください
from .srcs.tools import robot_tools

# 1. 状態の定義
class State(TypedDict):
    messages: Annotated[list, add_messages]

# 2. モデルのセットアップ
model = ChatOpenAI(model="gpt-4o").bind_tools(robot_tools)

# 3. ノード関数の定義
def chatbot(state: State):
    return {"messages": [model.invoke(state["messages"])]}

# 4. グラフの構築
from langgraph.prebuilt import ToolNode, tools_condition

builder = StateGraph(State)
builder.add_node("chatbot", chatbot)
builder.add_node("tools", ToolNode(robot_tools))

builder.add_edge(START, "chatbot")
builder.add_conditional_edges("chatbot", tools_condition)
builder.add_edge("tools", "chatbot")

graph = builder.compile()

# 5. 実行
# config = {"configurable": {"thread_id": "1"}}
# graph.invoke({"messages": [("user", "トレイの場所を調べて、その20cm上に移動して")]})

```

---

## 学習内容のまとめとメリット

* **状態の永続化**: `sim` インスタンスを Tool の外で保持することで、AI が「トレイを確認（Tool A）」→「移動（Tool B）」と連続して実行しても、シミュレーション内の時間は連続して流れます。
* **物理的なフィードバック**: AI がもし不可能な座標（例えば地面より下）を指定しても、PyBullet の `calculateInverseKinematics` の結果を Tool が返すことで、AI は「その指示は物理的に不可能だった」と自覚し、推論を修正できます。

**次は何をしましょうか？**

* この構成を実際に動かせるよう、`main_graph.py` の詳細な実行コードや API キー設定を含めた完成版を作成しますか？
* それとも、PyBullet の `getContactPoints` を使って、「トレイにぶつかったら AI に警告を出す」ような、より「物理らしい」フィードバック機能を追加しますか？