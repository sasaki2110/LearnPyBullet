# Vision60 リファクタリングプラン

## 現状分析

### 現在のファイル構造
- **ファイル**: `srcs/p08_vision60.py` (1330行)
- **構造**: 単一ファイルにすべての機能が含まれている
- **問題点**:
  - コードが長すぎて解析が困難
  - 責任が混在している
  - 再利用性が低い
  - テストが困難

### 主要な機能ブロック

1. **初期化・環境設定** (~112行)
   - PyBullet接続
   - 地面とロボットのロード
   - 物理パラメータ設定
   - カメラ設定

2. **設定・定数** (~100行)
   - ジョイント構造定義
   - 姿勢角度設定
   - PD制御パラメータ
   - フィードバックゲイン

3. **立ち上がり制御** (~180行)
   - 安定確認
   - 段階的な角度変更
   - 進行度計算
   - ログ出力

4. **足踏み動作制御** (~140行)
   - フェーズ管理
   - 角度調整
   - バランス制御

5. **姿勢フィードバック制御** (~300行)
   - roll/pitchフィードバック
   - 位置フィードバック
   - 接地状態フィードバック
   - 左右バランス調整

6. **ジョイント制御** (~40行)
   - PD制御
   - ジョイント角度設定

7. **リセット機能** (~40行)
   - 姿勢チェック
   - リセット処理

8. **安定化検知** (~220行)
   - 安定性指標計算
   - 安定化検知ロジック

9. **ログ出力** (各所に分散)
   - 立ち上がり進行度
   - 足踏み動作状態
   - 最終状態表示

## リファクタリング提案

### 重要な設計判断: LangGraphの使用について

既存の`agents/p02_ex_arm2`と`agents/p01_check_connection`を確認した結果、以下の2つの選択肢があります：

#### 選択肢A: LangGraphを使わない（単純なメインエントリーポイント）
- **`agent.py`**: 単純な`main()`関数を持つメインエントリーポイント
- **`test_invoke.py`**: `agent.main()`を直接呼び出す
- **利点**: 
  - シンプルで理解しやすい
  - LangGraphのオーバーヘッドがない
  - 現在のコード構造に近い
- **欠点**: 
  - 既存のagents構造と完全には一致しない

#### 選択肢B: LangGraphを使う（既存構造に完全に合わせる）
- **`agent.py`**: LangGraphの`StateGraph`を作成し、`graph`変数をエクスポート
- **`test_invoke.py`**: `from my_agent.agent import graph`して`graph.invoke()`を呼び出す
- **利点**: 
  - 既存のagents構造と完全に一致
  - 将来的にLLMエージェント機能を追加しやすい
  - LangGraph Studioで可視化・デバッグ可能
- **欠点**: 
  - LangGraphの学習コスト
  - 現在のコード構造から大きく変更が必要

### 推奨: 選択肢A（LangGraphを使わない）

現在のVision60は単純なシミュレーションスクリプトであり、LLMエージェント機能は不要です。
そのため、**選択肢A（LangGraphを使わない）を推奨**します。

ただし、将来的にLLMエージェント機能を追加する可能性がある場合は、選択肢Bを検討してください。

---

### 提案1: agents構造に合わせた分割（選択肢A: LangGraphを使わない）

既存の`agents/p02_ex_arm2`の構造を参考に、以下のように分割：

```
agents/p08_vision60/
├── README.md
├── my_agent/
│   ├── __init__.py
│   ├── main.py                # メインエントリーポイント（将来的にagent.pyを作ることを想定）
│   └── utils/
│       ├── __init__.py
│       ├── config.py          # 設定・定数
│       ├── pybullet_env.py    # PyBullet環境の初期化
│       ├── robot_model.py     # ロボットモデル（ジョイント構造など）
│       ├── standing_control.py    # 立ち上がり制御
│       ├── stepping_control.py    # 足踏み動作制御
│       ├── posture_control.py     # 姿勢フィードバック制御
│       ├── joint_control.py       # ジョイント制御（PD制御）
│       ├── reset_handler.py        # リセット機能
│       ├── stabilization_detector.py  # 安定化検知
│       ├── logger.py               # ログ出力
│       └── state.py                # 状態管理
└── tests/
    ├── __init__.py
    └── test_main.py            # main.pyをテスト（将来的にtest_invoke.pyに変更可能）
```

**注意**: 
- `langgraph.json`は作成しない（LangGraphを使わないため）
- `main.py`を使用（将来的にグラフ化する時に`agent.py`を作成することを想定）
- `test_main.py`を使用（将来的に`test_invoke.py`に変更可能）

### 各ファイルの責任

#### `main.py` (~100行)
- **メインエントリーポイント**: `main()`関数
- メインループ（シミュレーションステップの繰り返し）
- 各モジュールの統合
- シミュレーションの開始・終了
- **注意**: 
  - LangGraphのグラフは作成しない（単純な関数ベース）
  - 将来的にグラフ化する時に`agent.py`を作成することを想定

#### `utils/config.py` (~100行)
- すべての設定値・定数
- PD制御パラメータ
- フィードバックゲイン
- タイミングパラメータ（ステップ数など）

#### `utils/pybullet_env.py` (~150行)
- PyBullet接続
- 地面とロボットのロード
- 物理パラメータ設定
- カメラ設定

#### `utils/robot_model.py` (~80行)
- ジョイント構造定義
- リンク名マッピング
- ロボット固有の情報

#### `utils/standing_control.py` (~200行)
- 立ち上がり制御ロジック
- 安定確認
- 段階的な角度変更
- 進行度計算

#### `utils/stepping_control.py` (~180行)
- 足踏み動作制御
- フェーズ管理
- 角度調整
- バランス制御

#### `utils/posture_control.py` (~250行)
- 姿勢フィードバック制御
- roll/pitchフィードバック
- 位置フィードバック
- 接地状態フィードバック
- 左右バランス調整

#### `utils/joint_control.py` (~80行)
- PD制御
- ジョイント角度設定
- トルク制御

#### `utils/reset_handler.py` (~80行)
- リセット判定
- リセット処理
- リセット統計

#### `utils/stabilization_detector.py` (~250行)
- 安定性指標計算
- 安定化検知ロジック
- 安定化時の詳細ログ

#### `utils/logger.py` (~150行)
- ログ出力の統一
- 立ち上がり進行度ログ
- 足踏み動作ログ
- 最終状態表示

#### `utils/state.py` (~100行)
- 状態管理クラス
- 状態の更新・取得
- 前回値の保持

### 提案2: クラスベースの設計（代替案）

各機能をクラスとして実装：

```python
class Vision60Robot:
    """ロボットの基本操作"""
    
class StandingController:
    """立ち上がり制御"""
    
class SteppingController:
    """足踏み動作制御"""
    
class PostureController:
    """姿勢フィードバック制御"""
    
class StabilizationDetector:
    """安定化検知"""
```

## 推奨アプローチ

**提案1（モジュール分割）を推奨**します。理由：

1. **既存構造との一貫性**: `agents/p02_ex_arm2`と同じ構造で統一
2. **AI解析の容易さ**: 各ファイルが単一責任で、解析しやすい
3. **テスト容易性**: 各モジュールを個別にテスト可能
4. **保守性**: 変更が局所化される
5. **再利用性**: 各モジュールを他のプロジェクトで再利用可能

## 実装手順

1. **ディレクトリ構造の作成**
   - `agents/p08_vision60/` ディレクトリを作成
   - 各サブディレクトリとファイルを作成

2. **設定・定数の抽出** (`config.py`)
   - すべての定数値を抽出
   - 設定クラスまたは辞書として定義

3. **環境初期化の分離** (`pybullet_env.py`)
   - PyBullet接続
   - ロボットロード
   - 物理パラメータ設定

4. **ロボットモデルの分離** (`robot_model.py`)
   - ジョイント構造
   - リンク名マッピング

5. **状態管理の実装** (`state.py`)
   - 状態クラスの定義
   - 状態の更新・取得メソッド

6. **各制御モジュールの実装**
   - `standing_control.py`
   - `stepping_control.py`
   - `posture_control.py`
   - `joint_control.py`

7. **リセット・検知モジュールの実装**
   - `reset_handler.py`
   - `stabilization_detector.py`

8. **ログ出力の統一** (`logger.py`)
   - すべてのログ出力を統一

9. **メインエントリーポイントの実装** (`main.py`)
   - 各モジュールを統合
   - メインループの実装
   - **注意**: 将来的にグラフ化する時に`agent.py`を作成することを想定

10. **テストの実装** (`tests/test_main.py`)
   - `main.main()`を直接呼び出す
   - 基本的な動作確認
   - **注意**: 
     - LangGraphの`graph.invoke()`ではなく、直接関数呼び出し
     - 将来的に`test_invoke.py`に変更可能

## 期待される効果

1. **コード行数の削減**: 各ファイルが100-250行程度に
2. **可読性の向上**: 各ファイルの責任が明確
3. **AI解析の容易さ**: 小さなファイル単位で解析可能
4. **保守性の向上**: 変更が局所化される
5. **テスト容易性**: 各モジュールを個別にテスト可能
6. **再利用性**: 各モジュールを他のプロジェクトで再利用可能

## 注意事項

1. **インポートの整理**: 循環参照を避ける
2. **状態の共有**: `state.py`で状態を一元管理
3. **後方互換性**: 既存の`srcs/p08_vision60.py`は残す（移行期間）
4. **ドキュメント**: 各モジュールにdocstringを追加
5. **LangGraphの使用**: 今回は使用しない（選択肢A）。将来的にLLMエージェント機能を追加する場合は選択肢Bを検討
6. **ファイル名**: `main.py`を使用（将来的にグラフ化する時に`agent.py`を作成することを想定）
7. **テストファイル名**: `test_main.py`を使用（将来的に`test_invoke.py`に変更可能）

## 選択肢Bの実装例（参考）

もしLangGraphを使う場合（選択肢B）の実装例：

### `agent.py` (LangGraph版)
```python
from langgraph.graph import StateGraph, START, END
from .utils.state import Vision60State
from .utils.nodes import (
    initialize_node,
    standing_control_node,
    stepping_control_node,
    posture_control_node,
    joint_control_node,
    reset_handler_node,
    stabilization_detector_node
)

graph = StateGraph(Vision60State)
graph.add_node("initialize", initialize_node)
graph.add_node("standing_control", standing_control_node)
graph.add_node("stepping_control", stepping_control_node)
graph.add_node("posture_control", posture_control_node)
graph.add_node("joint_control", joint_control_node)
graph.add_node("reset_handler", reset_handler_node)
graph.add_node("stabilization_detector", stabilization_detector_node)

graph.add_edge(START, "initialize")
graph.add_edge("initialize", "standing_control")
# ... エッジの追加

graph = graph.compile()
```

### `main.py` (現在の実装例)
```python
from .utils.pybullet_env import PyBulletEnvironment
from .utils.robot_model import RobotModel
from .utils.state import Vision60State
from .utils.standing_control import StandingController
from .utils.stepping_control import SteppingController
from .utils.posture_control import PostureController
from .utils.joint_control import JointController
from .utils.reset_handler import ResetHandler
from .utils.stabilization_detector import StabilizationDetector
from .utils.logger import Logger
from .utils.config import Config

def main():
    """メインエントリーポイント"""
    # 環境初期化
    env = PyBulletEnvironment()
    env.create_environment()
    
    # ロボットモデル
    robot_model = RobotModel(env.robot_id)
    
    # 状態管理
    state = Vision60State()
    
    # 各コントローラー
    standing_controller = StandingController(robot_model, state, Config)
    stepping_controller = SteppingController(robot_model, state, Config)
    posture_controller = PostureController(robot_model, state, Config)
    joint_controller = JointController(robot_model, state, Config)
    reset_handler = ResetHandler(robot_model, state, Config)
    stabilization_detector = StabilizationDetector(robot_model, state, Config)
    logger = Logger(Config)
    
    # メインループ
    for step in range(Config.TOTAL_SIMULATION_STEPS):
        # 各制御モジュールを実行
        # ...
    
    # クリーンアップ
    env.disconnect()

if __name__ == "__main__":
    main()
```

### `test_main.py` (現在の実装例)
```python
from my_agent.main import main

def test_main():
    """main()関数が正常に実行できることを確認"""
    main()
    # 基本的な動作確認
```

### 将来的な`agent.py` (LangGraph版、参考)
```python
# 将来的にグラフ化する時に作成
from langgraph.graph import StateGraph, START, END
from .utils.state import Vision60State
from .utils.nodes import (
    initialize_node,
    standing_control_node,
    stepping_control_node,
    # ...
)

graph = StateGraph(Vision60State)
# ... グラフの構築
graph = graph.compile()
```

### 将来的な`test_invoke.py` (LangGraph版、参考)
```python
# 将来的にグラフ化する時に作成
from my_agent.agent import graph

def test_invoke():
    initial_state = {
        "step": 0,
        "robot_id": None,
        # ... 初期状態
    }
    result = graph.invoke(initial_state)
    # ... 検証
```
