# p01_check_connection

LangGraphからPyBulletを呼び出すテストエージェント

## 概要

このエージェントは、LangGraphからPyBulletの物理シミュレーションを呼び出すことができることを確認するためのシンプルな実装です。

## 構造

```
p01_check_connection/
├── langgraph.json          # LangGraph設定ファイル
├── my_agent/
│   ├── __init__.py
│   ├── agent.py            # グラフ定義（START -> run_simulation -> END）
│   └── utils/
│       ├── __init__.py
│       ├── state.py        # State定義
│       ├── nodes.py        # ノード関数（run_simulation_node）
│       ├── tools.py         # PyBulletツール関数（run_pybullet_simulation）
│       └── logging_config.py # ロギング設定
└── tests/
    ├── __init__.py
    └── test_invoke.py      # テストファイル
```

## グラフ構造

```
START -> run_simulation -> END
```

- **run_simulation**: PyBulletシミュレーションを実行するノード
  - R2D2ロボットを高さ1mから落下させる
  - **DIRECTモード専用**（GUIなし、物理シミュレーションのみ）

## 使用方法

### テスト実行

```bash
cd agents/p01_check_connection
source ../../venv/bin/activate
pytest tests/test_invoke.py -v -s
```

### LangGraph Studioで実行

```bash
cd agents/p01_check_connection
langgraph dev
```

## 接続モード

**⚠️ このエージェントはDIRECTモード専用です**

- **接続モード**: `p.DIRECT`（GUIなし、物理シミュレーションのみ）
- **シミュレーションステップ数**: 50ステップ
- **ロボット**: R2D2（高さ1mから落下）

### なぜDIRECTモード専用か

LangGraph経由ではGUIモード（`p.GUI`）は使用できません。理由：
1. LangGraphの実行環境ではDISPLAY環境変数が設定されていない
2. Xサーバー（VcXsrvなど）への接続ができない
3. GUIモードで接続しようとすると処理がブロックされる

### GUIが必要な場合

GUIが必要な場合は、直接スクリプトを実行してください：
- `srcs/move_to_tray.py`
- `srcs/check_gui.py`
- `srcs/spawn_objects.py`

## 動作確認結果

✅ PyBullet接続成功  
✅ R2D2ロボットのロード成功  
✅ 物理シミュレーション実行成功  
✅ ロボットの落下が正常に計算される（高さ1.0m → 0.78m）
