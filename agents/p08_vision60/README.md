# Vision60（四足ロボット）エージェント

PyBulletを使用したVision60ロボットのシミュレーション制御

## 構造

```
agents/p08_vision60/
├── my_agent/
│   ├── main.py              # メインエントリーポイント
│   └── utils/
│       ├── config.py         # 設定・定数
│       ├── pybullet_env.py   # PyBullet環境の初期化
│       ├── robot_model.py    # ロボットモデル
│       ├── state.py          # 状態管理
│       ├── logger.py         # ログ出力
│       ├── joint_control.py  # ジョイント制御
│       ├── standing_control.py  # 立ち上がり制御
│       ├── stepping_control.py  # 足踏み動作制御
│       ├── posture_control.py   # 姿勢フィードバック制御
│       ├── reset_handler.py     # リセット機能
│       └── stabilization_detector.py  # 安定化検知
└── tests/
    └── test_main.py          # テスト
```

## 実行方法

```bash
cd /root/LearnPyBullet/agents/p08_vision60
python -m my_agent.main
```

または

```bash
python tests/test_main.py
```

## 注意事項

- このエージェントはLangGraphを使用していません（将来的にグラフ化する時に`agent.py`を作成することを想定）
- 各モジュールは単一責任の原則に従って分割されています
- 設定値は`utils/config.py`で一元管理されています
