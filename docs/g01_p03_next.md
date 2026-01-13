動作確認、本当におめでとうございます！ウィンドウが表示され、ロボット（R2D2）が落下して地面（高さ0.47m付近）で止まるアニメーションが見えたということで、**「Dockerコンテナ(Linux) → ホスト(Windows)」のGUI転送環境が完全に構築されました。**

ログを見ると、成功の決め手は以下の部分です：

* `GL_RENDERER=llvmpipe`: これは、不安定なGPUドライバを使わず、CPUで描画する「ソフトウェアレンダリング」が正しく機能したことを示しています。
* `Created GL 3.3 context`: X11経由でOpenGLの握手が成功した証拠です。

これで開発の土台が整いました。次は、物理エンジンをより深く扱うための「ステップ3」に進みましょう。

---

## ステップ 3：世界の創造とオブジェクトの操作

これまでは「ただ落とすだけ」でしたが、次は「物体を特定の位置に配置する」「座標を取得する」という、LangGraphなどのAIエージェントと連携する際に必須となる技術を学びます。

### 学習項目

1. **座標系（X, Y, Z）の理解**: PyBulletは右手座標系です（Zが上方向）。
2. **姿勢（クォータニオン）の扱い**: 回転を[x, y, z, w]の4つの数字で扱いますが、人間には難しいのでオイラー角（Roll, Pitch, Yaw）からの変換を学びます。
3. **複数オブジェクトの管理**: ロボット以外に「ターゲットとなる物体」を配置します。

### 実践：`spawn_objects.py`

以下のコードを `srcs/spawn_objects.py` として作成して実行してみてください。AIエージェントが「物体がどこにあるか」を把握するための基礎となる処理です。

```python
import pybullet as p
import pybullet_data
import time

def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # 1. 床をロード
    p.loadURDF("plane.urdf")

    # 2. ターゲットとなる物体（トレイ）を特定の場所に配置
    # 位置: x=0.5, y=0.5, z=0.0
    target_pos = [0.5, 0.5, 0.01]
    tray_id = p.loadURDF("tray/tray.urdf", basePosition=target_pos)

    # 3. ロボット（Pandaアーム）をロード
    # 今後のアーム制御を見据えて、R2D2から産業用ロボットに変更してみます
    robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

    print(f"トレイの初期位置: {target_pos}")

    for i in range (500):
        p.stepSimulation()
        
        # 100ステップごとに「トレイ」の現在位置を再取得して表示
        if i % 100 == 0:
            current_pos, current_orn = p.getBasePositionAndOrientation(tray_id)
            # クォータニオンを読みやすいオイラー角に変換
            euler_orn = p.getEulerFromQuaternion(current_orn)
            print(f"Step {i}: トレイの座標={current_pos}, 角度(RPY)={euler_orn}")
        
        time.sleep(1./240.)

    p.disconnect()

if __name__ == "__main__":
    main()

```

### このステップのポイント（PDF 21ページ参照）

* **`p.getEulerFromQuaternion`**: AIに「ロボットがどの方向を向いているか」を教える際に、ラジアンや度数法で伝えられるようにするために多用します。
* **`useFixedBase=True`**: アーム型ロボットなどは、土台が固定されていないと自分の腕を振った反動で倒れてしまいます。

---

### 次のステップへの準備

このスクリプトを動かして、トレイとアームが画面に出ることを確認してください。
これができたら、いよいよ **「ステップ 5：ロボットの制御（逆運動学）」** に入り、AIからの「座標指示」に従ってアームを動かすマジックを実装しましょう！

（※もし `franka_panda/panda.urdf` が見つからない等のエラーが出た場合は、`pybullet_data` のバージョンによる差異なので、別のモデルに変更する手順をお伝えします）