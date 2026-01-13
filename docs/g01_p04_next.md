「もこもこと白いものが上に伸びている」というのは、まさに **Franka Panda** という産業用ロボットアームが、初期状態で真上（垂直）にピンと立っている姿ですね！

無事に召喚できて何よりです。これで「静止した世界」は作れました。
次は、いよいよ今回の学習ロードマップの山場である **「ステップ 5：ロボットの制御（逆運動学）」** に挑戦しましょう。

---

## ステップ 5：逆運動学（IK）でアームを自由に動かす

「この座標  を触って」というAIの指示を、ロボットの「各関節の角度」に翻訳するのが **逆運動学（Inverse Kinematics: IK）** です。

### 学習項目

* **`p.calculateInverseKinematics()`**: 目標座標を入力すると、そこへ到達するための関節角度のリストを返してくれます。
* **`p.setJointMotorControl2()`**: 計算した角度に向けて、実際にモーターを回して腕を動かします。

### 実践：`move_to_tray.py`

トレイの少し上の位置（空中の座標）をターゲットにして、アームがそこへ「シュッ」と動くスクリプトを作成します。

```python
import pybullet as p
import pybullet_data
import time

def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    p.loadURDF("plane.urdf")
    tray_id = p.loadURDF("tray/tray.urdf", basePosition=[0.5, 0.0, 0.01])
    robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

    # Pandaアームの手先（エンドエフェクタ）のインデックス番号
    # Pandaの場合は通常 11 番が手先に相当します
    end_effector_index = 11

    # 【目標座標】トレイの真上 20cm の場所
    target_position = [0.5, 0.0, 0.2]

    print(f"ターゲット座標 {target_position} に向けて移動を開始します...")

    # シミュレーションを回しながら制御する
    for i in range (1000):
        # 1. 逆運動学の計算：目標地点に行くための関節角度を算出
        joint_poses = p.calculateInverseKinematics(
            robot_id, 
            end_effector_index, 
            target_position
        )

        # 2. モーター制御：各関節（0番〜6番くらいまで）を計算した角度に動かす
        for j in range(len(joint_poses)):
            p.setJointMotorControl2(
                bodyIndex=robot_id,
                jointIndex=j,
                controlMode=p.POSITION_CONTROL,
                targetPosition=joint_poses[j]
            )

        p.stepSimulation()
        
        if i % 200 == 0:
            # 現在の手先の位置を確認
            ls = p.getLinkState(robot_id, end_effector_index)
            actual_pos = ls[4] # 世界座標系での位置
            print(f"Step {i}: 現在の手先位置 = {actual_pos}")

        time.sleep(1./240.)

    print("移動完了。5秒間停止して終了します。")
    time.sleep(5)
    p.disconnect()

if __name__ == "__main__":
    main()

```

---

### ここでの学び（PDF 63ページ参照）

1. **エンドエフェクタ**: ロボットの「手」に相当する部分です。IKはこの「手」をどこに持っていくかを計算します。
2. **POSITION_CONTROL**: 「この角度になれ」と命令するモードです。これを使うことで、AIエージェントからの「トレイに手を伸ばして」という抽象的な命令を、具体的なロボットの動きに変換できます。

### 次のステップへの展望

これが動けば、**「LangGraphがトレイの座標を特定し、このIKコードを呼び出す」** という仕組みを作るだけで、AIロボットの原型が完成します。

まずは、あの「白いもこもこ（アーム）」が、トレイに向かって **お辞儀をするようにグニャリと動く姿** を確認してみてください！