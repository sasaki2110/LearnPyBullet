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