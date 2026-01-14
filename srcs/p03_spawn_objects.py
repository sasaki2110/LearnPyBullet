import pybullet as p
import pybullet_data
import time

def main():
    # GUIモードで接続（前回のソフトウェアレンダリング設定が効いているはずです）
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # 床の設置
    p.loadURDF("plane.urdf")

    # 1. ターゲット物体の配置 (x=0.5, y=0.0, z=0.01)
    target_pos = [0.5, 0.0, 0.01]
    tray_id = p.loadURDF("tray/tray.urdf", basePosition=target_pos)

    # 2. ロボット(Franka Panda)の設置
    # useFixedBase=True で土台を地面に固定します
    robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

    print(f"Pandaロボットとトレイを召喚しました。")

    # シミュレーションループ
    for i in range (1000):
        p.stepSimulation()
        
        if i % 200 == 0:
            # トレイの位置を取得して表示
            pos, _ = p.getBasePositionAndOrientation(tray_id)
            print(f"Step {i}: トレイの現在地 {pos}")
            
        time.sleep(1./240.) # 実時間に近い速度で表示

    p.disconnect()

if __name__ == "__main__":
    main()