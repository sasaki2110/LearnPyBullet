import pybullet as p
import pybullet_data
import time

def main():
    # --- 物理サーバーへの接続 (GUIモード) ---
    # ここを GUI に変更します
    client_id = p.connect(p.GUI)
    #client_id = p.connect(p.GUI, options="--opengl2")    

    if client_id < 0:
        print("GUI接続に失敗しました。DISPLAY設定やVcXsrvを確認してください。")
        return

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # 地面とロボットをロード
    p.loadURDF("plane.urdf")
    robot_id = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 2.0]) # 少し高めから

    print("Windows側にPyBulletのウィンドウが表示されているはずです。")

    # ゆっくり落下する様子を見るために sleep を入れる
    for i in range (1000):
        p.stepSimulation()
        time.sleep(1./240.) # 物理シミュレーションの標準的な時間刻み
        
        if i % 100 == 0:
            pos, _ = p.getBasePositionAndOrientation(robot_id)
            print(f"ステップ {i}: 高さ {pos[2]:.2f}m")

    p.disconnect()

if __name__ == "__main__":
    main()