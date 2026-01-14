import pybullet as p
import pybullet_data
import time

def main():
    # --- 物理サーバーへの接続 ---
    # コンテナ内ではGUIがないため、まずは p.DIRECT を使用します
    client_id = p.connect(p.DIRECT)
    
    if client_id < 0:
        print("物理サーバーへの接続に失敗しました。")
        return

    # --- 基本設定 ---
    # PyBulletに付属している標準データ（床やロボットモデル）のパスを追加
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # 重力を地球（-9.81）に設定
    p.setGravity(0, 0, -9.81)

    # --- データのロード ---
    # 床(plane)を召喚
    plane_id = p.loadURDF("plane.urdf")
    # ロボット(R2D2)を空中(高さ1m)に召喚
    robot_id = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 1.0])

    print(f"接続成功！ Client ID: {client_id}")
    print(f"標準データパス: {pybullet_data.getDataPath()}")

    # --- 物理シミュレーションの進展テスト ---
    for i in range(50):
        p.stepSimulation()
        if i % 10 == 0:
            pos, _ = p.getBasePositionAndOrientation(robot_id)
            print(f"ステップ {i}: ロボットの現在の高さ = {pos[2]:.4f}m")

    # --- 切断 ---
    p.disconnect()
    print("シミュレーションを正常に終了しました。")

if __name__ == "__main__":
    main()

