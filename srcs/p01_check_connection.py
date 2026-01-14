import pybullet as p
import pybullet_data
import time
import math

def main():
    # --- 物理サーバーへの接続 ---
    # コンテナ内ではGUIがないため、まずは p.DIRECT を使用します
    """とりあえず、GUIとDIRECTは理解した"""
    #client_id = p.connect(p.DIRECT)
    client_id = p.connect(p.GUI)
    
    if client_id < 0:
        print("物理サーバーへの接続に失敗しました。")
        return

    # --- 基本設定 ---
    # PyBulletに付属している標準データ（床やロボットモデル）のパスを追加
    """ここは呪文でいいかな"""
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # 重力を地球（-9.81）に設定
    """ここは呪文でいいかな"""
    p.setGravity(0, 0, -9.81)

    # --- データのロード ---
    # 床(plane)を召喚
    """ここは呪文でいいかな。床を変えたい時だけ気にする。"""
    plane_id = p.loadURDF("plane.urdf")
    # ロボット(R2D2)を空中(高さ1m)に召喚
    """ここでロボットを召喚する。ロボットを変えたい時だけ気にする。"""
    """basePositionはロボットの位置を指定する。座標はm単位。"""
    robot_id = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 1.0])

    print(f"接続成功！ Client ID: {client_id}")
    print(f"標準データパス: {pybullet_data.getDataPath()}")

    steps = 50

    # --- 物理シミュレーションの進展テスト ---
    """指定されたステップ数シミュレーションする"""
    for i in range(steps):
        """1ステップ進める"""
        p.stepSimulation()
        """10ステップごとにロボットの位置を取得して表示する"""
        if i % 10 == 0:
            """ロボットの位置と姿勢を取得する"""
            pos, orn = p.getBasePositionAndOrientation(robot_id)
            print(f"ステップ {i}: ロボットの現在の高さ = {pos[2]:.4f}m")

            # クォータニオン形式で表示
            print(f"位置: {pos}")
            print(f"姿勢（クォータニオン）: {orn}")

            # オイラー角に変換
            euler = p.getEulerFromQuaternion(orn)
            print(f"姿勢（オイラー角・ラジアン）: {euler}")

            # 度に変換して見やすく表示
            print(f"向き:")
            print(f"  Roll:  {math.degrees(euler[0]):.2f}° (X軸周り)")
            print(f"  Pitch: {math.degrees(euler[1]):.2f}° (Y軸周り)")
            print(f"  Yaw:   {math.degrees(euler[2]):.2f}° (Z軸周り)")

    # --- 切断 ---
    p.disconnect()
    print("シミュレーションを正常に終了しました。")

if __name__ == "__main__":
    main()

