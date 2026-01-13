"""
R2D2がふらふら歩くサンプル

R2D2のホイール関節を制御して、ふらふら歩くような動きを実現します。
GUIモードで動作します。
"""
import pybullet as p
import pybullet_data
import time
import math

def main():
    # GUIモードで接続
    client_id = p.connect(p.GUI)
    
    if client_id < 0:
        print("GUI接続に失敗しました。DISPLAY設定やVcXsrvを確認してください。")
        return
    
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    
    # 地面をロード
    plane_id = p.loadURDF("plane.urdf")
    print("✅ 地面をロードしました")
    
    # R2D2をロード（地面の上に配置）
    robot_id = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 0.5])
    print("✅ R2D2をロードしました")
    
    # 初期状態を安定させるために少しシミュレーションを進める
    for _ in range(50):
        p.stepSimulation()
    
    # 関節情報を取得
    num_joints = p.getNumJoints(robot_id)
    print(f"📊 R2D2の関節数: {num_joints}")
    
    # ホイール関節のインデックスを確認
    # 関節2: right_front_wheel_joint
    # 関節3: right_back_wheel_joint
    # 関節6: left_front_wheel_joint
    # 関節7: left_back_wheel_joint
    wheel_joints = [2, 3, 6, 7]  # ホイール関節のインデックス
    
    # 頭部の回転関節（オプション）
    head_joint = 13  # head_swivel
    
    print("🚀 R2D2の歩行を開始します...")
    print("   - 左右のホイールを交互に回転させて歩行を実現")
    print("   - 頭部も左右に振ります")
    print("\n📹 GUI操作:")
    print("   - 左マウスドラッグ: カメラを回転")
    print("   - 右マウスドラッグ: カメラをパン（移動）")
    print("   - マウスホイール: ズームイン/アウト")
    print("   - Homeキー: カメラビューをリセット")
    print("   - カメラはR2D2を自動追従します\n")
    
    # シミュレーションループ
    max_steps = 5000  # 約20秒（240Hzで）
    for i in range(max_steps):
        # 時間に基づいて周期的な動きを生成
        t = i * 0.01  # 時間（秒）
        
        # 歩行パターン: 左右のホイールを交互に回転させて前進
        # 基本速度（前進）
        base_speed = 3.0
        
        # 左右のホイールにわずかな位相差を与えてふらふら歩く
        # 右側のホイール
        right_wheel_speed = base_speed + math.sin(t * 1.5) * 2.0  # 1.5Hz、±2rad/sの変動
        # 左側のホイール（少し位相をずらす）
        left_wheel_speed = base_speed + math.sin(t * 1.5 + 0.3) * 2.0  # 右と少し位相がずれている
        
        # ホイールを回転させる（VELOCITY_CONTROL）
        for joint_idx in wheel_joints:
            if joint_idx in [2, 3]:  # 右側のホイール
                p.setJointMotorControl2(
                    bodyIndex=robot_id,
                    jointIndex=joint_idx,
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocity=right_wheel_speed,
                    force=10.0  # 最大トルク
                )
            else:  # 左側のホイール
                p.setJointMotorControl2(
                    bodyIndex=robot_id,
                    jointIndex=joint_idx,
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocity=left_wheel_speed,
                    force=10.0
                )
        
        # 頭部を左右に振る（オプション）
        head_angle = math.sin(t * 1.5) * 0.5  # 1.5Hz、最大0.5rad（約30度）
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=head_joint,
            controlMode=p.POSITION_CONTROL,
            targetPosition=head_angle,
            force=5.0
        )
        
        # シミュレーションを1ステップ進める
        p.stepSimulation()
        
        # カメラをR2D2の位置に追従させる（画面から見失わないように）
        pos, _ = p.getBasePositionAndOrientation(robot_id)
        p.resetDebugVisualizerCamera(
            cameraDistance=3.0,      # カメラまでの距離（3m）
            cameraYaw=45.0,          # 水平角度（45度）
            cameraPitch=-20.0,       # 垂直角度（上から見下ろす）
            cameraTargetPosition=pos  # R2D2の位置を追従
        )
        
        # 定期的に状態を表示
        if i % 200 == 0:
            pos, orn = p.getBasePositionAndOrientation(robot_id)
            # オイラー角に変換（見やすくするため）
            euler = p.getEulerFromQuaternion(orn)
            print(f"ステップ {i}: 位置=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
            print(f"  向き: ({math.degrees(euler[0]):.1f}°, {math.degrees(euler[1]):.1f}°, {math.degrees(euler[2]):.1f}°)")
            print(f"  右ホイール速度: {right_wheel_speed:.2f} rad/s")
            print(f"  左ホイール速度: {left_wheel_speed:.2f} rad/s")
        
        # GUI表示のための待機
        time.sleep(1./240.)  # 240Hz
    
    print("\n✅ 歩行シミュレーションが完了しました。")
    print("5秒間停止して終了します...")
    time.sleep(5)
    
    p.disconnect()
    print("✅ シミュレーションを終了しました。")

if __name__ == "__main__":
    main()
