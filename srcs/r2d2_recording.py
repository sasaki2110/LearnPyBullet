"""
R2D2がふらふら歩くサンプル（録画版）

R2D2のホイール関節を制御して、ふらふら歩くような動きを実現し、
その様子をmp4動画として録画します。
DIRECTモードで動作し、カメラ画像を取得して動画として保存します。
"""
import pybullet as p
import pybullet_data
import time
import math
import numpy as np
import imageio
import os

def main():
    # DIRECTモードで接続（録画用）
    client_id = p.connect(p.DIRECT)
    
    if client_id < 0:
        print("PyBullet接続に失敗しました。")
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
    
    # カメラパラメータを設定
    width, height = 1280, 720  # 高解像度で録画
    fps = 30  # フレームレート
    
    # 録画用のフレームリスト
    frames = []
    
    print("🚀 R2D2の歩行を開始します（録画中）...")
    print("   - 左右のホイールを交互に回転させて歩行を実現")
    print("   - 頭部も左右に振ります")
    print(f"   - 解像度: {width}x{height}, FPS: {fps}\n")
    
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
        
        # R2D2の位置を取得（カメラ追従用）
        pos, _ = p.getBasePositionAndOrientation(robot_id)
        
        # カメラをR2D2の位置に追従させる
        # カメラはR2D2の後ろ上から見下ろす角度で配置
        camera_distance = 3.0
        camera_yaw = 45.0  # 水平角度（度）
        camera_pitch = -20.0  # 垂直角度（度、上から見下ろす）
        
        # カメラ位置を計算
        yaw_rad = math.radians(camera_yaw)
        pitch_rad = math.radians(camera_pitch)
        camera_x = pos[0] + camera_distance * math.cos(yaw_rad) * math.cos(pitch_rad)
        camera_y = pos[1] + camera_distance * math.sin(yaw_rad) * math.cos(pitch_rad)
        camera_z = pos[2] + camera_distance * math.sin(pitch_rad) + 1.0
        
        # ビューマトリックスを計算
        view_matrix = p.computeViewMatrix(
            cameraEyePosition=[camera_x, camera_y, camera_z],
            cameraTargetPosition=pos,  # R2D2の位置を注視
            cameraUpVector=[0, 0, 1]  # Z軸が上方向
        )
        
        # プロジェクションマトリックスを計算
        projection_matrix = p.computeProjectionMatrixFOV(
            fov=60,  # 視野角（度）
            aspect=width / height,
            nearVal=0.1,
            farVal=100.0
        )
        
        # カメラ画像を取得（30FPSで録画するため、240Hzのうち8フレームに1回）
        if i % 8 == 0:  # 240Hz / 8 = 30FPS
            images = p.getCameraImage(
                width, height,
                viewMatrix=view_matrix,
                projectionMatrix=projection_matrix
            )
            
            # RGB画像を取得
            # getCameraImageは (width, height, rgbPixels, depthPixels, segmentationMaskBuffer) を返す
            # rgbPixelsは1次元配列として返されるので、適切にリシェイプする
            rgb_pixels = images[2]  # rgbPixels
            rgb_array = np.array(rgb_pixels, dtype=np.uint8)
            
            # 形状を (height, width, 4) にリシェイプ（RGBA）
            rgb_array = rgb_array.reshape((height, width, 4))
            
            # RGBのみを取得（RGBA -> RGB）
            rgb_array = rgb_array[:, :, :3]
            
            # フレームリストに追加
            frames.append(rgb_array)
        
        # 定期的に状態を表示
        if i % 200 == 0:
            pos, orn = p.getBasePositionAndOrientation(robot_id)
            # オイラー角に変換（見やすくするため）
            euler = p.getEulerFromQuaternion(orn)
            max_frames = max_steps // 8  # 最大録画フレーム数
            remaining_frames = max_frames - len(frames)
            progress = (len(frames) / max_frames) * 100
            print(f"ステップ {i}/{max_steps} ({progress:.1f}%): 位置=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
            print(f"  向き: ({math.degrees(euler[0]):.1f}°, {math.degrees(euler[1]):.1f}°, {math.degrees(euler[2]):.1f}°)")
            print(f"  右ホイール速度: {right_wheel_speed:.2f} rad/s")
            print(f"  左ホイール速度: {left_wheel_speed:.2f} rad/s")
            print(f"  録画フレーム数: {len(frames)}/{max_frames} (残り: {remaining_frames}フレーム)")
    
    print("\n✅ 歩行シミュレーションが完了しました。")
    print(f"📹 録画フレーム数: {len(frames)}")
    
    # 動画として保存
    output_path = os.path.join(os.path.dirname(__file__), "r2d2_walk_recording.mp4")
    print(f"💾 動画を保存中: {output_path}")
    
    try:
        imageio.mimsave(output_path, frames, fps=fps, codec='libx264', quality=8)
        print(f"✅ 動画の保存が完了しました: {output_path}")
        print(f"   解像度: {width}x{height}, FPS: {fps}, フレーム数: {len(frames)}")
    except Exception as e:
        print(f"❌ 動画の保存に失敗しました: {e}")
        print("   imageioがインストールされているか確認してください: pip install imageio[ffmpeg]")
    
    p.disconnect()
    print("✅ シミュレーションを終了しました。")

if __name__ == "__main__":
    main()
