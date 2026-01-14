"""
Quadrotor（クアッドロータードローン）のサンプル

地面に配置後、2メートル上昇し、その後2メートル降下します。
GUIモードで1倍速表示します。
"""
import pybullet as p
import pybullet_data
import time
import math
# 録画用のインポート（コメントアウト）
# import numpy as np
# import imageio
# import os

def main():
    # GUIモードで接続
    client_id = p.connect(p.GUI)
    
    if client_id < 0:
        print("PyBullet接続に失敗しました。")
        return
    
    import os
    
    # ローカルのQuadrotorディレクトリのパスを取得
    quadrotor_dir = os.path.join(os.path.dirname(__file__), "Quadrotor")
    quadrotor_urdf = os.path.join(quadrotor_dir, "quadrotor.urdf")
    
    # 検索パスを追加（両方のパスを追加）
    # 注意: setAdditionalSearchPathは複数回呼ぶと、検索パスのリストに追加される
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # 標準データ用
    p.setAdditionalSearchPath(quadrotor_dir)  # Quadrotor用（メッシュファイルを見つけるため）
    
    p.setGravity(0, 0, -9.81)
    
    # 地面をロード（pybullet_dataから、絶対パスで確実に読み込む）
    plane_urdf = os.path.join(pybullet_data.getDataPath(), "plane.urdf")
    plane_id = p.loadURDF(plane_urdf)
    print("✅ 地面をロードしました")
    
    # Quadrotorを地面に配置（初期位置、絶対パスで読み込み）
    initial_height = 0.1  # 地面から10cm
    drone_id = p.loadURDF(quadrotor_urdf, basePosition=[0, 0, initial_height])
    print("✅ Quadrotorをロードしました")
    
    # 初期状態を安定させるために少しシミュレーションを進める
    for _ in range(50):
        p.stepSimulation()
    
    # 現在の位置を取得
    pos, _ = p.getBasePositionAndOrientation(drone_id)
    current_height = pos[2]
    print(f"📊 初期高さ: {current_height:.2f}m")
    
    # 目標高さ
    target_height_up = 2.0  # 2メートル上昇
    target_height_down = initial_height  # 元の高さに降下
    
    # 上昇力（重力を打ち消す + 上昇する力）
    # Quadrotorの質量は0.5kg（URDFから）
    # 重力を打ち消す力 = 質量 × 重力加速度 = 0.5 × 9.81 ≈ 4.9N
    mass = 0.5  # kg
    gravity = 9.81  # m/s²
    base_hover_force = mass * gravity  # 約4.9N（基本ホバリング用）
    
    # PID制御風のパラメータ（簡易版）
    kp = 2.0  # 比例ゲイン
    
    # 録画用のカメラパラメータ（コメントアウト）
    # width, height = 1280, 720  # 高解像度で録画
    # fps = 30  # フレームレート
    # frames = []  # 録画用のフレームリスト
    
    print("\n🚁 Quadrotorの飛行を開始します...")
    print("   1. 2メートル上昇")
    print("   2. ホバリング（少し待機）")
    print("   3. 2メートル降下\n")
    
    # フェーズ管理
    phase = "ascending"  # ascending, hovering, descending, finished
    hover_start_step = None
    
    # シミュレーションループ
    max_steps = 5000  # 約20秒（240Hzで）
    for i in range(max_steps):
        # 現在の位置と姿勢を取得
        pos, orn = p.getBasePositionAndOrientation(drone_id)
        current_height = pos[2]
        
        # 姿勢をオイラー角に変換（傾きを確認）
        euler = p.getEulerFromQuaternion(orn)
        roll = euler[0]  # X軸周りの回転
        pitch = euler[1]  # Y軸周りの回転
        
        # 姿勢が大きく傾いている場合、姿勢を補正するトルクを適用
        # 目標姿勢: 水平（roll=0, pitch=0）
        max_tilt = 0.3  # 最大許容傾き（ラジアン、約17度）
        if abs(roll) > max_tilt or abs(pitch) > max_tilt:
            # 姿勢を補正するトルクを適用
            # 傾きに比例したトルクで姿勢を戻す
            torque_roll = -kp * roll * 0.1  # X軸周りのトルク
            torque_pitch = -kp * pitch * 0.1  # Y軸周りのトルク
            
            p.applyExternalTorque(
                objectUniqueId=drone_id,
                linkIndex=-1,
                torqueObj=[torque_roll, torque_pitch, 0],
                flags=p.WORLD_FRAME
            )
        
        # フェーズに応じた制御
        if phase == "ascending":
            # 上昇フェーズ
            if current_height < target_height_up:
                # 目標高度との差に応じて力を調整（PID制御風）
                height_error = target_height_up - current_height
                # 上向きの力 = 基本ホバリング力 + 高さ誤差に比例した力（より緩やかに設定）
                # kpをさらに小さくして、より緩やかな上昇にする
                lift_force = base_hover_force + kp * 0.15 * height_error  # 0.15倍に減らす
                # 最大力を制限（暴走防止、より控えめに）
                lift_force = min(lift_force, base_hover_force * 1.2)  # 最大1.2倍に制限
                
                p.applyExternalForce(
                    objectUniqueId=drone_id,
                    linkIndex=-1,  # ベース
                    forceObj=[0, 0, lift_force],
                    posObj=[0, 0, 0],
                    flags=p.WORLD_FRAME
                )
            else:
                # 目標高度に到達
                print(f"✅ 目標高度 {target_height_up:.2f}m に到達しました（ステップ {i}）")
                phase = "hovering"
                hover_start_step = i
        
        elif phase == "hovering":
            # ホバリングフェーズ（目標高度を維持）
            if i - hover_start_step < 4000:  # 約16秒間ホバリング
                # 目標高度との差に応じて力を調整（フィードバック制御）
                height_error = target_height_up - current_height
                # 高さ誤差に比例した力を加える（上昇時と同程度の緩やかな出力）
                hover_force = base_hover_force + kp * 0.15 * height_error  # 0.15倍に調整
                # 力を制限（暴走防止、上昇時と同程度に）
                hover_force = max(base_hover_force * 0.8, min(hover_force, base_hover_force * 1.2))
                
                p.applyExternalForce(
                    objectUniqueId=drone_id,
                    linkIndex=-1,
                    forceObj=[0, 0, hover_force],
                    posObj=[0, 0, 0],
                    flags=p.WORLD_FRAME
                )
            else:
                # ホバリング終了、降下開始
                print("⬇️ 降下を開始します...")
                phase = "descending"
        
        elif phase == "descending":
            # 降下フェーズ（目標高度に向かって降下）
            if current_height > target_height_down:
                # 目標高度との差に応じて力を調整
                height_error = current_height - target_height_down
                # 降下時は、目標高度に近づくにつれて力を弱める
                # 高さが高いほど、力を弱くする（重力に任せる）
                if height_error > 1.0:  # 1m以上高い場合
                    descend_force = base_hover_force * 0.5  # 重力の50%を打ち消す（速く降下）
                elif height_error > 0.5:  # 0.5m以上高い場合
                    descend_force = base_hover_force * 0.7  # 重力の70%を打ち消す
                else:  # 0.5m以下
                    descend_force = base_hover_force * 0.9  # 重力の90%を打ち消す（緩やかに降下）
                
                p.applyExternalForce(
                    objectUniqueId=drone_id,
                    linkIndex=-1,
                    forceObj=[0, 0, descend_force],
                    posObj=[0, 0, 0],
                    flags=p.WORLD_FRAME
                )
            else:
                # 目標高度に到達
                print(f"✅ 降下完了。最終高さ: {current_height:.2f}m（ステップ {i}）")
                phase = "finished"
                # 着陸後は力を停止（重力で自然に地面に着地）
        
        elif phase == "finished":
            # 着陸後、力を停止（重力のみ）
            pass
            break
        
        # シミュレーションを1ステップ進める
        p.stepSimulation()
        time.sleep(1./240.) # 物理シミュレーションの標準的な時間刻み
        
        # ドローンの位置を取得
        pos, orn = p.getBasePositionAndOrientation(drone_id)
        
        # 録画用のカメラ追従コード（コメントアウト）
        # # カメラをドローンの位置に追従させる
        # # カメラはドローンの後ろ上から見下ろす角度で配置
        # camera_distance = 2.0  # カメラまでの距離（2m）
        # camera_yaw = 45.0  # 水平角度（度）
        # camera_pitch = -30.0  # 垂直角度（度、上から見下ろす）
        # 
        # # カメラ位置を計算
        # yaw_rad = math.radians(camera_yaw)
        # pitch_rad = math.radians(camera_pitch)
        # camera_x = pos[0] + camera_distance * math.cos(yaw_rad) * math.cos(pitch_rad)
        # camera_y = pos[1] + camera_distance * math.sin(yaw_rad) * math.cos(pitch_rad)
        # camera_z = pos[2] + camera_distance * math.sin(pitch_rad) + 0.5
        # 
        # # ビューマトリックスを計算
        # view_matrix = p.computeViewMatrix(
        #     cameraEyePosition=[camera_x, camera_y, camera_z],
        #     cameraTargetPosition=pos,  # ドローンの位置を注視
        #     cameraUpVector=[0, 0, 1]  # Z軸が上方向
        # )
        # 
        # # プロジェクションマトリックスを計算
        # projection_matrix = p.computeProjectionMatrixFOV(
        #     fov=60,  # 視野角（度）
        #     aspect=width / height,
        #     nearVal=0.1,
        #     farVal=100.0
        # )
        # 
        # # カメラ画像を取得（30FPSで録画するため、240Hzのうち8フレームに1回）
        # if i % 8 == 0:  # 240Hz / 8 = 30FPS
        #     images = p.getCameraImage(
        #         width, height,
        #         viewMatrix=view_matrix,
        #         projectionMatrix=projection_matrix
        #     )
        #     
        #     # RGB画像を取得
        #     rgb_pixels = images[2]  # rgbPixels
        #     rgb_array = np.array(rgb_pixels, dtype=np.uint8)
        #     
        #     # 形状を (height, width, 4) にリシェイプ（RGBA）
        #     rgb_array = rgb_array.reshape((height, width, 4))
        #     
        #     # RGBのみを取得（RGBA -> RGB）
        #     rgb_array = rgb_array[:, :, :3]
        #     
        #     # フレームリストに追加
        #     frames.append(rgb_array)
        
        # 定期的に状態を表示
        if i % 200 == 0:
            euler = p.getEulerFromQuaternion(orn)
            print(f"ステップ {i}: 高さ = {pos[2]:.2f}m, フェーズ = {phase}, "
                  f"Roll = {euler[0]:.2f}rad, Pitch = {euler[1]:.2f}rad")
                  # f"録画フレーム数: {len(frames)}")  # 録画用（コメントアウト）
    
    print("\n✅ 飛行シミュレーションが完了しました。")
    
    # 録画用の動画保存コード（コメントアウト）
    # print(f"📹 録画フレーム数: {len(frames)}")
    # output_path = os.path.join(os.path.dirname(__file__), "quadrotor_flight_recording.mp4")
    # print(f"💾 動画を保存中: {output_path}")
    # try:
    #     imageio.mimsave(output_path, frames, fps=fps, codec='libx264', quality=8)
    #     print(f"✅ 動画の保存が完了しました: {output_path}")
    #     print(f"   解像度: {width}x{height}, FPS: {fps}, フレーム数: {len(frames)}")
    # except Exception as e:
    #     print(f"❌ 動画の保存に失敗しました: {e}")
    #     print("   imageioがインストールされているか確認してください: pip install imageio[ffmpeg]")
    
    p.disconnect()
    print("✅ シミュレーションを終了しました。")

if __name__ == "__main__":
    main()
