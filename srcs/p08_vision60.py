"""
Vision60（四足ロボット）のサンプル

まずはロボットを登場させて、安定して立たせることを目標にします。
"""
import pybullet as p
import pybullet_data
import time
import math

def main():
    # GUIモードで接続
    client_id = p.connect(p.GUI)
    
    if client_id < 0:
        print("GUI接続に失敗しました。")
        return
    
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    
    # 地面をロード
    plane_id = p.loadURDF("plane.urdf")
    print("✅ 地面をロードしました")
    
    # Vision60をロード（脚を閉じた状態で登場させる）
    initial_pos = [0, 0, 0.1]  # 地面から10cm上
    robot_id = p.loadURDF("quadruped/vision60.urdf", basePosition=initial_pos)
    print("✅ Vision60をロードしました")
    
    # カメラを初期設定（GUIが開いた時点からクローズアップ）
    p.resetDebugVisualizerCamera(
        cameraDistance=2.0,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=initial_pos
    )
    
    # ジョイント情報を取得
    num_joints = p.getNumJoints(robot_id)
    print(f"\n📊 ジョイント数: {num_joints}")
    
    # 最初の16個のジョイント情報を表示
    print("\n📊 ジョイント情報:")
    for i in range(min(16, num_joints)):
        joint_info = p.getJointInfo(robot_id, i)
        joint_name = joint_info[1].decode('utf-8')
        joint_type = joint_info[2]
        print(f"  ジョイント {i}: {joint_name} (タイプ: {joint_type})")
    
    # Vision60の脚のジョイント構造
    # 脚0: ジョイント0(abduction), 1(hip), 2(knee) - front_left
    # 脚1: ジョイント4(abduction), 5(hip), 6(knee) - front_right
    # 脚2: ジョイント8(abduction), 9(hip), 10(knee) - back_left
    # 脚3: ジョイント12(abduction), 13(hip), 14(knee) - back_right
    
    # 立つ姿勢のジョイント角度を設定
    # abduction: 脚を横に広げる（左右のバランス）
    # hip: 脚を前後に動かす
    # knee: 膝を曲げる（逆関節ロボットなので後ろ向きに曲げる）
    
    # 各脚のジョイント角度（試行錯誤が必要な値）
    leg_joints = {
        'front_left': [0, 1, 2],      # abduction, hip, knee
        'front_right': [4, 5, 6],
        'back_left': [8, 9, 10],
        'back_right': [12, 13, 14]
    }
    
    # 立つ姿勢（脚を閉じた状態で登場）
    # 脚を伸ばした状態（kneeを小さく）で登場させ、そのまま安定させる
    standing_angles = {
        'front_left': [0.0, 0.0, 0.5],      # abduction(0), hip(0), knee(脚を伸ばした状態)
        'front_right': [0.0, 0.0, 0.5],
        'back_left': [0.0, 0.0, 0.5],      # すべての脚を同じ角度で
        'back_right': [0.0, 0.0, 0.5]
    }
    
    # 立つ姿勢を設定
    print("🦵 立つ姿勢を設定中...")
    for leg_name, joint_indices in leg_joints.items():
        angles = standing_angles.get(leg_name, [0.0, 0.0, 1.5])
        for i, joint_idx in enumerate(joint_indices):
            try:
                # ジョイントを目標角度にリセット
                p.resetJointState(robot_id, joint_idx, targetValue=angles[i], targetVelocity=0.0)
                # 速度モーターを無効にする（重要：デフォルトの速度モーターが干渉しないように）
                p.setJointMotorControl2(
                    bodyIndex=robot_id,
                    jointIndex=joint_idx,
                    controlMode=p.VELOCITY_CONTROL,
                    force=0.0  # 速度モーターを無効化
                )
            except:
                print(f"  警告: ジョイント {joint_idx} の設定に失敗しました")
    
    # 姿勢を設定した後、そのまま放置して安定させる（高さ調整は行わない）
    print("⚙️ 姿勢を安定化中（脚を閉じた状態でそのまま放置）...")
    for _ in range(100):  # 100ステップ進めて安定させる
        p.stepSimulation()
        time.sleep(1.0 / 240.0)
    
    # 安定化後の初期高さを記録（リセットチェック用）
    initial_pos_after_setup, _ = p.getBasePositionAndOrientation(robot_id)
    target_height = initial_pos_after_setup[2]  # 安定化後の高さを基準とする
    
    # 姿勢を維持・安定化（PD制御と姿勢フィードバック制御を使用）
    print("⚡ 姿勢を維持・安定化中（PD制御と姿勢フィードバック制御）...")
    time_step = 1.0 / 240.0
    
    # PD制御パラメータ
    position_gain = 0.2  # 位置ゲイン（Kp）
    velocity_gain = 1.0  # 速度ゲイン（Kd）
    
    # 姿勢フィードバック制御のゲイン（過剰反応を防ぐため小さめに設定）
    roll_feedback_gain = 0.01  # roll誤差に対するabduction調整ゲイン（0.05 → 0.01に削減）
    pitch_feedback_gain = 0.005  # pitch誤差に対するhip調整ゲイン（0.02 → 0.005に削減）
    
    # 位置フィードバック制御のゲイン（立ち上がり中の位置移動を抑制）
    position_feedback_gain = 0.001  # 位置誤差に対するabduction調整ゲイン（左右バランス）
    
    reset_count = 0
    reset_reasons_count = {'位置移動': 0, '高さ低下': 0, '姿勢傾き': 0}
    
    # 立ち上がり制御用の変数
    stability_confirmed = False
    stability_check_start = None
    standing_up_angles = None  # 立ち上がった後の角度（従来方式用）
    initial_standing_angles = None  # 立ち上がり開始時の初期角度（固定）
    standing_up_start_step = None  # 立ち上がり開始ステップ
    standing_up_duration = 2000  # 立ち上がりにかけるステップ数（極めてゆっくりと、1000→2000に延長）
    
    # 逆運動学を使った立ち上がり制御用の変数（現在は無効）
    use_ik_for_standing_up = False  # 逆運動学を使用するかどうか（Vision60では動作不良のため無効）
    end_effector_indices_ik = {}  # 各脚のエンドエフェクタ（足先）のリンクインデックス
    initial_toe_positions_ik = {}  # 立ち上がり開始時の足先位置
    target_toe_positions_ik = {}  # 目標の足先位置
    
    # リンク名マッピング（立ち上がりログ出力用）
    link_name_to_index = {}
    num_links = p.getNumJoints(robot_id)
    for link_idx in range(num_links):
        try:
            joint_info = p.getJointInfo(robot_id, link_idx)
            link_name = joint_info[12].decode('utf-8') if joint_info[12] else f"link_{link_idx}"
            link_name_to_index[link_name] = link_idx
        except:
            pass
    
    # 前回のログ値（差分計算用）
    prev_roll = None
    prev_pitch = None
    prev_knee_angles = None
    prev_toe_heights = None
    prev_base_pos = None
    
    total_simulation_steps = 3000  # 姿勢維持のステップ数（立ち上がり2000 + 安定化500ステップを完了させるため延長）
    for i in range(total_simulation_steps):
        # ベース姿勢を取得（姿勢フィードバック制御用）
        current_pos, current_orn = p.getBasePositionAndOrientation(robot_id)
        current_euler = p.getEulerFromQuaternion(current_orn)
        current_pitch = math.degrees(current_euler[1])
        current_roll = math.degrees(current_euler[0])
        
        # 安定確認（リセットが発生していない場合、20ステップ経過で安定とみなす）
        if not stability_confirmed and reset_count == 0:
            if stability_check_start is None:
                stability_check_start = i
            elif i - stability_check_start >= 20:  # 20ステップ安定していたら
                stability_confirmed = True
                standing_up_start_step = i  # 立ち上がり開始ステップを記録
                # 立ち上がり開始時の初期角度を固定して保存（重要：毎回変わるstanding_anglesではなく、固定値を使う）
                initial_standing_angles = {
                    leg_name: list(angles) for leg_name, angles in standing_angles.items()
                }
                print(f"  ✅ 安定確認完了（ステップ{i}）: リセット無しで20ステップ経過")
                
                # 立ち上がる角度を設定（従来方式：角度を直接指定）
                # 実機に合わせて膝を大きく開く - 1.7ラジアン（約97度）
                # 前のめりを防ぐため、前脚を後ろ向きに、後脚を前向きに調整（後傾姿勢）
                standing_up_angles = {
                    'front_left': [0.0, 0.5, 1.7],      # abduction, hip(後ろ向き、大きく), knee(大きく開く、約97度)
                    'front_right': [0.0, 0.5, 1.7],
                    'back_left': [0.0, -0.2, 1.7],      # 後脚はhipを前向きに（負の値）で後傾姿勢を実現
                    'back_right': [0.0, -0.2, 1.7]
                }
                print(f"  🦵 立ち上がります（膝を大きく開き、hipを調整 - {standing_up_duration}ステップかけてゆっくりと）...")
        
        # 立ち上がり処理（安定確認後、段階的に角度を変更）
        # 逆運動学を使った制御の場合
        if stability_confirmed and use_ik_for_standing_up and len(end_effector_indices_ik) > 0 and len(target_toe_positions_ik) > 0 and standing_up_start_step is not None:
            # 進行度を計算（0.0～1.0）
            elapsed_steps = i - standing_up_start_step
            progress = min(1.0, elapsed_steps / standing_up_duration)
            
            # 各脚の目標足先位置を線形補間（初期位置から目標位置へ）
            current_target_toe_positions = {}
            for leg_name in end_effector_indices_ik.keys():
                if initial_toe_positions_ik.get(leg_name) is not None and target_toe_positions_ik.get(leg_name) is not None:
                    current_target_toe_positions[leg_name] = [
                        initial_toe_positions_ik[leg_name][0] + (target_toe_positions_ik[leg_name][0] - initial_toe_positions_ik[leg_name][0]) * progress,
                        initial_toe_positions_ik[leg_name][1] + (target_toe_positions_ik[leg_name][1] - initial_toe_positions_ik[leg_name][1]) * progress,
                        initial_toe_positions_ik[leg_name][2] + (target_toe_positions_ik[leg_name][2] - initial_toe_positions_ik[leg_name][2]) * progress
                    ]
                else:
                    current_target_toe_positions[leg_name] = None
            
            # 各脚に対して逆運動学を計算
            ik_angles = {}
            for leg_name, joint_indices in leg_joints.items():
                if end_effector_indices_ik.get(leg_name) is not None and current_target_toe_positions.get(leg_name) is not None:
                    try:
                        # 逆運動学を計算
                        # 各脚のジョイントインデックスを取得（abduction, hip, knee）
                        joint_indices_for_ik = joint_indices  # [abduction, hip, knee]
                        
                        # 現在のジョイント角度を取得（初期値として使用）
                        current_joint_angles = []
                        for joint_idx in joint_indices_for_ik:
                            joint_state = p.getJointState(robot_id, joint_idx)
                            current_joint_angles.append(joint_state[0])
                        
                        # 逆運動学を計算
                        # calculateInverseKinematicsは全ジョイントの角度を返すので、
                        # 各脚のジョイントインデックスに対応する角度を取得する
                        ik_result = p.calculateInverseKinematics(
                            robot_id,
                            end_effector_indices_ik[leg_name],
                            current_target_toe_positions[leg_name],
                            maxNumIterations=100
                        )
                        
                        # 結果から該当するジョイントの角度を取得
                        # ik_resultは全ジョイントの角度のリストなので、各脚のジョイントインデックスに対応する角度を取得
                        if len(ik_result) > max(joint_indices_for_ik):
                            ik_angles[leg_name] = [
                                ik_result[joint_indices_for_ik[0]],  # abduction
                                ik_result[joint_indices_for_ik[1]],  # hip
                                ik_result[joint_indices_for_ik[2]]   # knee
                            ]
                        else:
                            # 逆運動学が失敗した場合、現在の角度を維持
                            ik_angles[leg_name] = current_joint_angles
                    except Exception as e:
                        # 逆運動学が失敗した場合、現在の角度を維持
                        try:
                            current_joint_angles = []
                            for joint_idx in joint_indices:
                                joint_state = p.getJointState(robot_id, joint_idx)
                                current_joint_angles.append(joint_state[0])
                            ik_angles[leg_name] = current_joint_angles
                        except:
                            ik_angles[leg_name] = [0.0, 0.0, 0.5]
                else:
                    # エンドエフェクタが見つからない場合、現在の角度を維持
                    try:
                        current_joint_angles = []
                        for joint_idx in joint_indices:
                            joint_state = p.getJointState(robot_id, joint_idx)
                            current_joint_angles.append(joint_state[0])
                        ik_angles[leg_name] = current_joint_angles
                    except:
                        ik_angles[leg_name] = [0.0, 0.0, 0.5]
            
            # standing_anglesを逆運動学の結果で更新
            standing_angles = ik_angles
        
        # 従来の角度指定方式（逆運動学を使わない場合）
        elif stability_confirmed and not use_ik_for_standing_up and standing_up_angles is not None and initial_standing_angles is not None and standing_up_start_step is not None:
            # 進行度を計算（0.0～1.0）
            elapsed_steps = i - standing_up_start_step
            progress = min(1.0, elapsed_steps / standing_up_duration)
            
            # 立ち上がり中の進行度をログ出力（100ステップごと）
            if elapsed_steps % 100 == 0 and elapsed_steps < standing_up_duration:
                # 現在のknee角度を取得
                current_knee_angles = {}
                for leg_name, joint_indices in leg_joints.items():
                    knee_joint = joint_indices[2]
                    knee_state = p.getJointState(robot_id, knee_joint)
                    current_knee_angles[leg_name] = math.degrees(knee_state[0])
                
                # 目標knee角度（逆運動学を使う場合は目標足先位置から計算、使わない場合はstanding_up_anglesから取得）
                if use_ik_for_standing_up and len(target_toe_positions_ik) > 0:
                    # 逆運動学を使う場合、目標knee角度は計算できないので、現在の最大knee角度を目標として表示
                    target_knee_deg = max(current_knee_angles.values()) if current_knee_angles else 97.4
                elif standing_up_angles is not None:
                    target_knee_deg = math.degrees(standing_up_angles['front_left'][2])
                else:
                    target_knee_deg = 97.4  # デフォルト値
                
                # 4つの足先（toe）の位置を取得
                toe_positions = {}
                leg_index_map = {'front_left': 0, 'front_right': 1, 'back_left': 2, 'back_right': 3}
                for leg_name, leg_idx in leg_index_map.items():
                    toe_link_name = f"toe{leg_idx}"
                    toe_link_idx = link_name_to_index.get(toe_link_name, -1)
                    if toe_link_idx >= 0:
                        try:
                            toe_state = p.getLinkState(robot_id, toe_link_idx)
                            toe_pos = toe_state[0]  # ワールド座標での位置
                            toe_positions[leg_name] = toe_pos
                        except:
                            toe_positions[leg_name] = None
                    else:
                        toe_positions[leg_name] = None
                
                # 膝角度の左右差・前後差を計算
                left_knee_avg = (current_knee_angles['front_left'] + current_knee_angles['back_left']) / 2.0
                right_knee_avg = (current_knee_angles['front_right'] + current_knee_angles['back_right']) / 2.0
                front_knee_avg = (current_knee_angles['front_left'] + current_knee_angles['front_right']) / 2.0
                back_knee_avg = (current_knee_angles['back_left'] + current_knee_angles['back_right']) / 2.0
                knee_lr_diff = left_knee_avg - right_knee_avg  # 左右差（左-右）
                knee_fb_diff = front_knee_avg - back_knee_avg  # 前後差（前-後）
                
                # 足先の高さ（Z座標）を取得
                toe_heights = {}
                if all(pos is not None for pos in toe_positions.values()):
                    for leg_name in leg_index_map.keys():
                        toe_heights[leg_name] = toe_positions[leg_name][2]
                    avg_toe_height = sum(toe_heights.values()) / 4.0
                else:
                    avg_toe_height = None
                
                # 姿勢変化の差分を計算
                roll_diff = current_roll - prev_roll if prev_roll is not None else 0.0
                pitch_diff = current_pitch - prev_pitch if prev_pitch is not None else 0.0
                
                # 膝角度の変化を計算
                knee_diffs = {}
                if prev_knee_angles is not None:
                    for leg_name in leg_joints.keys():
                        knee_diffs[leg_name] = current_knee_angles[leg_name] - prev_knee_angles[leg_name]
                else:
                    for leg_name in leg_joints.keys():
                        knee_diffs[leg_name] = 0.0
                
                # 足先の高さの変化を計算
                toe_height_diffs = {}
                if prev_toe_heights is not None and all(h is not None for h in toe_heights.values()):
                    for leg_name in leg_index_map.keys():
                        toe_height_diffs[leg_name] = toe_heights[leg_name] - prev_toe_heights[leg_name]
                else:
                    for leg_name in leg_index_map.keys():
                        toe_height_diffs[leg_name] = 0.0
                
                # ベース位置の変化を計算
                base_pos_diff = [0.0, 0.0, 0.0]
                if prev_base_pos is not None:
                    base_pos_diff = [
                        current_pos[0] - prev_base_pos[0],
                        current_pos[1] - prev_base_pos[1],
                        current_pos[2] - prev_base_pos[2]
                    ]
                
                # 各脚の接地判定と接触力を取得
                contact_info = {}
                for leg_name, leg_idx in leg_index_map.items():
                    toe_link_name = f"toe{leg_idx}"
                    toe_link_idx = link_name_to_index.get(toe_link_name, -1)
                    if toe_link_idx >= 0:
                        try:
                            # 接触点を取得
                            contact_points = p.getContactPoints(robot_id, plane_id, linkIndexA=toe_link_idx)
                            is_contact = len(contact_points) > 0
                            
                            # 接触力の合計を計算（接触点の数から推定、または実際の力があれば使用）
                            total_force = 0.0
                            if is_contact:
                                # リンクの状態から力を取得（可能な場合）
                                try:
                                    link_state = p.getLinkState(robot_id, toe_link_idx, computeLinkVelocity=True)
                                    # 接触点の数から推定（簡易的な方法）
                                    # 実際の力は別の方法で取得する必要がある場合があります
                                    total_force = len(contact_points) * 10.0  # 仮の値（接触点の数×10N）
                                except:
                                    total_force = len(contact_points) * 10.0  # 仮の値
                            
                            contact_info[leg_name] = {
                                'is_contact': is_contact,
                                'force': total_force,
                                'contact_count': len(contact_points)
                            }
                        except:
                            contact_info[leg_name] = {'is_contact': False, 'force': 0.0, 'contact_count': 0}
                    else:
                        contact_info[leg_name] = {'is_contact': False, 'force': 0.0, 'contact_count': 0}
                
                # 目標角度と実際の角度の誤差を計算
                angle_errors = {}
                for leg_name, joint_indices in leg_joints.items():
                    target_angles = standing_angles.get(leg_name, [0.0, 0.0, 0.5])
                    actual_angles = []
                    for j, joint_idx in enumerate(joint_indices):
                        try:
                            joint_state = p.getJointState(robot_id, joint_idx)
                            actual_angle = math.degrees(joint_state[0])
                            actual_angles.append(actual_angle)
                        except:
                            actual_angles.append(0.0)
                    
                    # 各ジョイントの誤差を計算（度単位）
                    errors = []
                    for j in range(len(joint_indices)):
                        target_deg = math.degrees(target_angles[j])
                        actual_deg = actual_angles[j]
                        error = actual_deg - target_deg
                        errors.append(error)
                    angle_errors[leg_name] = errors
                
                # ログ出力
                print(f"  📈 立ち上がり進行中 (ステップ{i}, 進行度{progress*100:.1f}%):")
                print(f"     姿勢: roll={current_roll:.1f}° (変化: {roll_diff:+.1f}°), pitch={current_pitch:.1f}° (変化: {pitch_diff:+.1f}°)")
                print(f"     ベース位置: X={current_pos[0]:.3f} ({base_pos_diff[0]:+.3f}), Y={current_pos[1]:.3f} ({base_pos_diff[1]:+.3f}), Z={current_pos[2]:.3f} ({base_pos_diff[2]:+.3f})")
                print(f"     膝角度: FL={current_knee_angles['front_left']:.1f}° ({knee_diffs['front_left']:+.1f}°), "
                      f"FR={current_knee_angles['front_right']:.1f}° ({knee_diffs['front_right']:+.1f}°), "
                      f"BL={current_knee_angles['back_left']:.1f}° ({knee_diffs['back_left']:+.1f}°), "
                      f"BR={current_knee_angles['back_right']:.1f}° ({knee_diffs['back_right']:+.1f}°) (目標={target_knee_deg:.1f}°)")
                print(f"     膝角度差: 左右差={knee_lr_diff:+.1f}° (左-右), 前後差={knee_fb_diff:+.1f}° (前-後)")
                print(f"     角度誤差: FL(abd={angle_errors['front_left'][0]:+.1f}°, hip={angle_errors['front_left'][1]:+.1f}°, knee={angle_errors['front_left'][2]:+.1f}°), "
                      f"FR(abd={angle_errors['front_right'][0]:+.1f}°, hip={angle_errors['front_right'][1]:+.1f}°, knee={angle_errors['front_right'][2]:+.1f}°)")
                print(f"             BL(abd={angle_errors['back_left'][0]:+.1f}°, hip={angle_errors['back_left'][1]:+.1f}°, knee={angle_errors['back_left'][2]:+.1f}°), "
                      f"BR(abd={angle_errors['back_right'][0]:+.1f}°, hip={angle_errors['back_right'][1]:+.1f}°, knee={angle_errors['back_right'][2]:+.1f}°)")
                print(f"     接地状態: FL={'接地' if contact_info['front_left']['is_contact'] else '浮上'}({contact_info['front_left']['contact_count']}点, {contact_info['front_left']['force']:.1f}N), "
                      f"FR={'接地' if contact_info['front_right']['is_contact'] else '浮上'}({contact_info['front_right']['contact_count']}点, {contact_info['front_right']['force']:.1f}N), "
                      f"BL={'接地' if contact_info['back_left']['is_contact'] else '浮上'}({contact_info['back_left']['contact_count']}点, {contact_info['back_left']['force']:.1f}N), "
                      f"BR={'接地' if contact_info['back_right']['is_contact'] else '浮上'}({contact_info['back_right']['contact_count']}点, {contact_info['back_right']['force']:.1f}N)")
                if all(pos is not None for pos in toe_positions.values()):
                    print(f"     足先位置: FL=({toe_positions['front_left'][0]:.3f}, {toe_positions['front_left'][1]:.3f}, {toe_positions['front_left'][2]:.3f} ({toe_height_diffs['front_left']:+.3f})), "
                          f"FR=({toe_positions['front_right'][0]:.3f}, {toe_positions['front_right'][1]:.3f}, {toe_positions['front_right'][2]:.3f} ({toe_height_diffs['front_right']:+.3f}))")
                    print(f"                BL=({toe_positions['back_left'][0]:.3f}, {toe_positions['back_left'][1]:.3f}, {toe_positions['back_left'][2]:.3f} ({toe_height_diffs['back_left']:+.3f})), "
                          f"BR=({toe_positions['back_right'][0]:.3f}, {toe_positions['back_right'][1]:.3f}, {toe_positions['back_right'][2]:.3f} ({toe_height_diffs['back_right']:+.3f}))")
                    print(f"     平均足先高さ: {avg_toe_height:.3f}m")
                else:
                    print(f"     足先位置: 取得失敗（一部のリンクが見つかりませんでした）")
                
                # 前回の値を更新
                prev_roll = current_roll
                prev_pitch = current_pitch
                prev_knee_angles = current_knee_angles.copy()
                prev_toe_heights = toe_heights.copy() if all(h is not None for h in toe_heights.values()) else None
                prev_base_pos = list(current_pos)
            
            # 初期角度（固定）と目標角度の間を線形補間
            interpolated_angles = {}
            for leg_name in leg_joints.keys():
                initial_angle = initial_standing_angles.get(leg_name, [0.0, 0.0, 0.5])  # 固定された初期角度を使用
                target_angle = standing_up_angles.get(leg_name, [0.0, 0.0, 0.5])
                interpolated_angles[leg_name] = [
                    initial_angle[0] + (target_angle[0] - initial_angle[0]) * progress,
                    initial_angle[1] + (target_angle[1] - initial_angle[1]) * progress,
                    initial_angle[2] + (target_angle[2] - initial_angle[2]) * progress
                ]
            
            # standing_anglesを更新（段階的に）
            standing_angles = interpolated_angles
        
        # 姿勢フィードバックに基づいてジョイント角度を調整
        # rollが負（左に傾く）場合、左側の脚のabductionを増やす
        roll_error = current_roll  # 目標roll=0度
        pitch_error = current_pitch  # 目標pitch=0度
        
        # 各ジョイントを目標角度に制御（PD制御と姿勢フィードバック制御）
        for leg_name, joint_indices in leg_joints.items():
            base_angles = standing_angles.get(leg_name, [0.0, 0.0, 1.5])
            adjusted_angles = list(base_angles)
            
            # 姿勢フィードバックによる角度調整
            abduction_idx = 0
            hip_idx = 1
            
            # 立ち上がり中かどうかを判定（複数箇所で使用するため先に定義）
            is_standing_up_phase = (stability_confirmed and standing_up_start_step is not None and 
                                   i - standing_up_start_step >= 0 and 
                                   i - standing_up_start_step < standing_up_duration)
            
            # roll誤差に基づいてabductionを調整（左右バランス）
            # 立ち上がり中はrollフィードバックゲインを増やして左右バランスを強化
            roll_gain_multiplier = 2.0 if is_standing_up_phase else 1.0  # 立ち上がり中は2倍に
            
            if leg_name in ['front_left', 'back_left']:  # 左側の脚
                # rollが負（左に傾く）場合、左側の脚を外側に広げる
                adjusted_angles[abduction_idx] += roll_error * roll_feedback_gain * roll_gain_multiplier
            else:  # 右側の脚
                # rollが負（左に傾く）場合、右側の脚を内側に閉じる
                adjusted_angles[abduction_idx] -= roll_error * roll_feedback_gain * roll_gain_multiplier
            
            # 位置フィードバック（X方向の移動を抑制 - 右前への傾きを防ぐ）
            # X方向に右へ移動している場合、右側の脚を内側に、左側の脚を外側に
            x_position_error = current_pos[0]  # 目標X=0
            if abs(x_position_error) > 0.05:  # 5cm以上移動した場合のみ調整
                if leg_name in ['front_left', 'back_left']:  # 左側の脚
                    # Xが正（右へ移動）の場合、左側の脚を外側に広げる
                    adjusted_angles[abduction_idx] += x_position_error * position_feedback_gain
                else:  # 右側の脚
                    # Xが正（右へ移動）の場合、右側の脚を内側に閉じる
                    adjusted_angles[abduction_idx] -= x_position_error * position_feedback_gain
            
            # pitch誤差に基づいてhipを調整（前後バランス）
            # 立ち上がり中は姿勢フィードバックゲインを増やして前のめりを抑制
            pitch_gain_multiplier = 2.0 if is_standing_up_phase else 1.0  # 立ち上がり中は2倍に
            
            if leg_name in ['front_left', 'front_right']:  # 前脚
                # pitchが正（前のめり）の場合、前脚のhipを後ろ向きに
                adjusted_angles[hip_idx] -= pitch_error * pitch_feedback_gain * pitch_gain_multiplier
            else:  # 後脚
                # pitchが正（前のめり）の場合、後脚のhipを前向きに
                adjusted_angles[hip_idx] += pitch_error * pitch_feedback_gain * pitch_gain_multiplier
            
            # ジョイント制御（PD制御パラメータ付き）
            # 立ち上がり開始後は、ずっと弱い力とゲインを維持
            is_standing_up_or_after = (stability_confirmed and standing_up_start_step is not None and 
                                      i - standing_up_start_step >= 0)
            
            if is_standing_up_or_after:
                # 立ち上がり開始後：弱い力とゲインを維持（安定性を優先）
                # 力とゲインを増やして目標角度に到達できるようにする
                current_force = 20.0  # 通常の10%（10.0→20.0に増加：目標角度に到達するため）
                current_position_gain = position_gain * 0.2  # 通常の20%（0.1→0.2に増加：目標角度に到達するため）
                current_velocity_gain = velocity_gain * 0.2  # 通常の20%（0.1→0.2に増加：目標角度に到達するため）
            else:
                # 立ち上がり前：通常の力とゲイン
                current_force = 200.0
                current_position_gain = position_gain
                current_velocity_gain = velocity_gain
            
            for j, joint_idx in enumerate(joint_indices):
                try:
                    # 現在のジョイント速度を取得（PD制御用）
                    joint_state = p.getJointState(robot_id, joint_idx)
                    current_velocity = joint_state[1]
                    
                    p.setJointMotorControl2(
                        bodyIndex=robot_id,
                        jointIndex=joint_idx,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=adjusted_angles[j],
                        targetVelocity=0.0,  # 目標速度は0（静止）
                        force=current_force,
                        positionGain=current_position_gain,
                        velocityGain=current_velocity_gain
                    )
                except:
                    pass
        
        # 緊急時のみベースの位置と姿勢をリセット（PD制御と姿勢フィードバックで姿勢を維持）
        if i % 50 == 0:  # 50ステップごと（PD制御で姿勢を維持できるため、頻度を減らす）
            # 立ち上がり中はリセットの閾値を緩和（立ち上がりを妨げないようにする）
            is_standing_up_phase = (stability_confirmed and standing_up_start_step is not None and 
                                   i - standing_up_start_step >= 0 and 
                                   i - standing_up_start_step < standing_up_duration)
            
            # 位置が大きく移動した場合、姿勢が大きく傾いた場合、高さが大きく下がった場合にのみリセット
            pos_moved = abs(current_pos[0]) > 0.2 or abs(current_pos[1]) > 0.2  # X, Y方向に20cm以上移動
            height_low = current_pos[2] < target_height * 0.7  # 高さが30%以上下がった
            # 立ち上がり中は姿勢傾きの閾値を緩和（15度→25度）
            tilt_threshold = 25.0 if is_standing_up_phase else 15.0
            tilted = abs(current_pitch) > tilt_threshold or abs(current_roll) > tilt_threshold
            
            if pos_moved or height_low or tilted:
                reset_count += 1
                # 原因別にカウント
                if pos_moved:
                    reset_reasons_count['位置移動'] += 1
                if height_low:
                    reset_reasons_count['高さ低下'] += 1
                if tilted:
                    reset_reasons_count['姿勢傾き'] += 1
                
                # リセットの原因をログ出力（最初の10回のみ詳細表示）
                if reset_count <= 10:
                    reasons = []
                    if pos_moved:
                        reasons.append(f"位置移動(X={current_pos[0]:.3f}, Y={current_pos[1]:.3f})")
                    if height_low:
                        reasons.append(f"高さ低下(Z={current_pos[2]:.3f})")
                    if tilted:
                        reasons.append(f"姿勢傾き(pitch={current_pitch:.1f}°, roll={current_roll:.1f}°)")
                    print(f"  🔄 リセット発生 #{reset_count} (ステップ{i}): {', '.join(reasons)}")
                
                new_pos = [0, 0, target_height]  # 常に原点に戻す
                new_orn = [0, 0, 0, 1]  # 水平姿勢
                p.resetBasePositionAndOrientation(robot_id, new_pos, new_orn)
                p.resetBaseVelocity(robot_id, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, 0])
        
        p.stepSimulation()
        time.sleep(time_step)
    
    # リセット統計を表示
    print(f"\n📊 リセット統計:")
    print(f"  総リセット回数: {reset_count}回 ({total_simulation_steps}ステップ中)")
    print(f"  原因別内訳:")
    for reason, count in reset_reasons_count.items():
        if count > 0:
            print(f"    - {reason}: {count}回")
    if reset_count == 0:
        print("  ✅ リセットは発生しませんでした")
    
    # 姿勢制御が終わった直後にカメラを設定
    pos, orn = p.getBasePositionAndOrientation(robot_id)
    p.resetDebugVisualizerCamera(
        cameraDistance=2.0,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=pos
    )
    
    # 現在の姿勢を確認
    euler = p.getEulerFromQuaternion(orn)
    print(f"\n📊 現在の姿勢:")
    print(f"  位置: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
    print(f"  姿勢: roll={math.degrees(euler[0]):.1f}°, pitch={math.degrees(euler[1]):.1f}°, yaw={math.degrees(euler[2]):.1f}°")
    
    # 膝上・膝下の角度を測定
    print(f"\n📐 脚のリンク角度を測定中...")
    
    # まず、リンク情報を確認
    print("📋 リンク情報を確認中...")
    num_links = p.getNumJoints(robot_id)  # リンク数はジョイント数と同じ
    for i in range(min(20, num_links)):
        link_state = p.getLinkState(robot_id, i)
        link_name = ""
        try:
            joint_info = p.getJointInfo(robot_id, i)
            link_name = joint_info[12].decode('utf-8') if joint_info[12] else f"link_{i}"
        except:
            link_name = f"link_{i}"
        print(f"  リンク {i}: {link_name}")
    
    # リンク情報を取得（膝上・膝下のリンクを特定）
    # Vision60の構造: 各脚には複数のリンクがある
    # リンク名から正しいリンクインデックスを取得する
    
    # まず、すべてのリンク名を取得してマッピングを作成
    link_name_to_index = {}
    num_links = p.getNumJoints(robot_id)
    for i in range(num_links):
        try:
            joint_info = p.getJointInfo(robot_id, i)
            link_name = joint_info[12].decode('utf-8') if joint_info[12] else f"link_{i}"
            link_name_to_index[link_name] = i
        except:
            pass
    
    leg_link_info = {}
    for leg_name, joint_indices in leg_joints.items():
        # 各ジョイントのリンク情報を取得
        hip_joint = joint_indices[1]
        knee_joint = joint_indices[2]
        
        # 脚のインデックスを取得（front_left=0, front_right=1, back_left=2, back_right=3）
        leg_index_map = {'front_left': 0, 'front_right': 1, 'back_left': 2, 'back_right': 3}
        leg_idx = leg_index_map[leg_name]
        
        # リンク名から正しいリンクインデックスを取得
        # upper0-3が膝上、lower0-3が膝下
        thigh_link_name = f"upper{leg_idx}"
        shank_link_name = f"lower{leg_idx}"
        
        # リンク名からインデックスを取得
        thigh_link = link_name_to_index.get(thigh_link_name, hip_joint)
        shank_link = link_name_to_index.get(shank_link_name, knee_joint)
        
        leg_link_info[leg_name] = {
            'thigh_link': thigh_link,
            'shank_link': shank_link,
            'hip_joint': hip_joint,
            'knee_joint': knee_joint,
            'thigh_link_name': thigh_link_name,
            'shank_link_name': shank_link_name
        }
    
    # 各脚の膝上・膝下の角度を計算
    print(f"\n📐 各脚の膝上・膝下の角度:")
    for leg_name, link_indices in leg_link_info.items():
        thigh_link = link_indices['thigh_link']
        shank_link = link_indices['shank_link']
        hip_joint = link_indices['hip_joint']
        knee_joint = link_indices['knee_joint']
        
        try:
            # リンク名を取得（既にleg_link_infoに保存されている）
            thigh_link_name = link_indices.get('thigh_link_name', f"link_{thigh_link}")
            shank_link_name = link_indices.get('shank_link_name', f"link_{shank_link}")
            
            # リンクの姿勢を取得
            thigh_state = p.getLinkState(robot_id, thigh_link)
            shank_state = p.getLinkState(robot_id, shank_link)
            
            # リンクの姿勢（クォータニオン）を取得
            thigh_orn = thigh_state[1]  # クォータニオン
            shank_orn = shank_state[1]  # クォータニオン
            
            # オイラー角に変換
            thigh_euler = p.getEulerFromQuaternion(thigh_orn)
            shank_euler = p.getEulerFromQuaternion(shank_orn)
            
            # 地面に対する角度を計算
            # pitch（前後方向の傾き）を使用
            # roll（左右方向の傾き）も確認
            
            # 膝上の角度（地面に対する傾き）
            # pitchが前後の傾き、rollが左右の傾き
            thigh_pitch = math.degrees(thigh_euler[1])  # 前後方向
            thigh_roll = math.degrees(thigh_euler[0])  # 左右方向
            
            # 膝下の角度（地面に対する傾き）
            shank_pitch = math.degrees(shank_euler[1])  # 前後方向
            shank_roll = math.degrees(shank_euler[0])  # 左右方向
            
            # 膝上と膝下の相対角度（膝関節の角度）
            # ジョイント角度を直接取得
            knee_joint_state = p.getJointState(robot_id, knee_joint)
            knee_angle = math.degrees(knee_joint_state[0])  # ジョイント角度
            
            # ジョイント角度も表示
            hip_joint_state = p.getJointState(robot_id, hip_joint)
            hip_angle = math.degrees(hip_joint_state[0])
            abduction_joint_state = p.getJointState(robot_id, leg_joints[leg_name][0])
            abduction_angle = math.degrees(abduction_joint_state[0])
            
            print(f"  {leg_name}:")
            print(f"    ジョイント角度: abduction={abduction_angle:.1f}°, hip={hip_angle:.1f}°, knee={knee_angle:.1f}°")
            print(f"    膝上リンク({thigh_link_name}): pitch={thigh_pitch:.1f}°, roll={thigh_roll:.1f}°")
            print(f"    膝下リンク({shank_link_name}): pitch={shank_pitch:.1f}°, roll={shank_roll:.1f}°")
            
            # 逆関節の判定（膝下が後ろ向きに曲がっているか）
            # 実機では逆くの字になるので、膝下が後ろ向き（pitchが負）になるはず
            # また、膝関節角度が大きい（曲がっている）はず
            if shank_pitch < 0 and knee_angle > 90:
                print(f"    → ✅ 逆関節: 膝下が後ろ向き、膝が曲がっている（正常）")
            elif shank_pitch < 0:
                print(f"    → ⚠️ 膝下は後ろ向きだが、膝の角度が小さい（{knee_angle:.1f}°）")
            else:
                print(f"    → ❌ 警告: 膝下が前向き（逆L字の可能性）")
                
        except Exception as e:
            print(f"  {leg_name}: エラー - {e}")
            print(f"    リンク {thigh_link} または {shank_link} の取得に失敗しました")
    
    print("\n✅ ロボットの登場が完了しました。")
    print("   5秒間停止して終了します...")
    time.sleep(5)
    
    p.disconnect()
    print("✅ シミュレーションを終了しました。")

if __name__ == "__main__":
    main()
