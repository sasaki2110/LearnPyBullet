"""
Vision60（四足ロボット）の構造調査スクリプト

1. agents/p08_vision60の立ち上がり→安定化処理を実行
2. 安定化後に各モーターの確認を行う
"""
import pybullet as p
import pybullet_data
import time
import math
import sys
import os
from typing import Dict, List, Tuple, Optional

# agents/p08_vision60/my_agentをインポート可能にする
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'agents', 'p08_vision60', 'my_agent'))

# agents/p08_vision60のモジュールをインポート
from utils.config import config
from utils.pybullet_env import PyBulletEnvironment
from utils.robot_model import RobotModel
from utils.state import Vision60State
from utils.joint_control import JointController
from utils.standing_control import StandingController
from utils.stabilization_detector import StabilizationDetector
from utils.posture_control import PostureController
from utils.reset_handler import ResetHandler
from utils.logging_config import setup_logging, get_logger, get_log_level


def print_section(title: str):
    """セクション区切りを表示"""
    print("\n" + "=" * 80)
    print(f"  {title}")
    print("=" * 80)


def print_subsection(title: str):
    """サブセクション区切りを表示"""
    print("\n" + "-" * 80)
    print(f"  {title}")
    print("-" * 80)


def get_joint_type_name(joint_type: int) -> str:
    """ジョイントタイプを文字列に変換"""
    joint_types = {
        p.JOINT_REVOLUTE: "REVOLUTE (回転)",
        p.JOINT_PRISMATIC: "PRISMATIC (直動)",
        p.JOINT_SPHERICAL: "SPHERICAL (球)",
        p.JOINT_PLANAR: "PLANAR (平面)",
        p.JOINT_FIXED: "FIXED (固定)",
        p.JOINT_POINT2POINT: "POINT2POINT",
        p.JOINT_GEAR: "GEAR"
    }
    return joint_types.get(joint_type, f"UNKNOWN ({joint_type})")


def radians_to_degrees(rad: float) -> float:
    """ラジアンを度に変換"""
    return math.degrees(rad)


def degrees_to_radians(deg: float) -> float:
    """度をラジアンに変換"""
    return math.radians(deg)


def main():
    print_section("Vision60 構造調査スクリプト")
    
    # ロギングをセットアップ
    log_level = get_log_level()
    setup_logging(log_level=log_level, initialize=True)
    logger = get_logger('main')
    
    logger.info("🚀 Vision60エージェントの立ち上がり→安定化処理を開始します")
    
    # 環境初期化（agents/p08_vision60のモジュールを使用）
    env = PyBulletEnvironment()
    env.create_environment()
    env.load_plane()
    env.load_robot()
    env.setup_camera()
    env.print_joint_info()
    
    # ロボットモデル
    robot_model = RobotModel(env.robot_id)
    
    # 状態管理
    state = Vision60State(env.robot_id)
    
    # 各コントローラー
    joint_controller = JointController(robot_model, state)
    standing_controller = StandingController(robot_model, state)
    stabilization_detector = StabilizationDetector(robot_model, state)
    posture_controller = PostureController(robot_model, state)
    reset_handler = ResetHandler(robot_model, state)
    
    # 初期姿勢を設定
    joint_controller.set_initial_pose()
    
    # 姿勢を安定化
    logger.info("⚙️ 姿勢を安定化中（脚を閉じた状態でそのまま放置）...")
    for _ in range(config.STABILITY_SETTLE_STEPS):
        p.stepSimulation()
        time.sleep(config.TIME_STEP)
    
    # 初期高さを記録
    state.update_initial_height()
    
    # 立ち上がり→安定化処理を実行
    logger.info("⚡ 立ち上がり→安定化処理を実行中...")
    max_steps = 5000  # 十分なステップ数を確保
    stabilization_detected = False
    
    for step in range(max_steps):
        # 安定確認
        standing_controller.check_stability(step)
        
        # 立ち上がり制御
        standing_controller.update_standing_angles(step, env.plane_id)
        is_standing_up = standing_controller.is_standing_up()
        
        # 安定化検知
        if stabilization_detector.check_stabilization(step):
            stabilization_detected = True
            logger.info(f"  ✅ 安定化が検知されました（ステップ{step}）")
            # 少し待機して安定化を確認
            for _ in range(100):
                p.stepSimulation()
                time.sleep(config.TIME_STEP)
            break
        
        # 姿勢フィードバック制御
        posture_controller.apply_posture_feedback(is_standing_up, False)
        
        # ジョイント制御
        joint_controller.apply_joint_control(is_standing_up)
        
        # リセットチェック
        reset_handler.check_and_reset(step, is_standing_up, False)
        
        # シミュレーションステップ
        p.stepSimulation()
        time.sleep(config.TIME_STEP)
    
    if not stabilization_detected:
        logger.warning("  ⚠️ 安定化が検知されませんでした。続行します...")
        # 少し待機
        for _ in range(200):
            p.stepSimulation()
            time.sleep(config.TIME_STEP)
    
    logger.info("✅ 立ち上がり→安定化処理が完了しました")
    
    # 少し待機（GUIが表示されるまで）
    for _ in range(10):
        p.stepSimulation()
        time.sleep(0.1)
    
    # ========================================================================
    # 1. ジョイントと骨格（リンク）を把握する
    # ========================================================================
    print_section("1. ジョイントと骨格（リンク）の把握（立ち上がり→安定化後の状態）")
    
    robot_id = env.robot_id
    num_joints = p.getNumJoints(robot_id)
    num_links = num_joints  # 通常、ジョイント数 = リンク数
    print(f"\n📊 総ジョイント数: {num_joints}")
    print(f"📊 総リンク数: {num_links} (ベースリンク含む)")
    
    # ジョイント情報を取得
    joints_info: List[Dict] = []
    links_info: List[Dict] = []
    
    print_subsection("全ジョイント情報")
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        joint_name = joint_info[1].decode('utf-8')
        joint_type = joint_info[2]
        parent_index = joint_info[16]  # 親リンクインデックス
        
        # リンク情報を取得
        link_state = p.getLinkState(robot_id, i)
        link_world_pos = link_state[0]
        link_world_orient = link_state[1]
        
        joint_data = {
            'index': i,
            'name': joint_name,
            'type': joint_type,
            'type_name': get_joint_type_name(joint_type),
            'parent_index': parent_index,
            'lower_limit': joint_info[8],
            'upper_limit': joint_info[9],
            'max_force': joint_info[10],
            'max_velocity': joint_info[11],
            'link_world_pos': link_world_pos,
            'link_world_orient': link_world_orient
        }
        joints_info.append(joint_data)
        
        print(f"\n  ジョイント {i}: {joint_name}")
        print(f"    タイプ: {joint_data['type_name']}")
        print(f"    親リンクインデックス: {parent_index}")
        print(f"    可動範囲: [{joint_data['lower_limit']:.3f}, {joint_data['upper_limit']:.3f}] rad")
        print(f"    最大力: {joint_data['max_force']:.1f} N")
        print(f"    最大速度: {joint_data['max_velocity']:.3f} rad/s")
        print(f"    リンク位置: ({link_world_pos[0]:.3f}, {link_world_pos[1]:.3f}, {link_world_pos[2]:.3f})")
    
    # ベースリンク情報を取得
    base_state = p.getBasePositionAndOrientation(robot_id)
    base_pos = base_state[0]
    base_orient = base_state[1]
    print_subsection("ベースリンク情報")
    print(f"  位置: ({base_pos[0]:.3f}, {base_pos[1]:.3f}, {base_pos[2]:.3f})")
    print(f"  姿勢（クォータニオン）: ({base_orient[0]:.3f}, {base_orient[1]:.3f}, {base_orient[2]:.3f}, {base_orient[3]:.3f})")
    
    # 脚の構造を特定（agents/p08_vision60のモジュールを使用）
    print_subsection("脚の構造の特定")
    leg_joints_map = robot_model.leg_joints  # agents/p08_vision60のモジュールから取得
    
    # 可動ジョイントのみをフィルタリング
    for leg_name in leg_joints_map.keys():
        leg_joints_map[leg_name] = [
            idx for idx in leg_joints_map[leg_name]
            if idx < num_joints and joints_info[idx]['type'] == p.JOINT_REVOLUTE
        ]
    
    # 脚の構造を表示
    joint_names = ['abduction', 'hip', 'knee']
    for leg_name, joint_indices in leg_joints_map.items():
        print(f"\n  {leg_name.upper()}:")
        if joint_indices:
            for i, idx in enumerate(joint_indices):
                joint_data = joints_info[idx]
                joint_role = joint_names[i] if i < len(joint_names) else 'unknown'
                print(f"    {joint_role}: ジョイント {idx} ({joint_data['name']}) - {joint_data['type_name']}")
                print(f"      可動範囲: [{radians_to_degrees(joint_data['lower_limit']):.1f}°, {radians_to_degrees(joint_data['upper_limit']):.1f}°]")
        else:
            print(f"    （未特定）")
    
    # ========================================================================
    # 2. ジョイントの稼働部を把握する
    # ========================================================================
    print_section("2. ジョイントの稼働部の把握")
    
    print_subsection("可動ジョイント（REVOLUTE）の一覧")
    revolute_joints = [j for j in joints_info if j['type'] == p.JOINT_REVOLUTE]
    print(f"\n  可動ジョイント数: {len(revolute_joints)}")
    
    for joint_data in revolute_joints:
        print(f"\n  ジョイント {joint_data['index']}: {joint_data['name']}")
        print(f"    可動範囲: [{radians_to_degrees(joint_data['lower_limit']):.1f}°, {radians_to_degrees(joint_data['upper_limit']):.1f}°]")
        print(f"    最大力: {joint_data['max_force']:.1f} N")
        print(f"    最大速度: {radians_to_degrees(joint_data['max_velocity']):.1f}°/s")
    
    # ========================================================================
    # 3. ジョイントの可動部を動かしてみて、それをログで確認する
    # ========================================================================
    print_section("3. ジョイントの可動部を動かして確認（立ち上がり→安定化後の状態から）")
    
    print_subsection("現在の状態の確認（立ち上がり→安定化後）")
    print("\n  各ジョイントの現在角度:")
    initial_angles: Dict[int, float] = {}
    for joint_data in revolute_joints:
        joint_idx = joint_data['index']
        joint_state = p.getJointState(robot_id, joint_idx)
        angle = joint_state[0]
        initial_angles[joint_idx] = angle
        print(f"    ジョイント {joint_idx} ({joint_data['name']}): {radians_to_degrees(angle):.2f}° ({angle:.4f} rad)")
    
    # 現在の姿勢も表示
    base_state = p.getBasePositionAndOrientation(robot_id)
    base_pos = base_state[0]
    base_orient = base_state[1]
    euler = p.getEulerFromQuaternion(base_orient)
    print(f"\n  現在のベース位置: ({base_pos[0]:.3f}, {base_pos[1]:.3f}, {base_pos[2]:.3f})")
    print(f"  現在のベース姿勢: roll={radians_to_degrees(euler[0]):.2f}°, pitch={radians_to_degrees(euler[1]):.2f}°, yaw={radians_to_degrees(euler[2]):.2f}°")
    
    # 各脚のジョイントを順番に動かして確認
    print_subsection("各脚のジョイントを動かして確認")
    print("  ⚠️ 注意: 物理シミュレーションとして正しい動作を確認するため、")
    print("     動かすジョイント以外は制御を無効化します（姿勢維持制御を停止）")
    
    test_angles = [
        degrees_to_radians(30),   # +30度
        degrees_to_radians(-30),  # -30度
        degrees_to_radians(0),     # 0度（初期位置に戻す）
    ]
    
    # すべての可動ジョイントのインデックスを取得
    all_revolute_joint_indices = [j['index'] for j in revolute_joints]
    
    for leg_name, joint_indices in leg_joints_map.items():
        if not joint_indices:
            continue
        
        print(f"\n  【{leg_name.upper()}】のテスト:")
        
        for joint_idx in joint_indices:
            joint_data = joints_info[joint_idx]
            if joint_data['type'] != p.JOINT_REVOLUTE:
                continue
            
            print(f"\n    ジョイント {joint_idx} ({joint_data['name']}) を動かします:")
            
            for test_angle in test_angles:
                # 動かすジョイント以外は、現在の角度を維持するようにPOSITION_CONTROLを設定
                # （姿勢維持制御（PD制御による自動調整）は停止するが、現在の姿勢は維持される）
                for other_joint_idx in all_revolute_joint_indices:
                    if other_joint_idx != joint_idx:
                        # 現在の角度を取得
                        other_joint_state = p.getJointState(robot_id, other_joint_idx)
                        current_other_angle = other_joint_state[0]
                        # 現在の角度を維持
                        p.setJointMotorControl2(
                            robot_id,
                            other_joint_idx,
                            p.POSITION_CONTROL,
                            targetPosition=current_other_angle,
                            force=50.0,
                            positionGain=0.5,
                            velocityGain=1.0
                        )
                
                # 動かすジョイントだけをPOSITION_CONTROLで制御
                p.setJointMotorControl2(
                    robot_id,
                    joint_idx,
                    p.POSITION_CONTROL,
                    targetPosition=test_angle,
                    force=50.0,
                    positionGain=0.5,
                    velocityGain=1.0
                )
                
                # シミュレーションを進める
                for step in range(100):
                    p.stepSimulation()
                    time.sleep(0.01)
                
                # 現在の角度を取得
                joint_state = p.getJointState(robot_id, joint_idx)
                current_angle = joint_state[0]
                current_velocity = joint_state[1]
                
                # リンクの位置を取得
                link_state = p.getLinkState(robot_id, joint_idx)
                link_pos = link_state[0]
                
                # ベースの姿勢も確認（傾きを確認）
                base_state = p.getBasePositionAndOrientation(robot_id)
                base_pos = base_state[0]
                base_orient = base_state[1]
                euler = p.getEulerFromQuaternion(base_orient)
                
                print(f"      目標角度: {radians_to_degrees(test_angle):.1f}° → "
                      f"現在角度: {radians_to_degrees(current_angle):.2f}° "
                      f"(速度: {radians_to_degrees(current_velocity):.2f}°/s)")
                print(f"      リンク位置: ({link_pos[0]:.3f}, {link_pos[1]:.3f}, {link_pos[2]:.3f})")
                print(f"      ベース姿勢: roll={radians_to_degrees(euler[0]):.2f}°, pitch={radians_to_degrees(euler[1]):.2f}°, yaw={radians_to_degrees(euler[2]):.2f}°")
            
            # 初期位置に戻す（他のジョイントは現在の角度を維持）
            for other_joint_idx in all_revolute_joint_indices:
                if other_joint_idx != joint_idx:
                    # 現在の角度を取得
                    other_joint_state = p.getJointState(robot_id, other_joint_idx)
                    current_other_angle = other_joint_state[0]
                    # 現在の角度を維持
                    p.setJointMotorControl2(
                        robot_id,
                        other_joint_idx,
                        p.POSITION_CONTROL,
                        targetPosition=current_other_angle,
                        force=50.0,
                        positionGain=0.5,
                        velocityGain=1.0
                    )
            
            p.setJointMotorControl2(
                robot_id,
                joint_idx,
                p.POSITION_CONTROL,
                targetPosition=initial_angles[joint_idx],
                force=50.0,
                positionGain=0.5,
                velocityGain=1.0
            )
            
            # シミュレーションを進める
            for step in range(100):
                p.stepSimulation()
                time.sleep(0.01)
    
    # 左右対称性の確認
    print_subsection("左右対称性の確認（立ち上がり→安定化後の状態から）")
    print("  ⚠️ 注意: 物理シミュレーションとして正しい動作を確認するため、")
    print("     動かすジョイント以外は制御を無効化します（姿勢維持制御を停止）")
    
    # 左右の対応するジョイントを特定（前脚と後脚を別々に）
    print("\n  【前脚の左右対称性確認】")
    front_left_joints = leg_joints_map['front_left']
    front_right_joints = leg_joints_map['front_right']
    
    print(f"  左前脚ジョイント: {front_left_joints} (abduction, hip, knee)")
    print(f"  右前脚ジョイント: {front_right_joints} (abduction, hip, knee)")
    
    # 対応するジョイントを動かして比較
    if len(front_left_joints) == len(front_right_joints):
        print("\n  左右対応するジョイントを同じ角度で動かして比較:")
        
        for i, (left_idx, right_idx) in enumerate(zip(front_left_joints, front_right_joints)):
            left_joint = joints_info[left_idx]
            right_joint = joints_info[right_idx]
            joint_role = joint_names[i] if i < len(joint_names) else 'unknown'
            
            print(f"\n    【{joint_role.upper()}】ジョイント:")
            print(f"      左: ジョイント {left_idx} ({left_joint['name']})")
            print(f"      右: ジョイント {right_idx} ({right_joint['name']})")
            
            # 同じ角度で動かす（可動範囲内）
            if joint_role == 'abduction':
                test_angle = degrees_to_radians(20)  # abductionは±24.6度まで
            else:
                test_angle = degrees_to_radians(30)
            
            # 初期位置を記録
            left_initial_link_state = p.getLinkState(robot_id, left_idx)
            right_initial_link_state = p.getLinkState(robot_id, right_idx)
            left_initial_pos = left_initial_link_state[0]
            right_initial_pos = right_initial_link_state[0]
            
            # ベースの初期姿勢も記録
            base_initial_state = p.getBasePositionAndOrientation(robot_id)
            base_initial_pos = base_initial_state[0]
            base_initial_orient = base_initial_state[1]
            base_initial_euler = p.getEulerFromQuaternion(base_initial_orient)
            
            # 動かすジョイント以外は、現在の角度を維持するようにPOSITION_CONTROLを設定
            # （姿勢維持制御（PD制御による自動調整）は停止するが、現在の姿勢は維持される）
            for other_joint_idx in all_revolute_joint_indices:
                if other_joint_idx != left_idx and other_joint_idx != right_idx:
                    # 現在の角度を取得
                    other_joint_state = p.getJointState(robot_id, other_joint_idx)
                    current_other_angle = other_joint_state[0]
                    # 現在の角度を維持
                    p.setJointMotorControl2(
                        robot_id,
                        other_joint_idx,
                        p.POSITION_CONTROL,
                        targetPosition=current_other_angle,
                        force=50.0,
                        positionGain=0.5,
                        velocityGain=1.0
                    )
            
            # 左側
            p.setJointMotorControl2(
                robot_id,
                left_idx,
                p.POSITION_CONTROL,
                targetPosition=test_angle,
                force=50.0,
                positionGain=0.5,
                velocityGain=1.0
            )
            
            # 右側
            p.setJointMotorControl2(
                robot_id,
                right_idx,
                p.POSITION_CONTROL,
                targetPosition=test_angle,
                force=50.0,
                positionGain=0.5,
                velocityGain=1.0
            )
            
            # シミュレーションを進める
            for step in range(150):
                p.stepSimulation()
                time.sleep(0.01)
            
            # 結果を確認
            left_state = p.getJointState(robot_id, left_idx)
            right_state = p.getJointState(robot_id, right_idx)
            left_angle = left_state[0]
            right_angle = right_state[0]
            
            left_link_state = p.getLinkState(robot_id, left_idx)
            right_link_state = p.getLinkState(robot_id, right_idx)
            left_link_pos = left_link_state[0]
            right_link_pos = right_link_state[0]
            
            # ベースの姿勢変化も確認
            base_current_state = p.getBasePositionAndOrientation(robot_id)
            base_current_pos = base_current_state[0]
            base_current_orient = base_current_state[1]
            base_current_euler = p.getEulerFromQuaternion(base_current_orient)
            
            # 移動量を計算
            left_movement = (
                left_link_pos[0] - left_initial_pos[0],
                left_link_pos[1] - left_initial_pos[1],
                left_link_pos[2] - left_initial_pos[2]
            )
            right_movement = (
                right_link_pos[0] - right_initial_pos[0],
                right_link_pos[1] - right_initial_pos[1],
                right_link_pos[2] - right_initial_pos[2]
            )
            
            # ベースの姿勢変化
            base_roll_change = radians_to_degrees(base_current_euler[0] - base_initial_euler[0])
            base_pitch_change = radians_to_degrees(base_current_euler[1] - base_initial_euler[1])
            base_yaw_change = radians_to_degrees(base_current_euler[2] - base_initial_euler[2])
            
            print(f"      目標角度: {radians_to_degrees(test_angle):.1f}°")
            print(f"      左側: 角度={radians_to_degrees(left_angle):.2f}°, "
                  f"移動=({left_movement[0]:.3f}, {left_movement[1]:.3f}, {left_movement[2]:.3f})")
            print(f"      右側: 角度={radians_to_degrees(right_angle):.2f}°, "
                  f"移動=({right_movement[0]:.3f}, {right_movement[1]:.3f}, {right_movement[2]:.3f})")
            
            # 移動方向の比較
            print(f"      → 同じ角度で動かした時の移動方向:")
            print(f"         左側: X={left_movement[0]:+.3f}, Y={left_movement[1]:+.3f}, Z={left_movement[2]:+.3f}")
            print(f"         右側: X={right_movement[0]:+.3f}, Y={right_movement[1]:+.3f}, Z={right_movement[2]:+.3f}")
            print(f"      → ベース姿勢変化: roll={base_roll_change:+.2f}°, pitch={base_pitch_change:+.2f}°, yaw={base_yaw_change:+.2f}°")
            
            # 符号が反転しているか確認
            if abs(left_movement[0]) > 0.001 and abs(right_movement[0]) > 0.001:
                if (left_movement[0] > 0) != (right_movement[0] > 0):
                    print(f"      ⚠️ X方向の符号が反転しています（左右で異なる方向に動いています）")
            if abs(left_movement[1]) > 0.001 and abs(right_movement[1]) > 0.001:
                if (left_movement[1] > 0) != (right_movement[1] > 0):
                    print(f"      ⚠️ Y方向の符号が反転しています（左右で異なる方向に動いています）")
            if abs(left_movement[2]) > 0.001 and abs(right_movement[2]) > 0.001:
                if (left_movement[2] > 0) != (right_movement[2] > 0):
                    print(f"      ⚠️ Z方向の符号が反転しています（左右で異なる方向に動いています）")
            
            # 初期位置に戻す（他のジョイントは現在の角度を維持）
            for other_joint_idx in all_revolute_joint_indices:
                if other_joint_idx != left_idx and other_joint_idx != right_idx:
                    # 現在の角度を取得
                    other_joint_state = p.getJointState(robot_id, other_joint_idx)
                    current_other_angle = other_joint_state[0]
                    # 現在の角度を維持
                    p.setJointMotorControl2(
                        robot_id,
                        other_joint_idx,
                        p.POSITION_CONTROL,
                        targetPosition=current_other_angle,
                        force=50.0,
                        positionGain=0.5,
                        velocityGain=1.0
                    )
            
            p.setJointMotorControl2(
                robot_id,
                left_idx,
                p.POSITION_CONTROL,
                targetPosition=initial_angles[left_idx],
                force=50.0,
                positionGain=0.5,
                velocityGain=1.0
            )
            p.setJointMotorControl2(
                robot_id,
                right_idx,
                p.POSITION_CONTROL,
                targetPosition=initial_angles[right_idx],
                force=50.0,
                positionGain=0.5,
                velocityGain=1.0
            )
            
            # シミュレーションを進める
            for step in range(150):
                p.stepSimulation()
                time.sleep(0.01)
    
    # 後脚も同様に確認
    print("\n  【後脚の左右対称性確認】")
    back_left_joints = leg_joints_map['back_left']
    back_right_joints = leg_joints_map['back_right']
    
    print(f"  左後脚ジョイント: {back_left_joints} (abduction, hip, knee)")
    print(f"  右後脚ジョイント: {back_right_joints} (abduction, hip, knee)")
    
    if len(back_left_joints) == len(back_right_joints):
        print("\n  左右対応するジョイントを同じ角度で動かして比較:")
        
        for i, (left_idx, right_idx) in enumerate(zip(back_left_joints, back_right_joints)):
            left_joint = joints_info[left_idx]
            right_joint = joints_info[right_idx]
            joint_role = joint_names[i] if i < len(joint_names) else 'unknown'
            
            print(f"\n    【{joint_role.upper()}】ジョイント:")
            print(f"      左: ジョイント {left_idx} ({left_joint['name']})")
            print(f"      右: ジョイント {right_idx} ({right_joint['name']})")
            
            # 同じ角度で動かす
            if joint_role == 'abduction':
                test_angle = degrees_to_radians(20)
            else:
                test_angle = degrees_to_radians(30)
            
            # 初期位置を記録
            left_initial_link_state = p.getLinkState(robot_id, left_idx)
            right_initial_link_state = p.getLinkState(robot_id, right_idx)
            left_initial_pos = left_initial_link_state[0]
            right_initial_pos = right_initial_link_state[0]
            
            # ベースの初期姿勢も記録
            base_initial_state = p.getBasePositionAndOrientation(robot_id)
            base_initial_pos = base_initial_state[0]
            base_initial_orient = base_initial_state[1]
            base_initial_euler = p.getEulerFromQuaternion(base_initial_orient)
            
            # 動かすジョイント以外は、現在の角度を維持するようにPOSITION_CONTROLを設定
            # （姿勢維持制御（PD制御による自動調整）は停止するが、現在の姿勢は維持される）
            for other_joint_idx in all_revolute_joint_indices:
                if other_joint_idx != left_idx and other_joint_idx != right_idx:
                    # 現在の角度を取得
                    other_joint_state = p.getJointState(robot_id, other_joint_idx)
                    current_other_angle = other_joint_state[0]
                    # 現在の角度を維持
                    p.setJointMotorControl2(
                        robot_id,
                        other_joint_idx,
                        p.POSITION_CONTROL,
                        targetPosition=current_other_angle,
                        force=50.0,
                        positionGain=0.5,
                        velocityGain=1.0
                    )
            
            # 左側
            p.setJointMotorControl2(
                robot_id,
                left_idx,
                p.POSITION_CONTROL,
                targetPosition=test_angle,
                force=50.0,
                positionGain=0.5,
                velocityGain=1.0
            )
            
            # 右側
            p.setJointMotorControl2(
                robot_id,
                right_idx,
                p.POSITION_CONTROL,
                targetPosition=test_angle,
                force=50.0,
                positionGain=0.5,
                velocityGain=1.0
            )
            
            # シミュレーションを進める
            for step in range(150):
                p.stepSimulation()
                time.sleep(0.01)
            
            # 結果を確認
            left_state = p.getJointState(robot_id, left_idx)
            right_state = p.getJointState(robot_id, right_idx)
            left_angle = left_state[0]
            right_angle = right_state[0]
            
            left_link_state = p.getLinkState(robot_id, left_idx)
            right_link_state = p.getLinkState(robot_id, right_idx)
            left_link_pos = left_link_state[0]
            right_link_pos = right_link_state[0]
            
            # ベースの姿勢変化も確認
            base_current_state = p.getBasePositionAndOrientation(robot_id)
            base_current_pos = base_current_state[0]
            base_current_orient = base_current_state[1]
            base_current_euler = p.getEulerFromQuaternion(base_current_orient)
            
            # 移動量を計算
            left_movement = (
                left_link_pos[0] - left_initial_pos[0],
                left_link_pos[1] - left_initial_pos[1],
                left_link_pos[2] - left_initial_pos[2]
            )
            right_movement = (
                right_link_pos[0] - right_initial_pos[0],
                right_link_pos[1] - right_initial_pos[1],
                right_link_pos[2] - right_initial_pos[2]
            )
            
            # ベースの姿勢変化
            base_roll_change = radians_to_degrees(base_current_euler[0] - base_initial_euler[0])
            base_pitch_change = radians_to_degrees(base_current_euler[1] - base_initial_euler[1])
            base_yaw_change = radians_to_degrees(base_current_euler[2] - base_initial_euler[2])
            
            print(f"      目標角度: {radians_to_degrees(test_angle):.1f}°")
            print(f"      左側: 角度={radians_to_degrees(left_angle):.2f}°, "
                  f"移動=({left_movement[0]:.3f}, {left_movement[1]:.3f}, {left_movement[2]:.3f})")
            print(f"      右側: 角度={radians_to_degrees(right_angle):.2f}°, "
                  f"移動=({right_movement[0]:.3f}, {right_movement[1]:.3f}, {right_movement[2]:.3f})")
            
            # 移動方向の比較
            print(f"      → 同じ角度で動かした時の移動方向:")
            print(f"         左側: X={left_movement[0]:+.3f}, Y={left_movement[1]:+.3f}, Z={left_movement[2]:+.3f}")
            print(f"         右側: X={right_movement[0]:+.3f}, Y={right_movement[1]:+.3f}, Z={right_movement[2]:+.3f}")
            print(f"      → ベース姿勢変化: roll={base_roll_change:+.2f}°, pitch={base_pitch_change:+.2f}°, yaw={base_yaw_change:+.2f}°")
            
            # 符号が反転しているか確認
            if abs(left_movement[0]) > 0.001 and abs(right_movement[0]) > 0.001:
                if (left_movement[0] > 0) != (right_movement[0] > 0):
                    print(f"      ⚠️ X方向の符号が反転しています（左右で異なる方向に動いています）")
            if abs(left_movement[1]) > 0.001 and abs(right_movement[1]) > 0.001:
                if (left_movement[1] > 0) != (right_movement[1] > 0):
                    print(f"      ⚠️ Y方向の符号が反転しています（左右で異なる方向に動いています）")
            if abs(left_movement[2]) > 0.001 and abs(right_movement[2]) > 0.001:
                if (left_movement[2] > 0) != (right_movement[2] > 0):
                    print(f"      ⚠️ Z方向の符号が反転しています（左右で異なる方向に動いています）")
            
            # 初期位置に戻す（他のジョイントは現在の角度を維持）
            for other_joint_idx in all_revolute_joint_indices:
                if other_joint_idx != left_idx and other_joint_idx != right_idx:
                    # 現在の角度を取得
                    other_joint_state = p.getJointState(robot_id, other_joint_idx)
                    current_other_angle = other_joint_state[0]
                    # 現在の角度を維持
                    p.setJointMotorControl2(
                        robot_id,
                        other_joint_idx,
                        p.POSITION_CONTROL,
                        targetPosition=current_other_angle,
                        force=50.0,
                        positionGain=0.5,
                        velocityGain=1.0
                    )
            
            p.setJointMotorControl2(
                robot_id,
                left_idx,
                p.POSITION_CONTROL,
                targetPosition=initial_angles[left_idx],
                force=50.0,
                positionGain=0.5,
                velocityGain=1.0
            )
            p.setJointMotorControl2(
                robot_id,
                right_idx,
                p.POSITION_CONTROL,
                targetPosition=initial_angles[right_idx],
                force=50.0,
                positionGain=0.5,
                velocityGain=1.0
            )
            
            # シミュレーションを進める
            for step in range(150):
                p.stepSimulation()
                time.sleep(0.01)
    
    print_section("調査完了")
    print("\n調査が完了しました。GUIで確認してください。")
    print("Enterキーを押すと終了します...")
    
    # 終了待機
    input()
    
    env.disconnect()
    logger.info("✅ 構造調査スクリプトの実行が完了しました")


if __name__ == "__main__":
    main()
