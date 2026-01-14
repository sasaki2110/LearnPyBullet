import pybullet as p
import pybullet_data
import time

def print_joint_info(robot_id, joint_index):
    """ジョイント情報を詳細に表示"""
    joint_info = p.getJointInfo(robot_id, joint_index)
    
    # ジョイントタイプの名前マッピング
    joint_type_names = {
        p.JOINT_REVOLUTE: "回転関節 (REVOLUTE)",
        p.JOINT_PRISMATIC: "直動関節 (PRISMATIC)",
        p.JOINT_SPHERICAL: "球関節 (SPHERICAL)",
        p.JOINT_PLANAR: "平面関節 (PLANAR)",
        p.JOINT_FIXED: "固定関節 (FIXED)",
        p.JOINT_POINT2POINT: "点対点関節 (POINT2POINT)",
        p.JOINT_GEAR: "ギア関節 (GEAR)"
    }
    
    joint_type = joint_info[2]
    joint_type_name = joint_type_names.get(joint_type, f"不明 ({joint_type})")
    
    print(f"\n  ジョイント {joint_index}:")
    print(f"    名前: {joint_info[1].decode('utf-8')}")
    print(f"    タイプ: {joint_type_name}")
    print(f"    可動範囲: [{joint_info[8]:.3f}, {joint_info[9]:.3f}]")
    print(f"    最大力: {joint_info[10]:.1f}")
    print(f"    最大速度: {joint_info[11]:.3f}")
    print(f"    親リンク: {joint_info[16]}")
    print(f"    子リンク: {joint_index}")  # ジョイントiはリンクiに接続

def print_link_info(robot_id, link_index):
    """リンク情報を表示"""
    try:
        link_state = p.getLinkState(robot_id, link_index)
        link_name = p.getJointInfo(robot_id, link_index)[12].decode('utf-8') if link_index < p.getNumJoints(robot_id) else "ベース"
        
        print(f"\n  リンク {link_index}:")
        print(f"    名前: {link_name}")
        print(f"    重心位置: [{link_state[0][0]:.3f}, {link_state[0][1]:.3f}, {link_state[0][2]:.3f}]")
        print(f"    姿勢（クォータニオン）: [{link_state[1][0]:.3f}, {link_state[1][1]:.3f}, {link_state[1][2]:.3f}, {link_state[1][3]:.3f}]")
    except:
        print(f"\n  リンク {link_index}: 情報取得不可")

def print_body_info(body_id):
    """ボディ情報を表示"""
    body_info = p.getBodyInfo(body_id)
    base_pos, base_orn = p.getBasePositionAndOrientation(body_id)
    
    print(f"\nボディ {body_id}:")
    print(f"  名前: {body_info[1].decode('utf-8')}")
    print(f"  ベース位置: [{base_pos[0]:.3f}, {base_pos[1]:.3f}, {base_pos[2]:.3f}]")
    print(f"  ベース姿勢: [{base_orn[0]:.3f}, {base_orn[1]:.3f}, {base_orn[2]:.3f}, {base_orn[3]:.3f}]")

def analyze_robot_structure(robot_id):
    """ロボットの階層構造を分析して表示"""
    print("\n" + "="*60)
    print("### 2.1 ロボットの階層構造の理解")
    print("="*60)
    
    # ボディ情報
    print("\n【ボディ情報】")
    print_body_info(robot_id)
    
    # ジョイント数とリンク数の取得
    num_joints = p.getNumJoints(robot_id)
    print(f"\n【基本情報】")
    print(f"  ジョイント数: {num_joints}")
    print(f"  リンク数: {num_joints} (リンク数 = ジョイント数)")
    print(f"  ベース: インデックス -1 (すべてのロボットのルート親)")
    
    # ジョイント情報の詳細表示
    print(f"\n【ジョイント情報】")
    for i in range(num_joints):
        print_joint_info(robot_id, i)
    
    # リンク情報の表示
    print(f"\n【リンク情報】")
    print("  ベース (リンク -1):")
    base_pos, base_orn = p.getBasePositionAndOrientation(robot_id)
    print(f"    位置: [{base_pos[0]:.3f}, {base_pos[1]:.3f}, {base_pos[2]:.3f}]")
    
    for i in range(num_joints):
        print_link_info(robot_id, i)
    
    # 階層構造の可視化
    print(f"\n【階層構造（親子関係）】")
    print("  ベース (-1)")
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        parent_index = joint_info[16]
        joint_name = joint_info[1].decode('utf-8')
        indent = "  " * (i + 1)
        print(f"  {indent}└─ ジョイント {i} ({joint_name})")
        print(f"  {indent}    └─ リンク {i} (親: {parent_index})")
    
    print("\n" + "="*60)

def demonstrate_joint_range_of_motion(robot_id, time_step):
    """各ジョイントの可動域を実際に動かして可視化"""
    print("\n" + "="*60)
    print("### 可動域の可視化デモンストレーション")
    print("="*60)
    
    num_joints = p.getNumJoints(robot_id)
    
    # 可動可能なジョイント（回転関節と直動関節）を抽出
    movable_joints = []
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        joint_type = joint_info[2]
        joint_name = joint_info[1].decode('utf-8')
        lower_limit = joint_info[8]
        upper_limit = joint_info[9]
        
        # 回転関節または直動関節で、可動範囲が有効なもの
        if (joint_type == p.JOINT_REVOLUTE or joint_type == p.JOINT_PRISMATIC) and lower_limit < upper_limit:
            movable_joints.append({
                'index': i,
                'name': joint_name,
                'type': '回転' if joint_type == p.JOINT_REVOLUTE else '直動',
                'lower': lower_limit,
                'upper': upper_limit
            })
    
    print(f"\n可動可能なジョイント数: {len(movable_joints)}")
    print("\n各ジョイントを順番に可動範囲の限界まで動かします...")
    print("（各ジョイントで3秒間、下限→上限→中央の順に動かします）\n")
    
    # 各ジョイントを順番に動かす
    for joint_data in movable_joints:
        joint_idx = joint_data['index']
        joint_name = joint_data['name']
        joint_type = joint_data['type']
        lower = joint_data['lower']
        upper = joint_data['upper']
        center = (lower + upper) / 2
        
        print(f"ジョイント {joint_idx} ({joint_name}) - {joint_type}関節")
        print(f"  可動範囲: [{lower:.3f}, {upper:.3f}] rad")
        
        # まず中央位置にリセット
        p.resetJointState(robot_id, joint_idx, targetValue=center, targetVelocity=0.0)
        
        # シミュレーションを少し進める（安定化のため）
        for _ in range(50):
            p.stepSimulation()
            time.sleep(time_step)
        
        # 下限まで動かす
        print(f"  → 下限位置 ({lower:.3f}) へ移動中...")
        for step in range(120):  # 約0.5秒間（240fps * 0.5秒）
            # 位置制御で下限へ
            p.setJointMotorControl2(
                bodyIndex=robot_id,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=lower,
                force=100.0
            )
            p.stepSimulation()
            time.sleep(time_step)
        
        # 上限まで動かす
        print(f"  → 上限位置 ({upper:.3f}) へ移動中...")
        for step in range(120):
            p.setJointMotorControl2(
                bodyIndex=robot_id,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=upper,
                force=100.0
            )
            p.stepSimulation()
            time.sleep(time_step)
        
        # 中央位置に戻す
        print(f"  → 中央位置 ({center:.3f}) に戻します...")
        for step in range(120):
            p.setJointMotorControl2(
                bodyIndex=robot_id,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=center,
                force=100.0
            )
            p.stepSimulation()
            time.sleep(time_step)
        
        print(f"  完了\n")
        
        # 次のジョイントに移る前に少し待つ
        time.sleep(0.5)
    
    print("="*60)
    print("可動域デモンストレーション完了！")
    print("="*60)

def main():
    # シミュレーション速度の設定
    # 標準速度: 1.0, 1/2倍速: 2.0, 2倍速: 0.5
    speed_factor = 2.0  # 1/2倍速（ゆっくり）
    time_step = (1.0 / 240.0) * speed_factor  # 物理シミュレーションの標準的な時間刻み
    
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
    
    # カメラ設定：正面からクローズアップ
    # ロボットのベース位置を取得
    robot_base_pos, _ = p.getBasePositionAndOrientation(robot_id)
    
    # カメラを正面からクローズアップに設定
    p.resetDebugVisualizerCamera(
        cameraDistance=1.6,        # カメラ距離（小さいほど近く、クローズアップ）
        cameraYaw=45,                # 水平方向の回転（0度=正面）
        cameraPitch=-40,            # 垂直方向の回転（-30度=少し上から見下ろす）
        cameraTargetPosition=robot_base_pos  # カメラが向くターゲット位置（ロボットのベース）
    )
    
    # ロボットの階層構造を分析
    analyze_robot_structure(robot_id)
    
    # すべてのボディ情報を表示
    print("\n【シミュレーション内の全ボディ情報】")
    num_bodies = p.getNumBodies()
    print(f"  総ボディ数: {num_bodies}")
    for i in range(num_bodies):
        body_id = p.getBodyUniqueId(i)
        print_body_info(body_id)

    # 可動域の可視化デモンストレーション
    # 各ジョイントを可動範囲の限界まで動かして可視化
    # スキップしたい場合は、以下の行をコメントアウトしてください
    demonstrate_joint_range_of_motion(robot_id, time_step)

    # 通常のシミュレーションループ（可動域デモの後）
    print("\n通常のシミュレーションループを開始します...")
    for i in range (1000):
        p.stepSimulation()
        time.sleep(time_step)
        
        if i % 200 == 0:
            # トレイの位置を取得して表示
            pos, _ = p.getBasePositionAndOrientation(tray_id)
            print(f"Step {i}: トレイの現在地 {pos}")

    p.disconnect()

if __name__ == "__main__":
    main()