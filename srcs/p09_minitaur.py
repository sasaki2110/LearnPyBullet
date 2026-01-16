"""
Minitaur（四足ロボット）の制御サンプル

PyBullet公式サンプルを参考にした、Minitaurロボットの制御例です。
- 基本的な立ち姿勢の制御
- シンプルな歩行パターンの実装
- モーター角度の可視化

参考:
- https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/minitaur.py
- https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/minitaur_evaluate.py
- https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/minitaur_test.py
"""
import pybullet as p
import pybullet_data
import numpy as np
import time
import math
from typing import List, Tuple, Optional


class MinitaurController:
    """Minitaurロボットの制御クラス"""
    
    def __init__(self, urdf_root_path: str = ''):
        """
        Minitaurコントローラーを初期化
        
        Args:
            urdf_root_path: URDFファイルのルートパス（空文字列の場合はpybullet_dataを使用）
        """
        self.urdf_root_path = urdf_root_path
        self.quadruped = None
        self.joint_name_to_id = {}
        self.motor_id_list = []
        
        # PD制御パラメータ
        self.kp = 1.0          # 位置ゲイン
        self.kd = 0.1          # 速度ゲイン
        self.max_force = 3.5   # 最大トルク
        
        # モーター数と方向
        self.n_motors = 8
        # モーター方向: 左側4つは-1、右側4つは+1
        self.motor_dir = np.array([-1, -1, -1, -1, 1, 1, 1, 1])
        
        # ロボットをリセット
        self.reset()
    
    def reset(self):
        """ロボットをリセット"""
        # URDFファイルのパスを設定
        if not self.urdf_root_path:
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            urdf_path = "quadruped/minitaur.urdf"
        else:
            urdf_path = f"{self.urdf_root_path}/quadruped/minitaur.urdf"
        
        # ロボットをロード（高さ0.2mの位置に配置）
        self.quadruped = p.loadURDF(urdf_path, 0, 0, 0.2)
        
        # ジョイント名とIDのマッピングを構築
        self._build_joint_name_to_id_dict()
        
        # モーターIDリストを構築
        self._build_motor_id_list()
        
        # 初期姿勢を設定
        self.reset_pose()
        
        # 物理シミュレーションを少し進めて安定化
        for _ in range(100):
            p.stepSimulation()
    
    def _build_joint_name_to_id_dict(self):
        """ジョイント名からIDへのマッピングを構築"""
        n_joints = p.getNumJoints(self.quadruped)
        self.joint_name_to_id = {}
        
        for i in range(n_joints):
            joint_info = p.getJointInfo(self.quadruped, i)
            joint_name = joint_info[1].decode('UTF-8')
            self.joint_name_to_id[joint_name] = i
    
    def _build_motor_id_list(self):
        """モーターIDリストを構築（8つのモーター）"""
        self.motor_id_list = [
            self.joint_name_to_id['motor_front_leftL_joint'],
            self.joint_name_to_id['motor_front_leftR_joint'],
            self.joint_name_to_id['motor_back_leftL_joint'],
            self.joint_name_to_id['motor_back_leftR_joint'],
            self.joint_name_to_id['motor_front_rightL_joint'],
            self.joint_name_to_id['motor_front_rightR_joint'],
            self.joint_name_to_id['motor_back_rightL_joint'],
            self.joint_name_to_id['motor_back_rightR_joint'],
        ]
    
    def reset_pose(self):
        """初期姿勢を設定（立ち姿勢）"""
        knee_friction_force = 0
        halfpi = math.pi / 2  # 90度
        knee_angle = -2.1834  # 膝の角度（計算された値）
        
        # 各脚を設定
        legs = [
            ('front_left', 0, 1),
            ('back_left', 2, 3),
            ('front_right', 4, 5),
            ('back_right', 6, 7),
        ]
        
        for leg_name, motor_l_idx, motor_r_idx in legs:
            # 左側モーター
            motor_l_name = f'motor_{leg_name}L_joint'
            knee_l_name = f'knee_{leg_name}L_link'
            p.resetJointState(
                self.quadruped,
                self.joint_name_to_id[motor_l_name],
                self.motor_dir[motor_l_idx] * halfpi
            )
            p.resetJointState(
                self.quadruped,
                self.joint_name_to_id[knee_l_name],
                self.motor_dir[motor_l_idx] * knee_angle
            )
            
            # 右側モーター
            motor_r_name = f'motor_{leg_name}R_joint'
            knee_r_name = f'knee_{leg_name}R_link'
            p.resetJointState(
                self.quadruped,
                self.joint_name_to_id[motor_r_name],
                self.motor_dir[motor_r_idx] * halfpi
            )
            p.resetJointState(
                self.quadruped,
                self.joint_name_to_id[knee_r_name],
                self.motor_dir[motor_r_idx] * knee_angle
            )
            
            # 膝のリンク間に制約を作成（左右の膝を同期）
            p.createConstraint(
                self.quadruped,
                self.joint_name_to_id[knee_r_name],
                self.quadruped,
                self.joint_name_to_id[knee_l_name],
                p.JOINT_POINT2POINT,
                [0, 0, 0],
                [0, 0.005, 0.2],
                [0, 0.01, 0.2]
            )
            
            # モーター角度を設定
            self.set_motor_angle_by_name(motor_l_name, self.motor_dir[motor_l_idx] * halfpi)
            self.set_motor_angle_by_name(motor_r_name, self.motor_dir[motor_r_idx] * halfpi)
            
            # 膝のリンクを固定（速度0で制御）
            p.setJointMotorControl2(
                bodyIndex=self.quadruped,
                jointIndex=self.joint_name_to_id[knee_l_name],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=0,
                force=knee_friction_force
            )
            p.setJointMotorControl2(
                bodyIndex=self.quadruped,
                jointIndex=self.joint_name_to_id[knee_r_name],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=0,
                force=knee_friction_force
            )
    
    def set_motor_angle_by_id(self, motor_id: int, desired_angle: float):
        """
        モーターIDを指定して角度を設定
        
        Args:
            motor_id: モーターID
            desired_angle: 目標角度（ラジアン）
        """
        p.setJointMotorControl2(
            bodyIndex=self.quadruped,
            jointIndex=motor_id,
            controlMode=p.POSITION_CONTROL,
            targetPosition=desired_angle,
            positionGain=self.kp,
            velocityGain=self.kd,
            force=self.max_force
        )
    
    def set_motor_angle_by_name(self, motor_name: str, desired_angle: float):
        """
        モーター名を指定して角度を設定
        
        Args:
            motor_name: モーター名
            desired_angle: 目標角度（ラジアン）
        """
        motor_id = self.joint_name_to_id[motor_name]
        self.set_motor_angle_by_id(motor_id, desired_angle)
    
    def apply_action(self, motor_commands: List[float]):
        """
        モーターコマンドを適用
        
        Args:
            motor_commands: 8つのモーターの目標角度リスト（ラジアン）
        """
        # モーター方向を考慮してコマンドを変換
        motor_commands_with_dir = np.multiply(motor_commands, self.motor_dir)
        
        for i in range(self.n_motors):
            self.set_motor_angle_by_id(self.motor_id_list[i], motor_commands_with_dir[i])
    
    def get_motor_angles(self) -> np.ndarray:
        """
        現在のモーター角度を取得
        
        Returns:
            8つのモーターの角度配列（ラジアン、モーター方向を考慮）
        """
        motor_angles = []
        for i in range(self.n_motors):
            joint_state = p.getJointState(self.quadruped, self.motor_id_list[i])
            motor_angles.append(joint_state[0])
        
        # モーター方向を考慮して変換
        motor_angles = np.multiply(motor_angles, self.motor_dir)
        return np.array(motor_angles)
    
    def get_motor_velocities(self) -> np.ndarray:
        """
        現在のモーター速度を取得
        
        Returns:
            8つのモーターの速度配列（rad/s、モーター方向を考慮）
        """
        motor_velocities = []
        for i in range(self.n_motors):
            joint_state = p.getJointState(self.quadruped, self.motor_id_list[i])
            motor_velocities.append(joint_state[1])
        
        # モーター方向を考慮して変換
        motor_velocities = np.multiply(motor_velocities, self.motor_dir)
        return np.array(motor_velocities)
    
    def get_motor_torques(self) -> np.ndarray:
        """
        現在のモータートルクを取得
        
        Returns:
            8つのモーターのトルク配列（N・m、モーター方向を考慮）
        """
        motor_torques = []
        for i in range(self.n_motors):
            joint_state = p.getJointState(self.quadruped, self.motor_id_list[i])
            motor_torques.append(joint_state[3])
        
        # モーター方向を考慮して変換
        motor_torques = np.multiply(motor_torques, self.motor_dir)
        return np.array(motor_torques)
    
    def get_base_position(self) -> Tuple[float, float, float]:
        """
        ベース（胴体）の位置を取得
        
        Returns:
            (x, y, z) 位置タプル
        """
        position, _ = p.getBasePositionAndOrientation(self.quadruped)
        return position
    
    def get_base_orientation(self) -> Tuple[float, float, float, float]:
        """
        ベース（胴体）の姿勢を取得
        
        Returns:
            (x, y, z, w) クォータニオンタプル
        """
        _, orientation = p.getBasePositionAndOrientation(self.quadruped)
        return orientation
    
    def is_fallen(self) -> bool:
        """
        ロボットが転倒しているかチェック
        
        Returns:
            True: 転倒している、False: 正常
        """
        orientation = self.get_base_orientation()
        rot_mat = p.getMatrixFromQuaternion(orientation)
        local_up = rot_mat[6:]  # 上方向ベクトル
        
        # 上方向とZ軸の内積が負の場合、転倒している
        return np.dot(np.array([0, 0, 1]), np.array(local_up)) < 0


def evaluate_desired_motorAngle_2Amplitude4Phase(step: int, params: List[float]) -> List[float]:
    """
    公式サンプルの2Amplitude4Phase歩行パターン
    
    公式サンプル: evaluate_desired_motorAngle_2Amplitude4Phase
    6つのパラメータを使用:
    - params[0]: 振幅1
    - params[1]: 振幅2
    - params[2]: 位相差
    - params[3]: 位相オフセット1
    - params[4]: 位相オフセット2
    - params[5]: 位相オフセット3
    
    Args:
        step: ステップ数
        params: パラメータリスト（6要素）
    
    Returns:
        8つのモーターの目標角度リスト（ラジアン）
    """
    speed = 0.35
    phase_diff = params[2]
    base_angle = 1.57  # π/2
    
    # 各モーターの角度を計算（公式サンプルと同じ計算式）
    a0 = math.sin(step * speed) * params[0] + base_angle
    a1 = math.sin(step * speed + phase_diff) * params[1] + base_angle
    a2 = math.sin(step * speed + params[3]) * params[0] + base_angle
    a3 = math.sin(step * speed + params[3] + phase_diff) * params[1] + base_angle
    a4 = math.sin(step * speed + params[4] + phase_diff) * params[1] + base_angle
    a5 = math.sin(step * speed + params[4]) * params[0] + base_angle
    a6 = math.sin(step * speed + params[5] + phase_diff) * params[1] + base_angle
    a7 = math.sin(step * speed + params[5]) * params[0] + base_angle
    
    return [a0, a1, a2, a3, a4, a5, a6, a7]


def evaluate_desired_motorAngle_hop(step: int, params: List[float]) -> List[float]:
    """
    ホップ（跳躍）パターン
    
    公式サンプル: evaluate_desired_motorAngle_hop
    2つのパラメータを使用:
    - params[0]: 振幅
    - params[1]: 速度
    
    Args:
        step: ステップ数
        params: パラメータリスト（2要素）
    
    Returns:
        8つのモーターの目標角度リスト（ラジアン）
    """
    amplitude = params[0]
    speed = params[1]
    base_angle = 1.57  # π/2
    
    a1 = math.sin(step * speed) * amplitude + base_angle
    a2 = math.sin(step * speed + 3.14) * amplitude + base_angle
    
    return [a1, base_angle, a2, base_angle, base_angle, a1, base_angle, a2]


def simple_walking_pattern(step: int, speed: float = 0.35, amplitude: float = 0.3) -> List[float]:
    """
    シンプルな歩行パターンを生成（公式サンプルの2Amplitude4Phaseを使用）
    
    Args:
        step: ステップ数
        speed: 歩行速度（未使用、互換性のため）
        amplitude: 振幅（未使用、互換性のため）
    
    Returns:
        8つのモーターの目標角度リスト（ラジアン）
    """
    # 公式サンプルの最適化されたパラメータを使用
    params = [
        0.1903581461951056,   # 振幅1
        0.0006732219568880068,  # 振幅2
        0.05018085615283363,   # 位相差
        3.219916795483583,     # 位相オフセット1
        6.2406418167980595,    # 位相オフセット2
        4.189869754607539      # 位相オフセット3
    ]
    return evaluate_desired_motorAngle_2Amplitude4Phase(step, params)


def trotting_pattern(step: int, speed: float = 0.35, amplitude: float = 0.3) -> List[float]:
    """
    トロット歩行パターンを生成（真っ直ぐ進むように左右対称に設定）
    
    トロット歩行では、対角線上の脚が同期します：
    - front_left + back_right が同じ位相
    - front_right + back_left が同じ位相（180度ずれ）
    
    Args:
        step: ステップ数
        speed: 歩行速度
        amplitude: 振幅
    
    Returns:
        8つのモーターの目標角度リスト（ラジアン）
    """
    speed = 0.35
    base_angle = 1.57  # π/2
    
    # トロットパターン: 対角線上の脚を同期
    # front_left + back_right が同じ位相
    # front_right + back_left が同じ位相（180度ずれ）
    phase1 = step * speed
    phase2 = step * speed + math.pi  # 180度ずれ
    
    # モーター順序: [front_left_L, front_left_R, back_left_L, back_left_R,
    #                front_right_L, front_right_R, back_right_L, back_right_R]
    # トロット: front_left + back_right が phase1、front_right + back_left が phase2
    angles = [
        math.sin(phase1) * amplitude + base_angle,  # front_left L
        math.sin(phase1) * amplitude + base_angle,  # front_left R
        math.sin(phase2) * amplitude + base_angle,  # back_left L
        math.sin(phase2) * amplitude + base_angle,  # back_left R
        math.sin(phase2) * amplitude + base_angle,  # front_right L
        math.sin(phase2) * amplitude + base_angle,  # front_right R
        math.sin(phase1) * amplitude + base_angle,  # back_right L
        math.sin(phase1) * amplitude + base_angle,  # back_right R
    ]
    
    return angles


def standing_pose() -> List[float]:
    """
    立ち姿勢の角度を返す
    
    Returns:
        8つのモーターの目標角度リスト（ラジアン、すべて90度）
    """
    base_angle = math.pi / 2
    return [base_angle] * 8


def main():
    """メイン関数"""
    print("=" * 80)
    print("Minitaur 制御サンプル")
    print("=" * 80)
    
    # PyBulletに接続（GUIモード）
    physics_client = p.connect(p.GUI)
    if physics_client < 0:
        print("PyBulletへの接続に失敗しました")
        return
    
    # データパスを設定
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # タイムステップを設定（公式サンプルと同じ）
    time_step = 0.01
    p.setTimeStep(time_step)
    
    # 重力を設定
    p.setGravity(0, 0, -10)
    
    # 地面をロード
    plane_id = p.loadURDF("plane.urdf")
    
    # Minitaurコントローラーを初期化
    print("\n📦 Minitaurロボットをロード中...")
    minitaur = MinitaurController()
    print("✅ ロボットのロードが完了しました")
    
    # カメラを設定
    p.resetDebugVisualizerCamera(
        cameraDistance=1.5,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0.2]
    )
    
    # 初期位置を記録
    start_position = minitaur.get_base_position()
    print(f"\n📍 初期位置: ({start_position[0]:.3f}, {start_position[1]:.3f}, {start_position[2]:.3f})")
    
    # 制御モードを選択
    print("\n制御モードを選択してください:")
    print("1: 立ち姿勢を維持")
    print("2: シンプルな歩行パターン")
    print("3: トロット歩行パターン")
    
    try:
        mode = input("モード番号を入力 (1-3, デフォルト: 1): ").strip()
        if not mode:
            mode = "1"
        mode = int(mode)
    except (ValueError, KeyboardInterrupt):
        mode = 1
    
    if mode not in [1, 2, 3]:
        mode = 1
    
    mode_names = {
        1: "立ち姿勢",
        2: "シンプル歩行",
        3: "トロット歩行"
    }
    print(f"\n🎮 モード: {mode_names[mode]}")
    print("シミュレーションを開始します...")
    print("Ctrl+Cで終了できます\n")
    
    # シミュレーションループ
    step = 0
    max_steps = 10000
    sleep_time = time_step  # 可視化のための待機時間
    
    try:
        while step < max_steps:
            # 転倒チェック
            if minitaur.is_fallen():
                print(f"\n⚠️ ロボットが転倒しました（ステップ: {step}）")
                break
            
            # 制御モードに応じて角度を計算
            if mode == 1:
                motor_angles = standing_pose()
            elif mode == 2:
                motor_angles = simple_walking_pattern(step, speed=0.35, amplitude=0.3)
            elif mode == 3:
                motor_angles = trotting_pattern(step, speed=0.35, amplitude=0.3)
            else:
                motor_angles = standing_pose()
            
            # モーターコマンドを適用
            minitaur.apply_action(motor_angles)
            
            # シミュレーションを1ステップ進める
            p.stepSimulation()
            
            # 100ステップごとに情報を表示
            if step % 100 == 0:
                position = minitaur.get_base_position()
                distance = np.linalg.norm(np.array(start_position) - np.array(position))
                print(f"ステップ {step:5d} | 位置: ({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}) | "
                      f"移動距離: {distance:.3f}m")
            
            step += 1
            time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        print("\n\n⏹️ シミュレーションを中断しました")
    
    # 最終結果を表示
    final_position = minitaur.get_base_position()
    total_distance = np.linalg.norm(np.array(start_position) - np.array(final_position))
    
    print("\n" + "=" * 80)
    print("📊 シミュレーション結果")
    print("=" * 80)
    print(f"総ステップ数: {step}")
    print(f"開始位置: ({start_position[0]:.3f}, {start_position[1]:.3f}, {start_position[2]:.3f})")
    print(f"終了位置: ({final_position[0]:.3f}, {final_position[1]:.3f}, {final_position[2]:.3f})")
    print(f"総移動距離: {total_distance:.3f}m")
    print("\nEnterキーを押すと終了します...")
    
    try:
        input()
    except KeyboardInterrupt:
        pass
    
    # 接続を切断
    p.disconnect()
    print("✅ シミュレーションを終了しました")


if __name__ == "__main__":
    main()
