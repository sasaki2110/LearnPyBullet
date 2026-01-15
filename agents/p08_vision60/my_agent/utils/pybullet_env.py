"""
PyBullet環境管理モジュール

PyBulletシミュレーション環境の作成・管理を行う
"""
import pybullet as p
import pybullet_data
from typing import Optional, Tuple
from .config import config
from .logging_config import get_logger

logger = get_logger('pybullet_env')


class PyBulletEnvironment:
    """PyBulletシミュレーション環境の管理クラス"""
    
    def __init__(self):
        """環境を初期化（接続は行わない）"""
        self.client_id: Optional[int] = None
        self.robot_id: Optional[int] = None
        self.plane_id: Optional[int] = None
        self.initial_pos = config.INITIAL_POS.copy()
        
    def create_environment(self) -> int:
        """
        PyBullet環境を作成
        
        Returns:
            クライアントID
        """
        # GUIモードで接続
        if config.GUI_MODE:
            self.client_id = p.connect(p.GUI)
        else:
            self.client_id = p.connect(p.DIRECT)
        
        if self.client_id < 0:
            raise RuntimeError("GUI接続に失敗しました。")
        
        # PyBulletデータパスを追加
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # タイムステップを設定（物理シミュレーションの精度を向上）
        p.setTimeStep(config.TIME_STEP)
        
        # 重力を設定
        p.setGravity(*config.GRAVITY)
        
        return self.client_id
    
    def load_plane(self) -> int:
        """
        地面をロード
        
        Returns:
            地面のID
        """
        self.plane_id = p.loadURDF("plane.urdf")
        logger.info("✅ 地面をロードしました")
        
        # 地面の物理パラメータを調整
        p.changeDynamics(
            self.plane_id,
            -1,  # ベースリンク
            restitution=config.PLANE_RESTITUTION,
            lateralFriction=config.PLANE_LATERAL_FRICTION,
            spinningFriction=config.PLANE_SPINNING_FRICTION,
            rollingFriction=config.PLANE_ROLLING_FRICTION
        )
        logger.info("✅ 地面の物理パラメータを調整しました（反発係数低減）")
        
        return self.plane_id
    
    def load_robot(self, urdf_path: str = "quadruped/vision60.urdf") -> int:
        """
        Vision60ロボットをロード
        
        Args:
            urdf_path: URDFファイルのパス
            
        Returns:
            ロボットのID
        """
        self.robot_id = p.loadURDF(urdf_path, basePosition=self.initial_pos)
        logger.info("✅ Vision60をロードしました")
        
        # 物理パラメータを調整
        self._configure_robot_physics()
        
        return self.robot_id
    
    def _configure_robot_physics(self):
        """ロボットの物理パラメータを設定"""
        num_links = p.getNumJoints(self.robot_id)
        
        for link_idx in range(-1, num_links):  # -1はベースリンク
            try:
                if link_idx == -1:
                    # ベースリンクの物理パラメータ
                    p.changeDynamics(
                        self.robot_id,
                        link_idx,
                        mass=config.BASE_MASS,
                        linearDamping=config.BASE_LINEAR_DAMPING,
                        angularDamping=config.BASE_ANGULAR_DAMPING,
                        restitution=config.BASE_RESTITUTION,
                        lateralFriction=config.BASE_LATERAL_FRICTION,
                        spinningFriction=config.BASE_SPINNING_FRICTION,
                        rollingFriction=config.BASE_ROLLING_FRICTION
                    )
                else:
                    # 各リンクの物理パラメータ
                    p.changeDynamics(
                        self.robot_id,
                        link_idx,
                        linearDamping=config.LINK_LINEAR_DAMPING,
                        angularDamping=config.LINK_ANGULAR_DAMPING,
                        restitution=config.LINK_RESTITUTION,
                        lateralFriction=config.LINK_LATERAL_FRICTION,
                        spinningFriction=config.LINK_SPINNING_FRICTION,
                        rollingFriction=config.LINK_ROLLING_FRICTION
                    )
            except Exception as e:
                logger.warning(f"  警告: リンク {link_idx} の物理パラメータ設定に失敗: {e}")
        
        # ジョイントのダンピングを設定
        num_joints = p.getNumJoints(self.robot_id)
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_type = joint_info[2]
            
            if joint_type == p.JOINT_REVOLUTE:  # 回転ジョイントの場合
                try:
                    p.changeDynamics(
                        self.robot_id,
                        i,
                        jointDamping=config.JOINT_DAMPING
                    )
                except:
                    pass
        
        logger.info("✅ 物理パラメータを調整しました（質量増加、減衰追加、反発係数低減）")
    
    def setup_camera(self):
        """カメラを初期設定"""
        p.resetDebugVisualizerCamera(
            cameraDistance=config.CAMERA_DISTANCE,
            cameraYaw=config.CAMERA_YAW,
            cameraPitch=config.CAMERA_PITCH,
            cameraTargetPosition=self.initial_pos
        )
    
    def print_joint_info(self, max_joints: int = 16):
        """ジョイント情報を表示"""
        num_joints = p.getNumJoints(self.robot_id)
        logger.info(f"\n📊 ジョイント数: {num_joints}")
        
        logger.info("\n📊 ジョイント情報:")
        for i in range(min(max_joints, num_joints)):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('utf-8')
            joint_type = joint_info[2]
            logger.info(f"  ジョイント {i}: {joint_name} (タイプ: {joint_type})")
    
    def disconnect(self):
        """PyBullet環境を切断"""
        if self.client_id is not None:
            p.disconnect()
            logger.info("✅ シミュレーションを終了しました。")
