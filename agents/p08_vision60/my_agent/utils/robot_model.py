"""
ロボットモデル管理モジュール

Vision60ロボットのジョイント構造、リンク情報などを管理
"""
import pybullet as p
from typing import Dict, List, Optional
from .config import config


class RobotModel:
    """Vision60ロボットモデル管理クラス"""
    
    def __init__(self, robot_id: int):
        """
        ロボットモデルを初期化
        
        Args:
            robot_id: PyBulletのロボットID
        """
        self.robot_id = robot_id
        self.leg_joints = config.LEG_JOINTS.copy()
        self.link_name_to_index: Dict[str, int] = {}
        self.original_colors: Dict[int, List[float]] = {}  # オリジナルの色を保存
        self._build_link_name_mapping()
        self._save_original_colors()
    
    def _build_link_name_mapping(self):
        """リンク名からインデックスへのマッピングを構築"""
        num_links = p.getNumJoints(self.robot_id)
        for link_idx in range(num_links):
            try:
                joint_info = p.getJointInfo(self.robot_id, link_idx)
                link_name = joint_info[12].decode('utf-8') if joint_info[12] else f"link_{link_idx}"
                self.link_name_to_index[link_name] = link_idx
            except:
                pass
    
    def _save_original_colors(self):
        """オリジナルの色を保存"""
        try:
            # getVisualShapeDataはタプルのリストを返す
            # 各タプル: (objectUniqueId, linkIndex, visualGeometryType, dimensions, 
            #           meshAssetFileName, localVisualFramePosition, 
            #           localVisualFrameOrientation, rgbaColor)
            all_visual_data = p.getVisualShapeData(self.robot_id)
            
            if not all_visual_data:
                # データが取得できなかった場合はデフォルトの半透明色を使用
                num_links = p.getNumJoints(self.robot_id)
                for link_idx in range(-1, num_links):
                    self.original_colors[link_idx] = [0.5, 0.5, 0.5, 0.7]  # グレー半透明
                return
            
            # リンクごとに最初の視覚形状の色を保存
            for visual_data in all_visual_data:
                if len(visual_data) > 7:
                    link_idx = visual_data[1]  # linkIndex
                    rgba_color = visual_data[7]  # rgbaColorはインデックス7
                    # 同じリンクの色は最初のものだけを保存
                    if link_idx not in self.original_colors:
                        # rgba_colorがタプルの場合はリストに変換
                        if isinstance(rgba_color, (tuple, list)):
                            self.original_colors[link_idx] = list(rgba_color)
                        else:
                            # デフォルトの半透明色を使用
                            self.original_colors[link_idx] = [0.5, 0.5, 0.5, 0.7]
            
            # ベースリンクを含むすべてのリンクの色が保存されていない場合はデフォルトを使用
            num_links = p.getNumJoints(self.robot_id)
            for link_idx in range(-1, num_links):
                if link_idx not in self.original_colors:
                    self.original_colors[link_idx] = [0.5, 0.5, 0.5, 0.7]  # グレー半透明
                    
        except Exception as e:
            # 色取得に失敗した場合はデフォルトの半透明色を使用
            num_links = p.getNumJoints(self.robot_id)
            for link_idx in range(-1, num_links):
                self.original_colors[link_idx] = [0.5, 0.5, 0.5, 0.7]  # グレー半透明
    
    def get_link_index(self, link_name: str) -> Optional[int]:
        """
        リンク名からインデックスを取得
        
        Args:
            link_name: リンク名（例: "upper0", "lower0"）
            
        Returns:
            リンクインデックス、見つからない場合はNone
        """
        return self.link_name_to_index.get(link_name)
    
    def get_joint_indices(self, leg_name: str) -> List[int]:
        """
        脚名からジョイントインデックスを取得
        
        Args:
            leg_name: 脚名（"front_left", "front_right", "back_left", "back_right"）
            
        Returns:
            ジョイントインデックスのリスト [abduction, hip, knee]
        """
        return self.leg_joints.get(leg_name, [])
    
    def get_knee_joint_index(self, leg_name: str) -> Optional[int]:
        """
        脚名から膝ジョイントインデックスを取得
        
        Args:
            leg_name: 脚名
            
        Returns:
            膝ジョイントインデックス（3番目のジョイント）
        """
        joint_indices = self.get_joint_indices(leg_name)
        if len(joint_indices) >= 3:
            return joint_indices[2]  # kneeは3番目
        return None
    
    def get_hip_joint_index(self, leg_name: str) -> Optional[int]:
        """
        脚名からhipジョイントインデックスを取得
        
        Args:
            leg_name: 脚名
            
        Returns:
            hipジョイントインデックス（2番目のジョイント）
        """
        joint_indices = self.get_joint_indices(leg_name)
        if len(joint_indices) >= 2:
            return joint_indices[1]  # hipは2番目
        return None
    
    def get_abduction_joint_index(self, leg_name: str) -> Optional[int]:
        """
        脚名からabductionジョイントインデックスを取得
        
        Args:
            leg_name: 脚名
            
        Returns:
            abductionジョイントインデックス（1番目のジョイント）
        """
        joint_indices = self.get_joint_indices(leg_name)
        if len(joint_indices) >= 1:
            return joint_indices[0]  # abductionは1番目
        return None
    
    def get_toe_link_index(self, leg_index: int) -> Optional[int]:
        """
        脚インデックスからtoeリンクインデックスを取得
        
        Args:
            leg_index: 脚インデックス（0: front_left, 1: front_right, 2: back_left, 3: back_right）
            
        Returns:
            toeリンクインデックス
        """
        toe_link_name = f"toe{leg_index}"
        return self.get_link_index(toe_link_name)
    
    def get_upper_link_index(self, leg_index: int) -> Optional[int]:
        """
        脚インデックスからupperリンクインデックスを取得
        
        Args:
            leg_index: 脚インデックス
            
        Returns:
            upperリンクインデックス
        """
        upper_link_name = f"upper{leg_index}"
        return self.get_link_index(upper_link_name)
    
    def get_lower_link_index(self, leg_index: int) -> Optional[int]:
        """
        脚インデックスからlowerリンクインデックスを取得
        
        Args:
            leg_index: 脚インデックス
            
        Returns:
            lowerリンクインデックス
        """
        lower_link_name = f"lower{leg_index}"
        return self.get_link_index(lower_link_name)
    
    def get_leg_index_map(self) -> Dict[str, int]:
        """
        脚名から脚インデックスへのマッピングを取得
        
        Returns:
            脚名 -> 脚インデックスのマッピング
        """
        return {
            'front_left': 0,
            'front_right': 1,
            'back_left': 2,
            'back_right': 3
        }
    
    def change_robot_color(self, rgba_color: List[float]):
        """
        ロボット全体の色を変更
        
        Args:
            rgba_color: RGBA色 [r, g, b, a] (各値は0.0～1.0)
        """
        num_links = p.getNumJoints(self.robot_id)
        
        # ベースリンクを含むすべてのリンクの色を変更
        for link_idx in range(-1, num_links):  # -1はベースリンク
            try:
                p.changeVisualShape(
                    self.robot_id,
                    link_idx,
                    rgbaColor=rgba_color
                )
            except Exception as e:
                # 色変更に失敗しても続行
                pass
    
    def restore_original_colors(self):
        """オリジナルの色に戻す"""
        if not self.original_colors:
            # オリジナルの色が保存されていない場合は何もしない
            return
        
        num_links = p.getNumJoints(self.robot_id)
        restored_count = 0
        
        # ベースリンクを含むすべてのリンクの色を復元
        for link_idx in range(-1, num_links):  # -1はベースリンク
            if link_idx in self.original_colors:
                try:
                    rgba_color = self.original_colors[link_idx]
                    # リストでない場合はリストに変換
                    if not isinstance(rgba_color, list):
                        rgba_color = list(rgba_color)
                    p.changeVisualShape(
                        self.robot_id,
                        link_idx,
                        rgbaColor=rgba_color
                    )
                    restored_count += 1
                except Exception:
                    # 色変更に失敗しても続行
                    pass
