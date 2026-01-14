# PyBullet カメラを使ったロボット学習プラン

このドキュメントは、PyBulletでカメラを使ったロボット学習を進めるためのガイドです。実機が存在するロボットを中心に、カメラ視点での制御と学習方法を説明します。

---

## カメラ対応ロボット一覧（実機あり）

PyBullet_URDF_List.mdから、実機が存在し、カメラを使った学習に適したロボットを以下にまとめました。

### 1. **Franka Panda** ⭐ おすすめ
- **URDFファイル**: `franka_panda/panda.urdf`
- **実機**: ✅ あり（Franka Emika社製）
- **特徴**: 
  - 7自由度の産業用ロボットアーム
  - 実機でカメラ統合が一般的
  - 研究・教育用途で広く使用されている
  - PyBulletでのサンプルコードが豊富
- **カメラ用途**: エンドエフェクタにカメラを配置し、物体認識・把持タスクに使用
- **学習難易度**: ⭐⭐⭐（中級）

### 2. **KUKA LBR iiwa**
- **URDFファイル**: `kuka_iiwa/model.urdf`
- **実機**: ✅ あり（KUKA社製）
- **特徴**:
  - 7自由度の産業用ロボットアーム
  - 高精度な制御が可能
  - グリッパー付きモデルも利用可能
- **カメラ用途**: エンドエフェクタにカメラを配置
- **学習難易度**: ⭐⭐⭐（中級）

### 3. **Quadrotor（クアッドロータードローン）** ⭐ おすすめ
- **URDFファイル**: `Quadrotor/quadrotor.urdf`
- **実機**: ✅ あり（一般的なドローン）
- **特徴**:
  - 4つのプロペラで飛行
  - カメラ視点での制御が一般的
  - 視覚ナビゲーションの研究に最適
- **カメラ用途**: 機体下部にカメラを配置し、地上の物体を認識・追跡
- **学習難易度**: ⭐⭐⭐⭐（上級）

### 4. **Husky**
- **URDFファイル**: `husky/husky.urdf`
- **実機**: ✅ あり（Clearpath Robotics社製）
- **特徴**:
  - 移動ロボットプラットフォーム
  - 実機にカメラが標準搭載されていることが多い
  - ナビゲーション・マッピングタスクに適している
- **カメラ用途**: 前方カメラで障害物検出・ナビゲーション
- **学習難易度**: ⭐⭐⭐（中級）

### 5. **NAO**
- **URDFファイル**: `humanoid/nao.urdf`
- **実機**: ✅ あり（SoftBank Robotics社製）
- **特徴**:
  - ヒューマノイドロボット
  - 頭部にカメラが標準搭載
  - 人間とのインタラクション研究に使用
- **カメラ用途**: 頭部カメラで顔認識・物体認識
- **学習難易度**: ⭐⭐⭐⭐（上級）

### 6. **Vision60（四足ロボット）**
- **URDFファイル**: `quadruped/vision/vision60.urdf`
- **実機**: ✅ あり（Ghost Robotics社製）
- **特徴**:
  - 四足歩行ロボット
  - 名前からカメラ機能が想定される
  - 不整地歩行と視覚ナビゲーション
- **カメラ用途**: 頭部カメラで地形認識・ナビゲーション
- **学習難易度**: ⭐⭐⭐⭐（上級）

---

## カメラを使った学習プラン

### ステージ1: 基礎 - カメラ画像の取得

#### 1.1 固定カメラからの画像取得

**目標**: ロボットを観察する固定カメラから画像を取得する

```python
import pybullet as p
import pybullet_data
import numpy as np
import cv2

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# 環境のセットアップ
p.loadURDF("plane.urdf")
robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

# カメラパラメータ
width, height = 640, 480
view_matrix = p.computeViewMatrix(
    cameraEyePosition=[1.0, 1.0, 1.0],  # カメラ位置
    cameraTargetPosition=[0, 0, 0],     # 注視点
    cameraUpVector=[0, 0, 1]
)
projection_matrix = p.computeProjectionMatrixFOV(
    fov=60, aspect=width/height, nearVal=0.1, farVal=100.0
)

# シミュレーションループ
for i in range(1000):
    p.stepSimulation()
    
    # カメラ画像を取得
    images = p.getCameraImage(width, height, 
                             viewMatrix=view_matrix,
                             projectionMatrix=projection_matrix)
    
    # RGB画像に変換
    rgb_array = np.array(images[2], dtype=np.uint8)
    rgb_array = rgb_array.reshape((height, width, 4))[:, :, :3]
    
    # 画像処理（例：グレースケール変換）
    gray = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2GRAY)
    
    # 必要に応じて保存
    if i % 100 == 0:
        cv2.imwrite(f"frame_{i}.png", rgb_array)

p.disconnect()
```

#### 1.2 ロボット追従カメラ

**目標**: ロボットの位置に合わせてカメラを動的に更新する

```python
# ロボットの位置を取得
pos, orn = p.getBasePositionAndOrientation(robot_id)

# カメラをロボットの後ろ上から見る
camera_distance = 2.0
camera_height = 1.0
camera_angle = 45.0  # 度

view_matrix = p.computeViewMatrix(
    cameraEyePosition=[
        pos[0] + camera_distance * np.cos(np.radians(camera_angle)),
        pos[1] + camera_distance * np.sin(np.radians(camera_angle)),
        pos[2] + camera_height
    ],
    cameraTargetPosition=pos,  # ロボットを注視
    cameraUpVector=[0, 0, 1]
)
```

---

### ステージ2: 応用 - ロボット視点カメラ

#### 2.1 エンドエフェクタ視点カメラ（Franka Panda）

**目標**: ロボットアームのエンドエフェクタ（手先）にカメラを配置し、その視点から画像を取得する

```python
import pybullet as p
import pybullet_data
import numpy as np
import cv2

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

p.loadURDF("plane.urdf")
robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

# Pandaのエンドエフェクタリンクインデックス（通常11）
end_effector_index = 11

width, height = 640, 480

for i in range(1000):
    p.stepSimulation()
    
    # エンドエフェクタの位置と姿勢を取得
    link_state = p.getLinkState(robot_id, end_effector_index)
    ee_pos = link_state[4]  # 世界座標での位置
    ee_orn = link_state[5]   # 世界座標での姿勢（クォータニオン）
    
    # エンドエフェクタの姿勢からカメラの向きを計算
    # カメラはエンドエフェクタの前方（Z軸方向）を見る
    camera_offset = 0.1  # エンドエフェクタから少し前
    camera_pos = [
        ee_pos[0],
        ee_pos[1],
        ee_pos[2] + camera_offset
    ]
    
    # カメラは下を見る（物体把持用）
    target_pos = [ee_pos[0], ee_pos[1], ee_pos[2] - 0.2]
    
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=camera_pos,
        cameraTargetPosition=target_pos,
        cameraUpVector=[0, 0, 1]
    )
    projection_matrix = p.computeProjectionMatrixFOV(
        fov=60, aspect=width/height, nearVal=0.01, farVal=10.0
    )
    
    # カメラ画像を取得
    images = p.getCameraImage(width, height,
                             viewMatrix=view_matrix,
                             projectionMatrix=projection_matrix)
    
    rgb_array = np.array(images[2], dtype=np.uint8)
    rgb_array = rgb_array.reshape((height, width, 4))[:, :, :3]
    
    # 画像処理や保存
    if i % 100 == 0:
        cv2.imwrite(f"ee_view_{i}.png", rgb_array)

p.disconnect()
```

#### 2.2 ドローン視点カメラ（Quadrotor）

**目標**: ドローンの機体下部にカメラを配置し、地上の物体を認識する

```python
import pybullet as p
import pybullet_data
import numpy as np
import cv2

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

p.loadURDF("plane.urdf")
drone_id = p.loadURDF("Quadrotor/quadrotor.urdf", basePosition=[0, 0, 2])

width, height = 640, 480

for i in range(1000):
    # ドローンの制御（例：ホバリング）
    # ... 制御コード ...
    
    p.stepSimulation()
    
    # ドローンの位置と姿勢を取得
    pos, orn = p.getBasePositionAndOrientation(drone_id)
    
    # 機体下部にカメラを配置（下を見る）
    camera_pos = [pos[0], pos[1], pos[2] - 0.1]  # 機体下部
    target_pos = [pos[0], pos[1], 0]  # 地面を注視
    
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=camera_pos,
        cameraTargetPosition=target_pos,
        cameraUpVector=[1, 0, 0]  # ドローンの前方向が上
    )
    projection_matrix = p.computeProjectionMatrixFOV(
        fov=90, aspect=width/height, nearVal=0.1, farVal=50.0
    )
    
    # カメラ画像を取得
    images = p.getCameraImage(width, height,
                             viewMatrix=view_matrix,
                             projectionMatrix=projection_matrix)
    
    rgb_array = np.array(images[2], dtype=np.uint8)
    rgb_array = rgb_array.reshape((height, width, 4))[:, :, :3]
    
    # 物体検出やナビゲーション処理
    # ... 画像処理コード ...

p.disconnect()
```

---

### ステージ3: 高度 - 視覚ベース制御

#### 3.1 視覚フィードバック制御（Franka Panda）

**目標**: カメラ画像から物体の位置を推定し、ロボットアームを制御する

```python
import pybullet as p
import pybullet_data
import numpy as np
import cv2

def detect_object_in_image(rgb_image):
    """画像から物体を検出し、位置を返す（簡易版）"""
    # 実際には、YOLO、SSD、またはカラー検出などを使用
    # ここでは簡易的に色検出の例
    hsv = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)
    
    # 赤色物体を検出（例）
    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red)
    
    # 重心を計算
    M = cv2.moments(mask)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return (cx, cy)
    return None

def pixel_to_world(pixel_x, pixel_y, camera_pos, camera_orn, 
                   image_width, image_height, depth_image):
    """ピクセル座標を世界座標に変換"""
    # 深度情報を使用して3D位置を計算
    # 簡易版：深度画像から距離を取得
    depth = depth_image[pixel_y, pixel_x]
    
    # カメラ座標系での位置を計算
    # （実際には、カメラの内部パラメータと外部パラメータを使用）
    # ここでは簡易的な変換
    fx = fy = image_width / (2 * np.tan(np.radians(60) / 2))
    x_cam = (pixel_x - image_width / 2) * depth / fx
    y_cam = (pixel_y - image_height / 2) * depth / fy
    z_cam = depth
    
    # 世界座標系に変換（クォータニオンを使用）
    # ... 座標変換コード ...
    
    return world_pos

# メインループ
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

p.loadURDF("plane.urdf")
robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
object_id = p.loadURDF("cube.urdf", basePosition=[0.5, 0.0, 0.1])

end_effector_index = 11

for i in range(1000):
    # エンドエフェクタ視点のカメラ画像を取得
    link_state = p.getLinkState(robot_id, end_effector_index)
    ee_pos = link_state[4]
    
    # カメラ画像を取得
    images = p.getCameraImage(640, 480, ...)
    rgb_array = np.array(images[2], dtype=np.uint8)
    rgb_array = rgb_array.reshape((480, 640, 4))[:, :, :3]
    depth_array = np.array(images[3])  # 深度画像
    
    # 物体を検出
    pixel_pos = detect_object_in_image(rgb_array)
    
    if pixel_pos:
        # ピクセル座標を世界座標に変換
        world_pos = pixel_to_world(
            pixel_pos[0], pixel_pos[1],
            ee_pos, link_state[5],
            640, 480, depth_array
        )
        
        # 逆運動学で関節角度を計算
        joint_angles = p.calculateInverseKinematics(
            robot_id, end_effector_index, world_pos
        )
        
        # ロボットを制御
        for j, angle in enumerate(joint_angles):
            p.setJointMotorControl2(
                robot_id, j, p.POSITION_CONTROL, targetPosition=angle
            )
    
    p.stepSimulation()

p.disconnect()
```

---

## 推奨学習順序

### 初心者向け
1. **Franka Panda + 固定カメラ**
   - ロボットの動作を観察
   - カメラ画像の取得方法を習得

2. **Franka Panda + エンドエフェクタ視点カメラ**
   - ロボットの視点を体験
   - 物体認識の基礎

### 中級者向け
3. **Quadrotor + ドローン視点カメラ**
   - 飛行制御と視覚ナビゲーション
   - 地上物体の追跡

4. **視覚フィードバック制御**
   - 画像から物体位置を推定
   - ロボット制御への統合

### 上級者向け
5. **複数カメラの統合**
   - ステレオビジョン
   - 3D位置推定の精度向上

6. **深層学習との統合**
   - 物体検出モデル（YOLO、SSD）の統合
   - エンドツーエンドの学習

---

## 実機への移行

### Franka Pandaの場合
- **ROS統合**: `franka_ros`パッケージを使用
- **カメラ**: Intel RealSense、ZEDカメラなど
- **実機制御**: PyBulletで学習した制御をROS経由で実機に適用

### Quadrotorの場合
- **フライトコントローラー**: PX4、ArduPilotなど
- **カメラ**: 機体下部にカメラを搭載
- **実機制御**: シミュレーションで学習した制御を実機に移植

---

## 参考リソース

- **PyBullet Camera Controls**: `docs/PyBullet_CAMERA_Controls.md`
- **Franka Panda公式**: https://www.franka.de/
- **PyBullet Panda Examples**: https://github.com/irom-lab/pybullet-panda
- **Panda Gym**: https://github.com/qgallouedec/panda-gym

---

## まとめ

PyBulletでは、実機が存在するロボットを使ってカメラベースの学習を進めることができます。特に**Franka Panda**と**Quadrotor**は、実機でのカメラ統合が一般的で、シミュレーションから実機への移行が容易です。

まずは固定カメラから始め、徐々にロボット視点カメラ、そして視覚フィードバック制御へと進めることをおすすめします。
