## DIRECTモードでのカメラ・録画機能

### 1. DIRECTモードでカメラ画像を取得

```python
import pybullet as p
import pybullet_data
import numpy as np
import cv2  # OpenCVが必要

p.connect(p.DIRECT)  # GUIなし
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

p.loadURDF("plane.urdf")
robot_id = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 0.5])

# カメラパラメータを設定
width, height = 640, 480
view_matrix = p.computeViewMatrix(
    cameraEyePosition=[3, 3, 3],      # カメラ位置
    cameraTargetPosition=[0, 0, 0],   # 注視点
    cameraUpVector=[0, 0, 1]           # 上方向
)
projection_matrix = p.computeProjectionMatrixFOV(
    fov=60,                            # 視野角
    aspect=width/height,
    nearVal=0.1,
    farVal=100.0
)

# カメラ画像を取得
for i in range(1000):
    p.stepSimulation()
    
    # カメラ画像を取得（RGBA、深度、セグメンテーション）
    images = p.getCameraImage(
        width, height,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix
    )
    
    # RGB画像を取得（RGBAの最初の3チャンネル）
    rgb_array = np.array(images[2])
    rgb_array = rgb_array[:, :, :3]  # RGBA -> RGB
    
    # OpenCVで表示や保存
    # cv2.imshow('PyBullet', rgb_array)
    # cv2.waitKey(1)
```

### 2. カメラの動的設定（オブジェクト追従）

```python
# オブジェクトの位置に合わせてカメラを更新
pos, _ = p.getBasePositionAndOrientation(robot_id)

view_matrix = p.computeViewMatrix(
    cameraEyePosition=[pos[0]+3, pos[1]+3, pos[2]+1],  # ロボットの後ろ上
    cameraTargetPosition=pos,                           # ロボットを注視
    cameraUpVector=[0, 0, 1]
)
```

### 3. 録画機能

**方法1: mp4オプション（GUIモードのみ）**
```python
# GUIモードで録画
p.connect(p.GUI, options="--mp4=output.mp4 --mp4fps=30")
```

**方法2: getCameraImageでフレームを保存（DIRECTモードでも可）**
```python
import imageio  # または opencv

frames = []
for i in range(1000):
    p.stepSimulation()
    
    # カメラ画像を取得
    images = p.getCameraImage(width, height, ...)
    rgb_array = np.array(images[2])[:, :, :3]
    
    frames.append(rgb_array)

# 動画として保存
imageio.mimsave('output.mp4', frames, fps=30)
```

### 4. 光源の設定

PyBulletの光源設定は限定的です：

```python
# 環境光の設定（限定的）
p.configureDebugVisualizer(
    p.COV_ENABLE_SHADOWS, 1  # シャドウを有効化
)

# より高度な光源制御は、カスタムシェーダーや
# 後処理で調整する必要があります
```

### 5. 実装例（DIRECTモード + 録画）

```python
import pybullet as p
import pybullet_data
import numpy as np
import imageio

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

p.loadURDF("plane.urdf")
robot_id = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 0.5])

width, height = 1280, 720
frames = []

for i in range(1000):
    p.stepSimulation()
    
    # ロボットの位置を取得
    pos, _ = p.getBasePositionAndOrientation(robot_id)
    
    # カメラを設定
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=[pos[0]+3, pos[1]+3, pos[2]+1],
        cameraTargetPosition=pos,
        cameraUpVector=[0, 0, 1]
    )
    projection_matrix = p.computeProjectionMatrixFOV(
        fov=60, aspect=width/height, nearVal=0.1, farVal=100.0
    )
    
    # 画像を取得
    images = p.getCameraImage(width, height, 
                             viewMatrix=view_matrix,
                             projectionMatrix=projection_matrix)
    rgb_array = np.array(images[2])[:, :, :3]
    frames.append(rgb_array)

# 動画として保存
imageio.mimsave('r2d2_simulation.mp4', frames, fps=30)
p.disconnect()
```

## 制限事項

- 光源制御: 環境光やシャドウの有効化は可能ですが、UnrealEngineほど細かくは制御できません
- レンダリング品質: OpenGLレンダリングより品質は劣りますが、DIRECTモードでも画像取得は可能です
- パフォーマンス: 毎フレーム`getCameraImage`を呼ぶと処理が重くなります

## まとめ

DIRECTモードでも、`getCameraImage`でカメラ画像を取得し、動画として保存できます。UnrealEngineほどの高機能ではありませんが、シミュレーション結果の可視化と録画は可能です。