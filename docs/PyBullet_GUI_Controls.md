# PyBullet GUI操作マニュアル

PyBulletのGUIモード（`p.GUI`）でのカメラ操作とキーボードショートカットの説明です。

## マウス操作

### カメラの回転
- **左マウスボタン + ドラッグ**: カメラを回転させます
  - マウスを左右に動かすと、カメラが左右に回転します
  - マウスを上下に動かすと、カメラが上下に回転します

### カメラのパン（平行移動）
- **右マウスボタン + ドラッグ**: カメラを平行移動させます
  - マウスを動かす方向にカメラが移動します
  - オブジェクトを中心にカメラを移動させたい場合に便利です

### ズーム
- **マウスホイール**: ズームイン/アウト
  - 上にスクロール: ズームイン（近づく）
  - 下にスクロール: ズームアウト（遠ざかる）

## キーボードショートカット

### カメラのリセット
- **Homeキー**: カメラビューをデフォルト位置にリセットします
  - オブジェクトが画面から見えなくなった場合に便利です

## プログラムからカメラを制御

PyBulletでは、プログラムからカメラを制御することもできます。

### カメラの位置と向きを設定

```python
import pybullet as p

# カメラの位置と向きを設定
camera_distance = 5.0  # カメラまでの距離
camera_yaw = 45.0      # 水平方向の角度（度）
camera_pitch = -30.0   # 垂直方向の角度（度）
camera_target = [0, 0, 0]  # カメラが向く目標位置

p.resetDebugVisualizerCamera(
    cameraDistance=camera_distance,
    cameraYaw=camera_yaw,
    cameraPitch=camera_pitch,
    cameraTargetPosition=camera_target
)
```

### オブジェクトを追従するカメラ

オブジェクトの位置に合わせてカメラを自動的に更新する例：

```python
import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

p.loadURDF("plane.urdf")
robot_id = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 0.5])

for i in range(1000):
    p.stepSimulation()
    
    # ロボットの位置を取得
    pos, _ = p.getBasePositionAndOrientation(robot_id)
    
    # カメラをロボットの位置に追従させる
    p.resetDebugVisualizerCamera(
        cameraDistance=3.0,      # カメラまでの距離
        cameraYaw=45.0,          # 水平角度
        cameraPitch=-20.0,       # 垂直角度（上から見下ろす）
        cameraTargetPosition=pos  # ロボットの位置を追従
    )
    
    time.sleep(1./240.)
```

## カメラ設定のオプション

`p.connect(p.GUI)`のオプションで、マウス感度などを調整できます：

```python
# マウス感度を調整
p.connect(p.GUI, options="--mouse_move_multiplier=0.5")
p.connect(p.GUI, options="--mouse_wheel_multiplier=0.5")

# ウィンドウサイズを設定
p.connect(p.GUI, options="--width=1280 --height=720")
```

## よくある問題と解決策

### オブジェクトが画面から見えなくなった

1. **Homeキーを押す**: カメラをデフォルト位置にリセット
2. **右マウスドラッグ**: カメラをパンしてオブジェクトを探す
3. **マウスホイール**: ズームアウトして広い範囲を見る
4. **プログラムでカメラをリセット**: `resetDebugVisualizerCamera()`を使用

### カメラが動きすぎる

- マウス感度を下げる: `--mouse_move_multiplier=0.2`など、小さい値を設定

### オブジェクトを常に追従させたい

- シミュレーションループ内で、オブジェクトの位置を取得して`resetDebugVisualizerCamera()`を毎フレーム呼び出す

## 参考リンク

- [PyBullet Documentation](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit)
- [Bullet Physics GitHub](https://github.com/bulletphysics/bullet3)
