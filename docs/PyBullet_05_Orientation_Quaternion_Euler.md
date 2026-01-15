# PyBullet Orientation（姿勢）の詳細解説
## クォータニオンとオイラー角の変換

このドキュメントでは、PyBulletにおける姿勢（Orientation）の表現方法と、クォータニオン・オイラー角の変換について詳しく説明します。

---

## 目次

1. [姿勢（Orientation）とは](#姿勢orientationとは)
2. [クォータニオン（Quaternion）](#クォータニオンquaternion)
3. [オイラー角（Euler Angles）](#オイラー角euler-angles)
4. [変換方法](#変換方法)
5. [実践的な使用例](#実践的な使用例)
6. [よくある質問](#よくある質問)

---

## 姿勢（Orientation）とは

姿勢（Orientation）は、**オブジェクトの向き（回転）**を表現する値です。

PyBulletでは、位置（Position）と姿勢（Orientation）を組み合わせて、オブジェクトの3D空間での状態を完全に表現します。

```python
pos, orn = p.getBasePositionAndOrientation(robot_id)
# pos: 位置 [x, y, z]
# orn: 姿勢 [x, y, z, w] ← クォータニオン形式
```

---

## クォータニオン（Quaternion）

### 基本概念

**クォータニオン**は、3D空間での回転を表現するための4つの値 `[x, y, z, w]` です。

- **形式**: `[x, y, z, w]` （4つの浮動小数点値）
- **単位**: 無次元（正規化された値）
- **用途**: 3D回転の計算に適している（ジンバルロックを回避）

### クォータニオンの特徴

**✅ メリット：**
- ジンバルロック（Gimbal Lock）が発生しない
- 回転の補間（スムーズな回転）が容易
- 計算が効率的

**❌ デメリット：**
- 人間にとって直感的ではない
- 4つの値が必要で理解しにくい

### PyBulletでのクォータニオン

PyBulletでは、**すべての姿勢はクォータニオン形式**で扱われます。

```python
# 姿勢を取得（クォータニオン形式）
pos, orn = p.getBasePositionAndOrientation(robot_id)
print(f"姿勢（クォータニオン）: {orn}")  # [x, y, z, w]

# 例: [0.0, 0.0, 0.0, 1.0] = 回転なし（初期姿勢）
# 例: [0.0, 0.0, 0.707, 0.707] = Z軸周りに90度回転
```

### クォータニオンの意味

クォータニオン `[x, y, z, w]` は、以下のように解釈できます：

- **`[x, y, z]`**: 回転軸の方向（正規化されたベクトル）
- **`w`**: 回転角度のコサイン成分

ただし、実際の使用では、**オイラー角に変換して理解する**ことが多いです。

---

## オイラー角（Euler Angles）

### 基本概念

**オイラー角**は、3つの軸（X, Y, Z）周りの回転角度で姿勢を表現する方法です。

- **形式**: `[roll, pitch, yaw]` （3つの浮動小数点値）
- **単位**: ラジアン（radian）
- **用途**: 人間にとって理解しやすい

### オイラー角の3つの角度

PyBulletでは、ROS URDF rpy規則に従って、以下の順序で回転を適用します：

1. **Roll（ロール）**: X軸周りの回転
   - 横転（左右に傾ける）
   - 例：飛行機が左右に傾く

2. **Pitch（ピッチ）**: Y軸周りの回転
   - 縦転（前後に傾ける）
   - 例：飛行機が機首を上げ下げする

3. **Yaw（ヨー）**: Z軸周りの回転
   - 左右の向き（水平面での回転）
   - 例：飛行機が左右を向く

### 回転の順序

PyBulletでは、**ZYX順（Yaw → Pitch → Roll）**で回転を適用します：

```
初期姿勢
  ↓
Z軸周りにYaw回転
  ↓
Y軸周りにPitch回転
  ↓
X軸周りにRoll回転
  ↓
最終姿勢
```

### オイラー角の例

```python
# オイラー角 [roll, pitch, yaw]（ラジアン）
euler = [0.0, 0.0, 0.0]        # 回転なし
euler = [0.0, 0.0, 1.57]       # Z軸周りに90度（約1.57ラジアン）回転
euler = [0.0, 0.785, 0.0]      # Y軸周りに45度（約0.785ラジアン）回転
euler = [1.57, 0.0, 0.0]       # X軸周りに90度回転
```

### 度（Degree）とラジアン（Radian）の変換

```python
import math

# 度 → ラジアン
degrees = 90
radians = math.radians(degrees)  # 約1.57

# ラジアン → 度
radians = 1.57
degrees = math.degrees(radians)  # 約90.0
```

---

## 変換方法

### 1. オイラー角 → クォータニオン

**関数**: `p.getQuaternionFromEuler()`

```python
import pybullet as p
import math

# オイラー角を指定（ラジアン）
euler = [0.0, 0.0, math.radians(90)]  # Z軸周りに90度回転

# クォータニオンに変換
quaternion = p.getQuaternionFromEuler(euler)
print(f"クォータニオン: {quaternion}")  # [x, y, z, w]
```

**使用例：**

```python
# ロボットを90度回転させて配置
startPos = [0, 0, 1.0]
startOrientation = p.getQuaternionFromEuler([0, 0, math.radians(90)])
robot_id = p.loadURDF("r2d2.urdf", startPos, startOrientation)
```

### 2. クォータニオン → オイラー角

**関数**: `p.getEulerFromQuaternion()`

```python
import pybullet as p
import math

# 姿勢を取得（クォータニオン形式）
pos, orn = p.getBasePositionAndOrientation(robot_id)

# オイラー角に変換
euler = p.getEulerFromQuaternion(orn)
print(f"オイラー角（ラジアン）: {euler}")  # [roll, pitch, yaw]

# 度に変換して表示
roll_deg = math.degrees(euler[0])
pitch_deg = math.degrees(euler[1])
yaw_deg = math.degrees(euler[2])
print(f"向き: Roll={roll_deg:.1f}°, Pitch={pitch_deg:.1f}°, Yaw={yaw_deg:.1f}°")
```

**使用例：**

```python
# ロボットの姿勢を確認
pos, orn = p.getBasePositionAndOrientation(robot_id)
euler = p.getEulerFromQuaternion(orn)
print(f"位置: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
print(f"向き: Roll={math.degrees(euler[0]):.1f}°, "
      f"Pitch={math.degrees(euler[1]):.1f}°, "
      f"Yaw={math.degrees(euler[2]):.1f}°")
```

---

## 実践的な使用例

### 例1: ロボットの姿勢を確認する

```python
import pybullet as p
import pybullet_data
import math

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

p.loadURDF("plane.urdf")
robot_id = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 1.0])

# 姿勢を取得
pos, orn = p.getBasePositionAndOrientation(robot_id)

# クォータニオン形式で表示
print(f"位置: {pos}")
print(f"姿勢（クォータニオン）: {orn}")

# オイラー角に変換
euler = p.getEulerFromQuaternion(orn)
print(f"姿勢（オイラー角・ラジアン）: {euler}")

# 度に変換して見やすく表示
print(f"向き:")
print(f"  Roll:  {math.degrees(euler[0]):.2f}° (X軸周り)")
print(f"  Pitch: {math.degrees(euler[1]):.2f}° (Y軸周り)")
print(f"  Yaw:   {math.degrees(euler[2]):.2f}° (Z軸周り)")

p.disconnect()
```

### 例2: ロボットを特定の向きに配置する

```python
import pybullet as p
import pybullet_data
import math

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

p.loadURDF("plane.urdf")

# オイラー角で向きを指定（度からラジアンに変換）
# 例：Z軸周りに45度回転
target_yaw = math.radians(45)  # 45度をラジアンに変換
orientation = p.getQuaternionFromEuler([0, 0, target_yaw])

# ロボットを配置
robot_id = p.loadURDF("r2d2.urdf", 
                      basePosition=[0, 0, 1.0],
                      baseOrientation=orientation)

# 確認
pos, orn = p.getBasePositionAndOrientation(robot_id)
euler = p.getEulerFromQuaternion(orn)
print(f"設定した向き: Yaw={math.degrees(target_yaw):.1f}°")
print(f"実際の向き: Yaw={math.degrees(euler[2]):.1f}°")

# シミュレーションを実行
for i in range(1000):
    p.stepSimulation()

p.disconnect()
```

### 例3: ロボットの向きを動的に変更する

```python
import pybullet as p
import pybullet_data
import math
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

p.loadURDF("plane.urdf")
robot_id = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 1.0])

# シミュレーションループ
for i in range(1000):
    # 現在の姿勢を取得
    pos, orn = p.getBasePositionAndOrientation(robot_id)
    euler = p.getEulerFromQuaternion(orn)
    
    # 10ステップごとに姿勢を表示
    if i % 10 == 0:
        print(f"ステップ {i}:")
        print(f"  位置: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
        print(f"  向き: Yaw={math.degrees(euler[2]):.1f}°")
    
    # 100ステップごとに向きを変更（例）
    if i == 100:
        # Z軸周りに90度回転
        new_orientation = p.getQuaternionFromEuler([0, 0, math.radians(90)])
        p.resetBasePositionAndOrientation(robot_id, pos, new_orientation)
        print("向きを90度回転させました")
    
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
```

### 例4: 複数の回転を組み合わせる

```python
import pybullet as p
import pybullet_data
import math

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

p.loadURDF("plane.urdf")

# 複数の回転を組み合わせる
# Roll=30度, Pitch=45度, Yaw=60度
roll = math.radians(30)
pitch = math.radians(45)
yaw = math.radians(60)

orientation = p.getQuaternionFromEuler([roll, pitch, yaw])

robot_id = p.loadURDF("r2d2.urdf", 
                      basePosition=[0, 0, 1.0],
                      baseOrientation=orientation)

# 確認
pos, orn = p.getBasePositionAndOrientation(robot_id)
euler = p.getEulerFromQuaternion(orn)

print("設定した向き:")
print(f"  Roll:  {math.degrees(roll):.1f}°")
print(f"  Pitch: {math.degrees(pitch):.1f}°")
print(f"  Yaw:   {math.degrees(yaw):.1f}°")

print("\n実際の向き:")
print(f"  Roll:  {math.degrees(euler[0]):.1f}°")
print(f"  Pitch: {math.degrees(euler[1]):.1f}°")
print(f"  Yaw:   {math.degrees(euler[2]):.1f}°")

# シミュレーションを実行
for i in range(1000):
    p.stepSimulation()

p.disconnect()
```

---

## よくある質問

### Q1: クォータニオンとオイラー角、どちらを使うべき？

**A:** 用途によります。

- **クォータニオン**: PyBulletの内部形式。APIで直接使用する場合
- **オイラー角**: 人間が理解しやすい。表示やデバッグに便利

**推奨：**
- PyBulletのAPIにはクォータニオンを渡す
- 表示や理解にはオイラー角に変換する

```python
# ✅ 推奨パターン
pos, orn = p.getBasePositionAndOrientation(robot_id)  # クォータニオンで取得
euler = p.getEulerFromQuaternion(orn)  # オイラー角に変換して表示
print(f"向き: {math.degrees(euler[2]):.1f}°")  # 人間が理解しやすい形式
```

### Q2: クォータニオンの値の意味は？

**A:** クォータニオン `[x, y, z, w]` は、回転軸と回転角度を表現しますが、直接解釈するのは困難です。

**実用的なアプローチ：**
- オイラー角に変換して理解する
- または、`getAxisAngleFromQuaternion()`で軸と角度を取得する

```python
# 軸と角度を取得
axis, angle = p.getAxisAngleFromQuaternion(orn)
print(f"回転軸: {axis}, 回転角度: {math.degrees(angle):.1f}°")
```

### Q3: オイラー角の順序は？

**A:** PyBulletでは、**ROS URDF rpy規則**に従います：

1. **X軸周りのRoll（ロール）**
2. **Y軸周りのPitch（ピッチ）**
3. **Z軸周りのYaw（ヨー）**

ただし、**適用順序は逆**（Z → Y → X）です。

### Q4: 度とラジアンの変換を忘れがちです

**A:** `math.radians()`と`math.degrees()`を使います。

```python
import math

# 度 → ラジアン（PyBulletのAPIに渡す）
degrees = 90
radians = math.radians(degrees)

# ラジアン → 度（表示用）
radians = 1.57
degrees = math.degrees(radians)
```

### Q5: 初期姿勢（回転なし）のクォータニオンは？

**A:** `[0.0, 0.0, 0.0, 1.0]` です。

```python
# 回転なしの姿勢
no_rotation = [0.0, 0.0, 0.0, 1.0]

# または、オイラー角から生成
no_rotation = p.getQuaternionFromEuler([0, 0, 0])
```

### Q6: ジンバルロックとは？

**A:** オイラー角で特定の姿勢を表現できない問題です。

**例：** Pitchが90度の時、RollとYawが同じ効果になってしまう

**解決策：** クォータニオンを使用（PyBulletは内部でクォータニオンを使用しているため、この問題は発生しません）

---

## まとめ

| 項目 | クォータニオン | オイラー角 |
|------|--------------|-----------|
| **形式** | `[x, y, z, w]` | `[roll, pitch, yaw]` |
| **単位** | 無次元 | ラジアン |
| **値の数** | 4つ | 3つ |
| **理解しやすさ** | ❌ 難しい | ✅ 簡単 |
| **ジンバルロック** | ✅ なし | ❌ あり |
| **PyBulletの内部形式** | ✅ 使用 | ❌ 変換が必要 |

**重要なポイント：**

1. ✅ PyBulletは**クォータニオン形式**で姿勢を扱う
2. ✅ **オイラー角に変換**して理解・表示する
3. ✅ **度とラジアンの変換**を忘れない
4. ✅ 回転順序は**Roll → Pitch → Yaw**（適用は逆順）

**実践的な使い方：**

```python
# 1. 姿勢を取得（クォータニオン）
pos, orn = p.getBasePositionAndOrientation(robot_id)

# 2. オイラー角に変換
euler = p.getEulerFromQuaternion(orn)

# 3. 度に変換して表示
print(f"Yaw: {math.degrees(euler[2]):.1f}°")

# 4. 向きを設定する場合は、オイラー角からクォータニオンに変換
new_orientation = p.getQuaternionFromEuler([0, 0, math.radians(90)])
p.resetBasePositionAndOrientation(robot_id, pos, new_orientation)
```

---

## 参考リソース

- **PyBullet公式ドキュメント**: 姿勢の変換について
- **ROS URDF rpy規則**: 回転順序の定義
- **クォータニオンの数学的説明**: より深い理解が必要な場合
