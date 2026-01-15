# PyBullet ジョイント制御完全ガイド

## 概要
Vision60のような複雑なロボットを制御するために必要な、PyBulletのジョイント制御の基礎知識を詳しく解説します。

## 目次
1. [PD制御の基礎](#pd制御の基礎)
2. [制御モードの違い](#制御モードの違い)
3. [setJointMotorControl2()の詳細](#setjointmotorcontrol2の詳細)
4. [forceパラメータの理解](#forceパラメータの理解)
5. [実践的な使用例](#実践的な使用例)
6. [よくある問題と解決策](#よくある問題と解決策)

---

## PD制御の基礎

### PD制御とは
PD制御（Proportional-Derivative Control）は、位置誤差と速度誤差に基づいてトルクを計算する制御方式です。

### 制御トルクの計算式
```
τ = Kp × (θ_target - θ_current) + Kd × (ω_target - ω_current)
```

ここで：
- `τ`: 制御トルク（Nm）
- `Kp`: 位置ゲイン（positionGain）
- `Kd`: 速度ゲイン（velocityGain）
- `θ_target`: 目標角度（rad）
- `θ_current`: 現在角度（rad）
- `ω_target`: 目標速度（rad/s、通常は0）
- `ω_current`: 現在速度（rad/s）

### positionGain（Kp）の役割
- **位置誤差に対する比例ゲイン**
- 値が大きいほど、目標角度に素早く到達しようとする
- 大きすぎると振動やオーバーシュートが発生
- 小さすぎると目標角度に到達できない

**推奨値の目安：**
- 小さなロボット（1-5kg）: 0.1 - 0.5
- 中型ロボット（5-15kg）: 0.2 - 1.0
- 大型ロボット（15kg以上）: 0.5 - 2.0

### velocityGain（Kd）の役割
- **速度誤差に対する微分ゲイン**
- 値が大きいほど、動きが安定する（減衰が強くなる）
- 振動を抑制する効果がある
- 大きすぎると動きが鈍くなる

**推奨値の目安：**
- 小さなロボット: 0.5 - 2.0
- 中型ロボット: 1.0 - 3.0
- 大型ロボット: 2.0 - 5.0

### Vision60での設定例
```python
# config.pyでの設定
POSITION_GAIN: float = 0.2  # 位置ゲイン（Kp）
VELOCITY_GAIN: float = 2.0  # 速度ゲイン（Kd）

# 立ち上がり中はより弱いゲインを使用（安定性優先）
STANDING_UP_POSITION_GAIN_MULTIPLIER: float = 0.5  # 0.2 × 0.5 = 0.1
STANDING_UP_VELOCITY_GAIN_MULTIPLIER: float = 0.5  # 2.0 × 0.5 = 1.0
```

---

## 制御モードの違い

### 1. POSITION_CONTROL（位置制御）

**動作：**
- 目標角度（`targetPosition`）に到達するまでPD制御でトルクを生成
- 目標速度は通常0（静止状態を目指す）

**使用例：**
```python
p.setJointMotorControl2(
    bodyIndex=robot_id,
    jointIndex=0,
    controlMode=p.POSITION_CONTROL,
    targetPosition=1.57,  # 90度（ラジアン）
    positionGain=0.2,      # Kp
    velocityGain=2.0,     # Kd
    force=30.0            # 最大トルク（N）
)
```

**特徴：**
- ✅ 目標角度に自動的に到達する
- ✅ 安定した姿勢制御に適している
- ⚠️ 物理的に不可能な姿勢でも強制的に維持しようとする（forceが大きい場合）
- ⚠️ 目標角度に到達できない場合、振動が発生する可能性がある

**適用場面：**
- 姿勢制御（立ち上がり、安定化）
- 目標角度への到達
- 定位置保持

### 2. VELOCITY_CONTROL（速度制御）

**動作：**
- 目標速度（`targetVelocity`）に到達するまでトルクを生成
- 位置は制御しない（速度のみ制御）

**使用例：**
```python
p.setJointMotorControl2(
    bodyIndex=robot_id,
    jointIndex=0,
    controlMode=p.VELOCITY_CONTROL,
    targetVelocity=1.0,    # 1.0 rad/s
    force=30.0            # 最大トルク（N）
)
```

**特徴：**
- ✅ 連続的な動きに適している
- ✅ 位置制御よりも滑らかな動き
- ⚠️ 位置は制御しないため、目標位置に到達できない可能性がある
- ⚠️ `force=0.0`にすると完全に無効化される（フリー状態）

**適用場面：**
- 連続的な動き（歩行、回転）
- 速度を一定に保つ
- ジョイントを無効化する（`force=0.0`）

### 3. TORQUE_CONTROL（トルク制御）

**動作：**
- 直接トルクを指定して制御
- PD制御は使用しない

**使用例：**
```python
# まず、速度モーターを無効化
p.setJointMotorControl2(
    bodyIndex=robot_id,
    jointIndex=0,
    controlMode=p.VELOCITY_CONTROL,
    targetVelocity=0,
    force=0  # 重要：最大力を0に設定
)

# その後、トルク制御を適用
p.setJointMotorControl2(
    bodyIndex=robot_id,
    jointIndex=0,
    controlMode=p.TORQUE_CONTROL,
    force=10.0  # トルク値（Nm）
)
```

**特徴：**
- ✅ 最も柔軟な制御が可能
- ✅ 物理的に正確なシミュレーション
- ⚠️ 制御が複雑（PD制御を自分で実装する必要がある）
- ⚠️ 不安定になりやすい

**適用場面：**
- 高度な制御（力制御、インピーダンス制御）
- 物理的に正確なシミュレーションが必要な場合

---

## setJointMotorControl2()の詳細

### 関数シグネチャ
```python
p.setJointMotorControl2(
    bodyIndex,           # ロボットのID
    jointIndex,          # ジョイントのインデックス
    controlMode,         # 制御モード（POSITION_CONTROL, VELOCITY_CONTROL, TORQUE_CONTROL）
    targetPosition=0,    # 目標角度（POSITION_CONTROL時、ラジアン）
    targetVelocity=0,   # 目標速度（VELOCITY_CONTROL時、rad/s）
    force=0,            # 最大トルク（N）またはトルク値（TORQUE_CONTROL時）
    positionGain=0,     # 位置ゲイン（Kp、POSITION_CONTROL時）
    velocityGain=0,     # 速度ゲイン（Kd、POSITION_CONTROL時）
    maxVelocity=0       # 最大速度（rad/s、オプション）
)
```

### 主要パラメータの説明

#### bodyIndex
- ロボットのID（`p.loadURDF()`の戻り値）

#### jointIndex
- 制御するジョイントのインデックス
- Vision60の場合：
  - `front_left`: [0, 1, 2] (abduction, hip, knee)
  - `front_right`: [4, 5, 6]
  - `back_left`: [8, 9, 10]
  - `back_right`: [12, 13, 14]

#### controlMode
- `p.POSITION_CONTROL`: 位置制御
- `p.VELOCITY_CONTROL`: 速度制御
- `p.TORQUE_CONTROL`: トルク制御

#### targetPosition（POSITION_CONTROL時）
- 目標角度（ラジアン）
- 例：`1.57` = 90度、`3.14` = 180度

#### targetVelocity（VELOCITY_CONTROL時）
- 目標速度（rad/s）
- 例：`1.0` = 1 rad/s = 約57度/秒

#### force
- **最大トルクの制限**（POSITION_CONTROL, VELOCITY_CONTROL時）
- **トルク値そのもの**（TORQUE_CONTROL時）
- 単位：ニュートンメートル（Nm）

**重要：**
- `force`が大きすぎると、物理法則を無視した動きを強制する
- `force`が小さすぎると、目標角度に到達できない
- Vision60（15kg）の場合、各ジョイントで20-30N程度が適切

#### positionGain（POSITION_CONTROL時）
- 位置ゲイン（Kp）
- 推奨値：0.1 - 2.0（ロボットのサイズによる）

#### velocityGain（POSITION_CONTROL時）
- 速度ゲイン（Kd）
- 推奨値：0.5 - 5.0（ロボットのサイズによる）

---

## forceパラメータの理解

### forceパラメータの役割

#### POSITION_CONTROL / VELOCITY_CONTROL時
- **最大トルクの制限**
- PD制御で計算されたトルクが、この値を超えないように制限される
- 実際のトルクは、PD制御の計算結果と`force`の小さい方

**計算式：**
```
τ_calculated = Kp × (θ_target - θ_current) + Kd × (ω_target - ω_current)
τ_actual = min(τ_calculated, force)
```

#### TORQUE_CONTROL時
- **トルク値そのもの**
- この値が直接ジョイントに適用される

### forceパラメータの適切な値

#### ロボットの質量に基づく目安
```
小型ロボット（1-5kg）:    10-20 N
中型ロボット（5-15kg）:   20-30 N
大型ロボット（15kg以上）: 30-50 N
```

#### Vision60の場合
- ロボット質量：15kg
- 各ジョイントの推奨force：
  - abductionジョイント：30-50 N（可動範囲が小さいため）
  - hip/kneeジョイント：20-30 N

**現在の設定：**
```python
# 通常時
current_force = 30.0  # 物理的に現実的な力

# 立ち上がり中
current_force = 50.0  # より強い力が必要
```

### forceパラメータが大きすぎる場合の問題

#### 問題1: 物理法則を無視した動き
```python
# 悪い例：forceが大きすぎる
p.setJointMotorControl2(
    robot_id, 0,
    controlMode=p.POSITION_CONTROL,
    targetPosition=1.57,
    force=200.0  # 大きすぎる！
)
```
- 結果：片足立ちなど、物理的に不可能な姿勢でも強制的に維持される
- 原因：重力を無視できるほど大きなトルクが発生する

#### 問題2: 不安定な動き
- 力が大きすぎると、急激な動きが発生
- 振動やオーバーシュートが発生しやすい

### forceパラメータが小さすぎる場合の問題

#### 問題1: 目標角度に到達できない
```python
# 悪い例：forceが小さすぎる
p.setJointMotorControl2(
    robot_id, 0,
    controlMode=p.POSITION_CONTROL,
    targetPosition=1.57,
    force=5.0  # 小さすぎる！
)
```
- 結果：目標角度に到達できない
- 原因：重力に抵抗できない

#### 問題2: 立ち上がれない
- 立ち上がり動作で、膝を伸ばす力が不足
- ロボットが立ち上がれない

### force=0.0の特殊な意味

#### VELOCITY_CONTROL時
```python
p.setJointMotorControl2(
    robot_id, 0,
    controlMode=p.VELOCITY_CONTROL,
    targetVelocity=0,
    force=0.0  # 完全に無効化
)
```
- **完全に無効化**（フリー状態）
- トルクが一切発生しない
- 重力で自然に動く

#### POSITION_CONTROL時
```python
p.setJointMotorControl2(
    robot_id, 0,
    controlMode=p.POSITION_CONTROL,
    targetPosition=1.57,
    force=0.0  # 無効化されない！
)
```
- **注意：完全には無効化されない**
- PD制御は動作するが、トルクが0に制限される
- 実質的には無効化に近いが、完全ではない

---

## 実践的な使用例

### 例1: 基本的な姿勢制御（Vision60）

```python
import pybullet as p

# ロボットをロード
robot_id = p.loadURDF("vision60.urdf")

# 各ジョイントを目標角度に制御
target_angles = {
    'front_left': [0.0, 0.5, 1.7],    # abduction, hip, knee
    'front_right': [0.0, 0.5, 1.7],
    'back_left': [0.0, -0.2, 1.7],
    'back_right': [0.0, -0.2, 1.7]
}

leg_joints = {
    'front_left': [0, 1, 2],
    'front_right': [4, 5, 6],
    'back_left': [8, 9, 10],
    'back_right': [12, 13, 14]
}

# PD制御パラメータ
position_gain = 0.2
velocity_gain = 2.0
force = 30.0  # 物理的に現実的な力

# 各ジョイントを制御
for leg_name, joint_indices in leg_joints.items():
    angles = target_angles[leg_name]
    for i, joint_idx in enumerate(joint_indices):
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=joint_idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=angles[i],
            positionGain=position_gain,
            velocityGain=velocity_gain,
            force=force
        )
```

### 例2: 立ち上がり中の弱い制御（安定性優先）

```python
# 立ち上がり中は、より弱い力とゲインを使用
if is_standing_up:
    current_force = 50.0  # 立ち上がりには強い力が必要
    current_position_gain = 0.2 * 0.5  # 0.1（弱いゲイン）
    current_velocity_gain = 2.0 * 0.5  # 1.0（弱いゲイン）
else:
    current_force = 30.0  # 通常時
    current_position_gain = 0.2
    current_velocity_gain = 2.0

# 制御を適用
p.setJointMotorControl2(
    bodyIndex=robot_id,
    jointIndex=joint_idx,
    controlMode=p.POSITION_CONTROL,
    targetPosition=target_angle,
    positionGain=current_position_gain,
    velocityGain=current_velocity_gain,
    force=current_force
)
```

### 例3: ジョイントを一時的に無効化

```python
# ジョイントを無効化（フリー状態）
p.setJointMotorControl2(
    bodyIndex=robot_id,
    jointIndex=joint_idx,
    controlMode=p.VELOCITY_CONTROL,
    targetVelocity=0,
    force=0.0  # 完全に無効化
)

# シミュレーションを進める（重力で自然に動く）
p.stepSimulation()

# 再度制御を有効化
p.setJointMotorControl2(
    bodyIndex=robot_id,
    jointIndex=joint_idx,
    controlMode=p.POSITION_CONTROL,
    targetPosition=target_angle,
    positionGain=0.2,
    velocityGain=2.0,
    force=30.0
)
```

### 例4: 弱い力で姿勢を維持（物理的に正しい制御）

```python
# 動かすジョイント以外を弱い力で維持
# （物理的に不可能な姿勢は維持できない）
for other_joint_idx in all_joint_indices:
    if other_joint_idx != target_joint_idx:
        current_angle = p.getJointState(robot_id, other_joint_idx)[0]
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=other_joint_idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=current_angle,
            positionGain=0.2,      # 通常より小さく
            velocityGain=0.5,       # 通常より小さく
            force=15.0              # 通常より小さく（重力に抵抗できる最小限）
        )
```

---

## よくある問題と解決策

### 問題1: 片足立ちなど、物理的に不可能な姿勢が維持される

**原因：**
- `force`パラメータが大きすぎる
- `POSITION_CONTROL`で強制的に姿勢を維持しようとしている

**解決策：**
```python
# forceを小さくする（物理的に現実的な値に）
force = 30.0  # 200.0 → 30.0に削減

# または、弱い力で維持する
force = 15.0  # さらに小さく
positionGain = 0.2  # ゲインも小さく
velocityGain = 0.5
```

### 問題2: 目標角度に到達できない

**原因：**
- `force`パラメータが小さすぎる
- `positionGain`が小さすぎる

**解決策：**
```python
# forceを増やす
force = 50.0  # 30.0 → 50.0に増加

# positionGainを増やす
positionGain = 0.5  # 0.2 → 0.5に増加
```

### 問題3: 振動が発生する

**原因：**
- `positionGain`が大きすぎる
- `velocityGain`が小さすぎる

**解決策：**
```python
# positionGainを小さくする
positionGain = 0.1  # 0.2 → 0.1に削減

# velocityGainを大きくする（減衰を増やす）
velocityGain = 3.0  # 2.0 → 3.0に増加
```

### 問題4: 動きが鈍い

**原因：**
- `velocityGain`が大きすぎる
- `force`が小さすぎる

**解決策：**
```python
# velocityGainを小さくする
velocityGain = 1.0  # 2.0 → 1.0に削減

# forceを増やす
force = 40.0  # 30.0 → 40.0に増加
```

### 問題5: VELOCITY_CONTROLでforce=0.0にすると倒れる

**原因：**
- `force=0.0`で完全に無効化されるため、重力で倒れる

**解決策：**
```python
# 一時的に無効化する場合は、すぐに元に戻す
p.setJointMotorControl2(
    robot_id, joint_idx,
    controlMode=p.VELOCITY_CONTROL,
    targetVelocity=0,
    force=0.0
)

# 短時間のみシミュレーションを進める
for _ in range(10):  # 10ステップのみ
    p.stepSimulation()

# すぐに元の制御に戻す
p.setJointMotorControl2(
    robot_id, joint_idx,
    controlMode=p.POSITION_CONTROL,
    targetPosition=target_angle,
    positionGain=0.2,
    velocityGain=2.0,
    force=30.0
)
```

### 問題6: 構造調査スクリプトで物理法則を無視した動きになる

**原因：**
- 動かすジョイント以外を`force=50.0`で強制的に維持している

**解決策：**
```python
# 弱い力で維持する（物理的に不可能な姿勢は維持できない）
for other_joint_idx in all_joint_indices:
    if other_joint_idx != target_joint_idx:
        current_angle = p.getJointState(robot_id, other_joint_idx)[0]
        p.setJointMotorControl2(
            robot_id, other_joint_idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=current_angle,
            positionGain=0.2,      # 0.5 → 0.2に削減
            velocityGain=0.5,       # 1.0 → 0.5に削減
            force=15.0              # 50.0 → 15.0に削減
        )
```

---

## まとめ

### 重要なポイント

1. **PD制御の理解**
   - `positionGain`（Kp）: 位置誤差に対する比例ゲイン
   - `velocityGain`（Kd）: 速度誤差に対する微分ゲイン
   - 制御トルク = Kp × 位置誤差 + Kd × 速度誤差

2. **制御モードの違い**
   - `POSITION_CONTROL`: 目標角度に到達するまでPD制御
   - `VELOCITY_CONTROL`: 目標速度に到達するまで制御
   - `TORQUE_CONTROL`: 直接トルクを指定

3. **forceパラメータの理解**
   - POSITION_CONTROL/VELOCITY_CONTROL時: 最大トルクの制限
   - TORQUE_CONTROL時: トルク値そのもの
   - 大きすぎると物理法則を無視した動きになる
   - 小さすぎると目標に到達できない

4. **適切な値の選択**
   - Vision60（15kg）の場合：
     - `force`: 20-30 N（通常時）、50 N（立ち上がり中）
     - `positionGain`: 0.2（通常時）、0.1（立ち上がり中）
     - `velocityGain`: 2.0（通常時）、1.0（立ち上がり中）

5. **物理的に正しい制御**
   - `force`を適切な値に設定（大きすぎない）
   - 物理的に不可能な姿勢は維持できないようにする
   - 弱い力で維持する場合は、重力に抵抗できる最小限の力を使用

---

## 参考資料

- [PyBullet公式ドキュメント](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvL3McEa0RwY/edit)
- [PyBullet Robot Learning Plan](./PyBullet_Robot_Learning_Plan.md)
- [Vision60実装進捗](./p08_vision60_progress.md)
