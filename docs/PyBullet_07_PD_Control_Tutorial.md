# PyBullet PD制御 完全理解チュートリアル

## はじめに

このチュートリアルでは、PD制御の概念を**視覚的に**、**実践的に**理解できるように、実際に動作するコードとGUIでの確認方法を提供します。

## 目次

1. [PD制御とは何か？](#1-pd制御とは何か)
2. [簡単な例：1つのジョイントを動かす](#2-簡単な例1つのジョイントを動かす)
3. [P制御（比例制御）を理解する](#3-p制御比例制御を理解する)
4. [D制御（微分制御）を理解する](#4-d制御微分制御を理解する)
5. [PD制御を組み合わせる](#5-pd制御を組み合わせる)
6. [パラメータ調整の実践](#6-パラメータ調整の実践)
7. [Vision60での実例](#7-vision60での実例)

---

## 1. PD制御とは何か？

### 1.1 基本的な考え方

PD制御は、**目標値と現在値の差（誤差）**を使って、**どれだけ強く動かすか**を決める制御方法です。

**例：ドアを閉める動作**
- **目標**: ドアを90度閉じる
- **現在**: ドアが10度開いている
- **誤差**: 90度 - 10度 = 80度
- **動作**: 誤差が大きいので、強く押す

### 1.2 PD制御の2つの要素

#### P制御（比例制御）
- **P** = **Proportional**（比例）
- **考え方**: 誤差が大きいほど、強く動かす
- **例**: 目標まで80度離れている → 強く押す
- **例**: 目標まで10度離れている → 弱く押す

#### D制御（微分制御）
- **D** = **Derivative**（微分）
- **考え方**: 動きが速すぎる（振動している） → ブレーキをかける
- **例**: ドアが勢いよく閉まっている → 少しブレーキをかける
- **例**: ドアがゆっくり動いている → ブレーキは不要

### 1.3 PD制御の数式（簡単な説明）

```
動かす力 = (Pゲイン × 位置の誤差) + (Dゲイン × 速度の誤差)
```

**日本語で言うと：**
- 位置の誤差が大きい → 強く動かす（P制御）
- 動きが速すぎる → ブレーキをかける（D制御）

---

## 2. 簡単な例：1つのジョイントを動かす

### 2.1 最も簡単なコード

まず、1つのジョイントを動かす最も簡単なコードを見てみましょう。

**重要**: このコードは**Franka Pandaロボットアーム**を使用し、**往復運動**を行います。大きなアームが動くので、GUIで視覚的に確認しやすくなっています。

```python
import pybullet as p
import pybullet_data
import time
import math

# PyBulletを起動（GUIモード）
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# 地面をロード
plane_id = p.loadURDF("plane.urdf")

# Franka Pandaロボットアームをロード（ベースを固定）
robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

# 初期状態を安定させる
for _ in range(50):
    p.stepSimulation()
    time.sleep(1.0/240.0)

# Franka Pandaの最初のジョイント（肩の回転）を使用
movable_joint = 0  # 最初のジョイント（肩の回転）

# 初期角度を取得
initial_state = p.getJointState(robot_id, movable_joint)
initial_angle = initial_state[0]

# カメラをロボットが見える位置に設定
robot_pos, _ = p.getBasePositionAndOrientation(robot_id)
p.resetDebugVisualizerCamera(
    cameraDistance=1.5,
    cameraYaw=45.0,
    cameraPitch=-30.0,
    cameraTargetPosition=robot_pos
)

# PD制御のパラメータ（動きを遅くするため、Pゲインを小さく）
position_gain = 0.2   # Pゲイン（小さくして動きを遅く）
velocity_gain = 1.0   # Dゲイン
max_force = 50.0      # 最大の力

# 往復運動の設定
min_angle = initial_angle  # 最小角度（初期角度）
max_angle = initial_angle + math.radians(90)  # 最大角度（+90度）
cycle_duration = 2000  # 1往復にかけるステップ数（約8秒）

# シミュレーションループ（往復運動）
for i in range(cycle_duration * 3):  # 3往復
    # 往復運動の目標角度を計算（サイン波）
    progress = (i % cycle_duration) / cycle_duration
    sine_value = math.sin(progress * 2 * math.pi)
    normalized = (sine_value + 1.0) / 2.0
    target_angle = min_angle + normalized * (max_angle - min_angle)
    
    # ジョイントを制御
    p.setJointMotorControl2(
        bodyIndex=robot_id,
        jointIndex=movable_joint,
        controlMode=p.POSITION_CONTROL,
        targetPosition=target_angle,
        positionGain=position_gain,
        velocityGain=velocity_gain,
        force=max_force
    )
    
    # 現在の角度を取得
    joint_state = p.getJointState(robot_id, movable_joint)
    current_angle = joint_state[0]
    current_velocity = joint_state[1]
    
    # ログ出力（50ステップごと）
    if i % 50 == 0:
        error = target_angle - current_angle
        current_angle_deg = math.degrees(current_angle)
        target_angle_deg = math.degrees(target_angle)
        error_deg = math.degrees(error)
        
        cycle_num = (i // cycle_duration) + 1
        cycle_progress = (i % cycle_duration) / cycle_duration * 100
        
        print(f"ステップ {i:4d} (往復{cycle_num}回目, {cycle_progress:5.1f}%): "
              f"角度={current_angle_deg:7.2f}° / {target_angle_deg:7.2f}°, "
              f"誤差={error_deg:7.2f}°, 速度={current_velocity:.3f} rad/s")
    
    # シミュレーションを1ステップ進める
    p.stepSimulation()
    time.sleep(1.0/240.0)

# 終了
input("Enterキーを押すと終了します...")
p.disconnect()
```

**このコードの特徴：**
- **Franka Pandaロボットアーム**を使用：大きなアームが動くので、視覚的に確認しやすい
- **往復運動**：0度 → 90度 → 0度 を3回繰り返すので、動きがずっと見える
- **Pゲインを小さく設定**（0.2）：動きを遅くして、PD制御の動作を観察しやすくしている
- **ログ出力**：角度を度（°）で表示し、目標角度との誤差も表示

### 2.2 実行方法

**方法1: 基本的なスクリプトを実行（推奨）**

すでに用意されている基本的なスクリプトを実行します：

```bash
cd /root/LearnPyBullet
python srcs/pd_tutorial_basic.py
```

このスクリプトは、2.1のコードをそのまま実行できる形で用意されています。

**方法2: 実践用スクリプトを使用**

より詳しいチュートリアル（P制御、D制御の個別テストなど）を実行する場合：

```bash
cd /root/LearnPyBullet
python srcs/pd_control_tutorial.py
```

このスクリプトを実行すると、チュートリアル1から順番に実行できます。

**方法3: 自分でコードを書く**

2.1のコードをコピーして、自分でファイルを作成することもできます。

### 2.3 何が起こるか？

1. **GUIが開く**: Franka Pandaロボットアームが表示される
2. **アームが往復運動する**: 0度 → 90度 → 0度 を3回繰り返す
3. **ログが出力される**: 50ステップごとに現在の角度、目標角度、誤差、速度が表示される

**観察ポイント：**
- **GUIで確認**: 大きなアームが往復運動する様子が視覚的に確認できる
- **ログで確認**: 角度が0°→90°→0°と変化する様子が数値で確認できる
- **PD制御の動作**:
  - 目標角度に近づくと誤差が小さくなる → ゆっくり動く
  - 目標角度から離れると誤差が大きくなる → 速く動く
  - Dゲインにより、動きが滑らかになる（振動が抑制される）

**なぜ往復運動にするのか？**
- 一度だけ目標角度に到達すると、その後は静止してしまう
- 往復運動にすることで、PD制御の動作を継続的に観察できる
- 動きがずっと見えるので、視覚的に理解しやすい

---

## 3. P制御（比例制御）を理解する

### 3.1 P制御とは？

**P制御**は、**位置の誤差**に比例して力を加える制御です。

```
力 = Pゲイン × 位置の誤差
```

**例：**
- 誤差が10度 → Pゲイン0.2 → 力 = 0.2 × 10 = 2.0
- 誤差が5度 → Pゲイン0.2 → 力 = 0.2 × 5 = 1.0
- 誤差が1度 → Pゲイン0.2 → 力 = 0.2 × 1 = 0.2

### 3.2 Pゲインを変えてみる

以下のコードで、Pゲインの効果を確認できます。

```python
import pybullet as p
import pybullet_data
import time
import math

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

plane_id = p.loadURDF("plane.urdf")
robot_id = p.loadURDF("r2d2.urdf", [0, 0, 1])

target_angle = 1.57  # 90度

# Pゲインを変えて試す
p_gains = [0.1, 0.5, 1.0, 2.0]  # 異なるPゲイン

for p_gain in p_gains:
    print(f"\n{'='*60}")
    print(f"Pゲイン = {p_gain} でテスト")
    print(f"{'='*60}")
    
    # ジョイントを0度にリセット
    p.resetJointState(robot_id, 0, targetValue=0.0, targetVelocity=0.0)
    time.sleep(0.5)
    
    for i in range(500):
        p.setJointMotorControl2(
            robot_id, 0,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_angle,
            positionGain=p_gain,      # Pゲインを変更
            velocityGain=0.0,         # Dゲインは0（P制御のみ）
            force=30.0
        )
        
        if i % 50 == 0:
            joint_state = p.getJointState(robot_id, 0)
            current_angle = joint_state[0]
            current_velocity = joint_state[1]
            error = target_angle - current_angle
            
            # 度に変換して表示
            error_deg = math.degrees(error)
            current_angle_deg = math.degrees(current_angle)
            
            print(f"  ステップ {i:3d}: 角度={current_angle_deg:6.1f}°, "
                  f"誤差={error_deg:6.1f}°, 速度={current_velocity:.3f} rad/s")
        
        p.stepSimulation()
        time.sleep(1.0/240.0)
    
    print(f"  → Pゲイン={p_gain}の場合: ", end="")
    final_state = p.getJointState(robot_id, 0)
    final_angle = math.degrees(final_state[0])
    final_error = math.degrees(target_angle - final_state[0])
    print(f"最終角度={final_angle:.1f}°, 最終誤差={final_error:.1f}°")
    
    time.sleep(1.0)  # 次のテスト前に少し待つ

print("\n" + "="*60)
print("テスト完了！")
print("="*60)
print("\n観察ポイント：")
print("  - Pゲインが小さい → ゆっくり動く、目標に到達しにくい")
print("  - Pゲインが大きい → 速く動く、振動する可能性がある")
print("\nEnterキーを押すと終了します...")
input()
p.disconnect()
```

### 3.3 実行結果の見方

**Pゲイン = 0.1 の場合：**
- 動きが**ゆっくり**
- 目標に到達するのに**時間がかかる**
- 振動は少ない

**Pゲイン = 0.5 の場合：**
- 動きが**適度な速度**
- 目標に**到達できる**
- 振動は少ない

**Pゲイン = 1.0 の場合：**
- 動きが**速い**
- 目標に**素早く到達**
- 少し振動する可能性がある

**Pゲイン = 2.0 の場合：**
- 動きが**非常に速い**
- 目標を**通り過ぎる**（オーバーシュート）
- **振動する**

### 3.4 P制御だけの問題点

P制御だけでは：
- ✅ 目標に近づくことができる
- ❌ 目標を**通り過ぎる**（オーバーシュート）
- ❌ **振動する**可能性がある
- ❌ 目標に**到達しない**可能性がある（定常誤差）

**解決策**: D制御を追加する！

---

## 4. D制御（微分制御）を理解する

### 4.1 D制御とは？

**D制御**は、**速度の誤差**に比例してブレーキをかける制御です。

```
ブレーキ = Dゲイン × 速度の誤差
```

**例：**
- 速度が速い（2.0 rad/s） → Dゲイン1.0 → ブレーキ = 1.0 × 2.0 = 2.0
- 速度が遅い（0.5 rad/s） → Dゲイン1.0 → ブレーキ = 1.0 × 0.5 = 0.5
- 速度が0 → ブレーキ = 0（ブレーキ不要）

### 4.2 Dゲインを変えてみる

以下のコードで、Dゲインの効果を確認できます。

```python
import pybullet as p
import pybullet_data
import time
import math

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

plane_id = p.loadURDF("plane.urdf")
robot_id = p.loadURDF("r2d2.urdf", [0, 0, 1])

target_angle = 1.57  # 90度
p_gain = 1.0  # Pゲインは固定

# Dゲインを変えて試す
d_gains = [0.0, 0.5, 1.0, 2.0]  # 異なるDゲイン

for d_gain in d_gains:
    print(f"\n{'='*60}")
    print(f"Dゲイン = {d_gain} でテスト（Pゲイン = {p_gain}）")
    print(f"{'='*60}")
    
    # ジョイントを0度にリセット
    p.resetJointState(robot_id, 0, targetValue=0.0, targetVelocity=0.0)
    time.sleep(0.5)
    
    max_velocity = 0.0  # 最大速度を記録
    
    for i in range(500):
        p.setJointMotorControl2(
            robot_id, 0,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_angle,
            positionGain=p_gain,
            velocityGain=d_gain,      # Dゲインを変更
            force=30.0
        )
        
        joint_state = p.getJointState(robot_id, 0)
        current_velocity = abs(joint_state[1])
        max_velocity = max(max_velocity, current_velocity)
        
        if i % 50 == 0:
            current_angle = joint_state[0]
            error = target_angle - current_angle
            error_deg = math.degrees(error)
            current_angle_deg = math.degrees(current_angle)
            
            print(f"  ステップ {i:3d}: 角度={current_angle_deg:6.1f}°, "
                  f"誤差={error_deg:6.1f}°, 速度={current_velocity:.3f} rad/s")
        
        p.stepSimulation()
        time.sleep(1.0/240.0)
    
    print(f"  → Dゲイン={d_gain}の場合: 最大速度={max_velocity:.3f} rad/s")
    final_state = p.getJointState(robot_id, 0)
    final_angle = math.degrees(final_state[0])
    final_error = math.degrees(target_angle - final_state[0])
    print(f"    最終角度={final_angle:.1f}°, 最終誤差={final_error:.1f}°")
    
    time.sleep(1.0)

print("\n" + "="*60)
print("テスト完了！")
print("="*60)
print("\n観察ポイント：")
print("  - Dゲインが0 → 振動する、目標を通り過ぎる")
print("  - Dゲインが小さい → 少し振動する")
print("  - Dゲインが適切 → 滑らかに目標に到達")
print("  - Dゲインが大きすぎる → 動きが鈍い")
print("\nEnterキーを押すと終了します...")
input()
p.disconnect()
```

### 4.3 実行結果の見方

**Dゲイン = 0.0 の場合：**
- **振動する**
- 目標を**通り過ぎる**（オーバーシュート）
- 最大速度が**大きい**

**Dゲイン = 0.5 の場合：**
- 少し**振動する**
- 目標に**近づく**

**Dゲイン = 1.0 の場合：**
- **滑らかに**目標に到達
- 振動が**少ない**
- 最大速度が**適切**

**Dゲイン = 2.0 の場合：**
- 動きが**鈍い**
- 目標に到達するのに**時間がかかる**
- 最大速度が**小さい**

### 4.4 D制御の効果

D制御を追加すると：
- ✅ **振動を抑制**できる
- ✅ **オーバーシュートを防ぐ**ことができる
- ✅ **滑らかな動き**になる
- ❌ 大きすぎると動きが**鈍くなる**

---

## 5. PD制御を組み合わせる

### 5.1 PD制御の完全な式

```
動かす力 = (Pゲイン × 位置の誤差) + (Dゲイン × 速度の誤差)
```

**日本語で言うと：**
- **位置の誤差が大きい** → 強く動かす（P制御）
- **動きが速すぎる** → ブレーキをかける（D制御）

### 5.2 PD制御の動作を可視化する

以下のコードで、PD制御の動作を詳しく観察できます。

```python
import pybullet as p
import pybullet_data
import time
import math

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

plane_id = p.loadURDF("plane.urdf")
robot_id = p.loadURDF("r2d2.urdf", [0, 0, 1])

target_angle = 1.57  # 90度

# PD制御のパラメータ
p_gain = 0.5
d_gain = 1.0
max_force = 30.0

print("="*80)
print("PD制御の動作を観察します")
print("="*80)
print(f"目標角度: {math.degrees(target_angle):.1f}°")
print(f"Pゲイン: {p_gain}")
print(f"Dゲイン: {d_gain}")
print("="*80)
print("\nステップ | 現在角度 | 位置誤差 | 現在速度 | 速度誤差 | P項 | D項 | 合計力")
print("-"*80)

# ジョイントを0度にリセット
p.resetJointState(robot_id, 0, targetValue=0.0, targetVelocity=0.0)
time.sleep(0.5)

for i in range(500):
    # 現在の状態を取得
    joint_state = p.getJointState(robot_id, 0)
    current_angle = joint_state[0]
    current_velocity = joint_state[1]
    
    # 誤差を計算
    position_error = target_angle - current_angle
    velocity_error = 0.0 - current_velocity  # 目標速度は0
    
    # PD制御の計算
    p_term = p_gain * position_error
    d_term = d_gain * velocity_error
    total_force = p_term + d_term
    
    # 制御を適用
    p.setJointMotorControl2(
        robot_id, 0,
        controlMode=p.POSITION_CONTROL,
        targetPosition=target_angle,
        positionGain=p_gain,
        velocityGain=d_gain,
        force=max_force
    )
    
    # 20ステップごとに詳細を表示
    if i % 20 == 0:
        print(f"{i:6d} | {math.degrees(current_angle):7.2f}° | "
              f"{math.degrees(position_error):7.2f}° | "
              f"{current_velocity:7.3f} | {velocity_error:7.3f} | "
              f"{p_term:5.2f} | {d_term:5.2f} | {total_force:6.2f}")
    
    p.stepSimulation()
    time.sleep(1.0/240.0)

print("-"*80)
final_state = p.getJointState(robot_id, 0)
final_angle = math.degrees(final_state[0])
final_error = math.degrees(target_angle - final_state[0])
print(f"\n最終結果: 角度={final_angle:.2f}°, 誤差={final_error:.2f}°")

print("\n" + "="*80)
print("観察ポイント：")
print("  1. 最初は位置誤差が大きい → P項が大きい → 強く動かす")
print("  2. 動きが速い → 速度誤差が大きい → D項が大きい → ブレーキをかける")
print("  3. 目標に近づくと位置誤差が小さくなる → P項が小さくなる")
print("  4. 速度が遅くなると速度誤差が小さくなる → D項が小さくなる")
print("  5. 最終的に目標角度に到達する")
print("="*80)
print("\nEnterキーを押すと終了します...")
input()
p.disconnect()
```

### 5.3 実行結果の見方

**ステップ 0（開始時）：**
- 位置誤差: 90.0°（大きい）
- 速度: 0.0（静止）
- P項: 大きい（強く動かす）
- D項: 0.0（ブレーキ不要）
- **結果**: 強く動き始める

**ステップ 100（中間）：**
- 位置誤差: 30.0°（中程度）
- 速度: 1.5 rad/s（速い）
- P項: 中程度
- D項: -1.5（ブレーキをかける）
- **結果**: 速度を抑えながら動く

**ステップ 200（目標に近い）：**
- 位置誤差: 5.0°（小さい）
- 速度: 0.3 rad/s（遅い）
- P項: 小さい
- D項: -0.3（少しブレーキ）
- **結果**: ゆっくり目標に近づく

**ステップ 500（最終）：**
- 位置誤差: 0.1°（ほぼ0）
- 速度: 0.0（静止）
- P項: ほぼ0
- D項: 0.0
- **結果**: 目標角度に到達

---

## 6. パラメータ調整の実践

### 6.1 インタラクティブな調整ツール

以下のコードで、リアルタイムでパラメータを調整できます。

```python
import pybullet as p
import pybullet_data
import time
import math

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

plane_id = p.loadURDF("plane.urdf")
robot_id = p.loadURDF("r2d2.urdf", [0, 0, 1])

# 初期パラメータ
p_gain = 0.5
d_gain = 1.0
target_angle = 1.57  # 90度

print("="*80)
print("PD制御パラメータ調整ツール")
print("="*80)
print("現在のパラメータ:")
print(f"  Pゲイン: {p_gain}")
print(f"  Dゲイン: {d_gain}")
print(f"  目標角度: {math.degrees(target_angle):.1f}°")
print("\n操作方法:")
print("  'p'キー: Pゲインを増やす（+0.1）")
print("  'P'キー: Pゲインを減らす（-0.1）")
print("  'd'キー: Dゲインを増やす（+0.1）")
print("  'D'キー: Dゲインを減らす（-0.1）")
print("  'r'キー: リセット（0度に戻す）")
print("  'q'キー: 終了")
print("="*80)

# キーボード入力の準備（簡易版：実際にはGUIのスライダーを使う方が良い）
# ここでは、パラメータを変更してテストする方法を示す

test_configs = [
    {"p": 0.1, "d": 0.0, "name": "Pのみ（小さい）"},
    {"p": 0.5, "d": 0.0, "name": "Pのみ（中程度）"},
    {"p": 1.0, "d": 0.0, "name": "Pのみ（大きい）"},
    {"p": 0.5, "d": 0.5, "name": "PD（D小さい）"},
    {"p": 0.5, "d": 1.0, "name": "PD（D適切）"},
    {"p": 0.5, "d": 2.0, "name": "PD（D大きい）"},
]

for config in test_configs:
    p_gain = config["p"]
    d_gain = config["d"]
    
    print(f"\n{'='*80}")
    print(f"テスト: {config['name']} (P={p_gain}, D={d_gain})")
    print(f"{'='*80}")
    
    # リセット
    p.resetJointState(robot_id, 0, targetValue=0.0, targetVelocity=0.0)
    time.sleep(0.3)
    
    max_error = 0.0
    max_velocity = 0.0
    oscillation_count = 0
    prev_error = None
    
    for i in range(500):
        p.setJointMotorControl2(
            robot_id, 0,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_angle,
            positionGain=p_gain,
            velocityGain=d_gain,
            force=30.0
        )
        
        joint_state = p.getJointState(robot_id, 0)
        current_angle = joint_state[0]
        current_velocity = joint_state[1]
        
        error = target_angle - current_angle
        max_error = max(max_error, abs(error))
        max_velocity = max(max_velocity, abs(current_velocity))
        
        # 振動を検出（誤差の符号が変わる回数をカウント）
        if prev_error is not None:
            if (prev_error > 0 and error < 0) or (prev_error < 0 and error > 0):
                oscillation_count += 1
        prev_error = error
        
        if i % 100 == 0:
            print(f"  ステップ {i:3d}: 角度={math.degrees(current_angle):6.1f}°, "
                  f"誤差={math.degrees(error):6.1f}°, 速度={current_velocity:.3f} rad/s")
        
        p.stepSimulation()
        time.sleep(1.0/240.0)
    
    final_state = p.getJointState(robot_id, 0)
    final_error = abs(target_angle - final_state[0])
    
    print(f"  結果: 最大誤差={math.degrees(max_error):.1f}°, "
          f"最大速度={max_velocity:.3f} rad/s, "
          f"振動回数={oscillation_count}, "
          f"最終誤差={math.degrees(final_error):.2f}°")
    
    time.sleep(1.0)

print("\n" + "="*80)
print("すべてのテスト完了！")
print("="*80)
print("\n最適なパラメータの見つけ方：")
print("  1. Pゲインを調整して、目標に到達できるようにする")
print("  2. Dゲインを調整して、振動を抑制する")
print("  3. 両方を微調整して、滑らかな動きにする")
print("\nEnterキーを押すと終了します...")
input()
p.disconnect()
```

### 6.2 パラメータ調整のコツ

1. **まずPゲインを調整**
   - 小さく始める（0.1）
   - 徐々に大きくする（0.2, 0.5, 1.0...）
   - 目標に到達できる最小の値を見つける

2. **次にDゲインを調整**
   - Pゲインを固定したまま
   - Dゲインを0から始める
   - 振動がなくなるまで増やす（0.5, 1.0, 2.0...）

3. **微調整**
   - PゲインとDゲインを少しずつ調整
   - 滑らかで速い動きを目指す

---

## 7. Vision60での実例

### 7.1 Vision60のPD制御パラメータ

Vision60では、以下のパラメータを使用しています：

```python
# config.py
POSITION_GAIN: float = 0.2   # Pゲイン
VELOCITY_GAIN: float = 2.0   # Dゲイン

# 立ち上がり中は弱く
STANDING_UP_POSITION_GAIN_MULTIPLIER: float = 0.5  # 0.2 × 0.5 = 0.1
STANDING_UP_VELOCITY_GAIN_MULTIPLIER: float = 0.5  # 2.0 × 0.5 = 1.0
```

### 7.2 Vision60でのPD制御の動作確認

以下のコードで、Vision60の実際のPD制御を観察できます：

```python
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'agents', 'p08_vision60', 'my_agent'))

from utils.config import config
from utils.pybullet_env import PyBulletEnvironment
from utils.robot_model import RobotModel
from utils.state import Vision60State
from utils.joint_control import JointController
import pybullet as p
import time
import math

# 環境初期化
env = PyBulletEnvironment()
env.create_environment()
env.load_plane()
env.load_robot()

robot_model = RobotModel(env.robot_id)
state = Vision60State(env.robot_id)
joint_controller = JointController(robot_model, state)

# 初期姿勢を設定
joint_controller.set_initial_pose()

print("="*80)
print("Vision60のPD制御を観察します")
print("="*80)
print(f"Pゲイン: {config.POSITION_GAIN}")
print(f"Dゲイン: {config.VELOCITY_GAIN}")
print("="*80)

# 1つのジョイント（front_leftのknee）を観察
target_joint_idx = 2  # front_leftのknee
target_angle = 1.7  # 約97度

# 目標角度を設定
state.standing_angles['front_left'][2] = target_angle

print(f"\nジョイント {target_joint_idx} (front_left knee) を {math.degrees(target_angle):.1f}° に動かします")
print("\nステップ | 現在角度 | 位置誤差 | 現在速度 | P項 | D項")
print("-"*80)

for i in range(1000):
    # ジョイント制御を適用
    joint_controller.apply_joint_control(is_standing_up=False)
    
    # 現在の状態を取得
    joint_state = p.getJointState(env.robot_id, target_joint_idx)
    current_angle = joint_state[0]
    current_velocity = joint_state[1]
    
    # 誤差を計算
    position_error = target_angle - current_angle
    velocity_error = 0.0 - current_velocity
    
    # PD制御の計算
    p_term = config.POSITION_GAIN * position_error
    d_term = config.VELOCITY_GAIN * velocity_error
    
    # 100ステップごとに表示
    if i % 100 == 0:
        print(f"{i:6d} | {math.degrees(current_angle):7.2f}° | "
              f"{math.degrees(position_error):7.2f}° | "
              f"{current_velocity:7.3f} | {p_term:5.2f} | {d_term:5.2f}")
    
    p.stepSimulation()
    time.sleep(config.TIME_STEP)

print("-"*80)
final_state = p.getJointState(env.robot_id, target_joint_idx)
final_angle = math.degrees(final_state[0])
final_error = math.degrees(target_angle - final_state[0])
print(f"\n最終結果: 角度={final_angle:.2f}°, 誤差={final_error:.2f}°")

print("\nEnterキーを押すと終了します...")
input()
env.disconnect()
```

### 7.3 Vision60での観察ポイント

1. **立ち上がり中**: PゲインとDゲインが小さくなる → ゆっくり安定した動き
2. **通常時**: Pゲイン0.2、Dゲイン2.0 → 適度な速度で滑らかな動き
3. **各ジョイント**: 同じPD制御パラメータを使用 → 統一された動き

---

## まとめ

### PD制御の理解チェックリスト

- [ ] P制御（比例制御）の概念を理解した
  - 位置の誤差が大きいほど、強く動かす
- [ ] D制御（微分制御）の概念を理解した
  - 動きが速すぎる場合、ブレーキをかける
- [ ] PD制御の組み合わせを理解した
  - P制御とD制御を組み合わせて、滑らかな動きを実現
- [ ] パラメータ調整の方法を理解した
  - PゲインとDゲインを調整して、最適な動きを見つける
- [ ] Vision60での実例を確認した
  - 実際のロボットでのPD制御の動作を観察

### 次のステップ

1. **実際にコードを実行してみる**
   - このチュートリアルのコードを実行
   - GUIで視覚的に確認
   - ログで数値を確認

2. **パラメータを調整してみる**
   - PゲインとDゲインを変えて、動作の違いを観察
   - 最適なパラメータを見つける

3. **Vision60に応用する**
   - Vision60のPD制御パラメータを調整
   - 立ち上がり動作や歩行動作を改善

---

## 参考資料

- [PyBullet Joint Control Guide](./PyBullet_Joint_Control_Guide.md) - より詳細な技術情報
- [Vision60実装進捗](./p08_vision60_progress.md) - Vision60の実装状況
