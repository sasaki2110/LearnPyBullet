# PyBullet stepSimulation() の詳細解説

## 質問：`p.stepSimulation()`とは、1ステップ（1/240秒）進めるという意味ですか？

**答え：はい、その通りです！** ただし、もう少し詳しく説明します。

---

## 基本的な意味

### `p.stepSimulation()`の役割

`p.stepSimulation()`は、**物理シミュレーションを1ステップ進める**関数です。

```python
p.stepSimulation()  # シミュレーションを1ステップ進める
```

### 1ステップ = 1/240秒（デフォルト）

- **デフォルトのタイムステップ**: **1/240秒**（約0.00417秒）
- **1回呼ぶと**: シミュレーション時間が約0.00417秒進む
- **240回呼ぶと**: 約1秒のシミュレーション時間が経過

```python
# 1ステップ = 1/240秒 ≈ 0.00417秒
p.stepSimulation()  # 0.00417秒進む

# 240ステップ = 1秒
for i in range(240):
    p.stepSimulation()  # 合計で1秒進む
```

---

## 1ステップで何が起こるか？

公式ドキュメントによると、`stepSimulation()`は以下の処理を**1回の呼び出しで**実行します：

1. **衝突検出（Collision Detection）**
   - オブジェクト同士が接触しているかチェック
   - 接触点の情報を計算

2. **制約解決（Constraint Solving）**
   - 関節の制約を満たすように力を計算
   - 接触力や反力を計算

3. **統合（Integration）**
   - 位置と速度を更新
   - 物理法則に従って物体を移動

### イメージ図

```
ステップ i の状態
├─ 位置: (x_i, y_i, z_i)
├─ 速度: (vx_i, vy_i, vz_i)
└─ 力: (fx_i, fy_i, fz_i)
         ↓
    stepSimulation()
         ↓
ステップ i+1 の状態
├─ 位置: (x_{i+1}, y_{i+1}, z_{i+1})  ← 更新される
├─ 速度: (vx_{i+1}, vy_{i+1}, vz_{i+1})  ← 更新される
└─ 力: (fx_{i+1}, fy_{i+1}, fz_{i+1})  ← 再計算される
```

---

## 具体例で理解する

### 例1: 落下するロボット

```python
import pybullet as p
import pybullet_data

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# ロボットを高さ1mに配置
robot_id = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 1.0])

# ステップ0: 初期状態
pos0, _ = p.getBasePositionAndOrientation(robot_id)
print(f"ステップ0: 高さ = {pos0[2]:.4f}m")  # 1.0000m

# 1ステップ進める（約0.00417秒経過）
p.stepSimulation()

# ステップ1: 重力で少し落下
pos1, _ = p.getBasePositionAndOrientation(robot_id)
print(f"ステップ1: 高さ = {pos1[2]:.4f}m")  # 約0.9999m（少し下がる）

# さらに1ステップ進める
p.stepSimulation()

# ステップ2: さらに落下
pos2, _ = p.getBasePositionAndOrientation(robot_id)
print(f"ステップ2: 高さ = {pos2[2]:.4f}m")  # 約0.9998m
```

**実行結果のイメージ：**
```
ステップ0: 高さ = 1.0000m  ← 初期位置
ステップ1: 高さ = 0.9999m  ← 0.00417秒後、少し落下
ステップ2: 高さ = 0.9998m  ← さらに0.00417秒後、さらに落下
...
ステップ50: 高さ = 0.9958m  ← 約0.21秒後
```

### 例2: ループで複数ステップ進める

```python
# 50ステップ進める = 約0.21秒のシミュレーション
for i in range(50):
    p.stepSimulation()
    
    # 10ステップごとに状態を確認
    if i % 10 == 0:
        pos, _ = p.getBasePositionAndOrientation(robot_id)
        time_elapsed = (i + 1) / 240.0  # 経過時間（秒）
        print(f"ステップ {i}: 高さ = {pos[2]:.4f}m, 経過時間 = {time_elapsed:.4f}秒")
```

**実行結果のイメージ：**
```
ステップ 0: 高さ = 1.0000m, 経過時間 = 0.0042秒
ステップ 10: 高さ = 0.9979m, 経過時間 = 0.0458秒
ステップ 20: 高さ = 0.9917m, 経過時間 = 0.0875秒
ステップ 30: 高さ = 0.9814m, 経過時間 = 0.1292秒
ステップ 40: 高さ = 0.9670m, 経過時間 = 0.1708秒
ステップ 50: 高さ = 0.9485m, 経過時間 = 0.2125秒
```

---

## タイムステップの変更

デフォルトは1/240秒ですが、変更することもできます。

```python
# タイムステップを変更（例：120Hzに変更）
p.setTimeStep(1.0 / 120.0)  # 1ステップ = 1/120秒 ≈ 0.0083秒

# この場合、1ステップで約0.0083秒進む
p.stepSimulation()  # 0.0083秒進む

# 120ステップで1秒
for i in range(120):
    p.stepSimulation()  # 合計で1秒進む
```

**注意：**
- タイムステップを大きくすると、計算が速くなるが精度が下がる
- タイムステップを小さくすると、精度が上がるが計算が遅くなる
- デフォルトの1/240秒は、精度と速度のバランスが良い

---

## よくある誤解

### 誤解1: `stepSimulation()`は時間を止める？

**違います。** `stepSimulation()`は時間を**進める**関数です。

```python
# ❌ 間違い: 時間を止める
p.stepSimulation()  # これは時間を進める！

# ✅ 正しい理解: 時間を1ステップ進める
p.stepSimulation()  # 約0.00417秒進む
```

### 誤解2: `stepSimulation()`を呼ばないと何も起こらない？

**その通りです。** `stepSimulation()`を呼ばない限り、物理シミュレーションは進みません。

```python
# ロボットを配置
robot_id = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 1.0])

# stepSimulation()を呼ばない
# → ロボットは動かない（時間が進まない）

# stepSimulation()を呼ぶ
p.stepSimulation()
# → ロボットが落下し始める（時間が進む）
```

### 誤解3: `stepSimulation()`は1秒進める？

**違います。** デフォルトでは約0.00417秒（1/240秒）進めます。

```python
# ❌ 間違い: 1秒進む
p.stepSimulation()  # これは1/240秒（約0.00417秒）進む

# ✅ 1秒進めるには240回呼ぶ必要がある
for i in range(240):
    p.stepSimulation()  # 合計で1秒進む
```

---

## まとめ

| 項目 | 説明 |
|------|------|
| **基本動作** | 物理シミュレーションを1ステップ進める |
| **デフォルトの時間** | 1ステップ = 1/240秒 ≈ 0.00417秒 |
| **1ステップで行われる処理** | 衝突検出、制約解決、統合（位置・速度の更新） |
| **240ステップ** | 約1秒のシミュレーション時間 |
| **タイムステップの変更** | `p.setTimeStep()`で変更可能 |

**重要なポイント：**
1. ✅ `p.stepSimulation()`は1ステップ（デフォルトで1/240秒）進める
2. ✅ 1回呼ぶと、約0.00417秒のシミュレーション時間が経過
3. ✅ 衝突検出、制約解決、統合が1回の呼び出しで実行される
4. ✅ ループで複数回呼ぶことで、長時間のシミュレーションが可能

---

## 参考：公式ドキュメントより

> stepSimulationは、衝突検出、制約解決、統合などの単一の順動力学シミュレーションステップですべてのアクションを実行します。デフォルトのタイムステップは1/240秒で、setTimeStepまたはsetPhysicsEngineParameter APIを使用して変更できます。

（出典: PyBullet Quickstart Guide）
