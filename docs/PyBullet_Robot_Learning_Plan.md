# PyBullet ロボット学習プラン

このドキュメントは、`g01_p06_robot.md`と`PyBullet Quickstart Guide.pdf`を参考に、PyBulletにおけるロボットの学習を段階的に進めるための包括的なプランです。

---

## 学習の全体像

PyBulletでのロボット学習は、以下の5つの段階で構成されています：

1. **基礎編：環境構築と基本操作**
2. **構造理解編：ロボットの階層構造と情報取得**
3. **制御編：ロボットの動的制御とセンシング**
4. **応用編：物理特性のカスタマイズと拘束**
5. **高度編：物理エンジンの調整とデータ記録**

各段階には、理論的な理解と実践的な演習が含まれています。

---

## 第1段階：基礎編 - 環境構築と基本操作

### 学習目標
- PyBulletの環境を構築し、基本的な接続方法を理解する
- シミュレーション空間にオブジェクトを配置し、物理演算を実行する
- 座標系と位置・姿勢の操作を習得する

### 1.1 環境構築と接続の理解

**学習項目：**
- `p.connect()` の種類と違い
  - `DIRECT`: GUIなし、高速、クラウド実行に適している
  - `GUI`: 3D可視化、デバッグに便利
  - `SHARED_MEMORY`, `UDP`, `TCP`: 分散実行用
- 接続の確認: `getConnectionInfo()`, `isConnected()`
- タイムアウト設定: `setTimeOut()`

**実践演習：**
```python
import pybullet as p
import pybullet_data

# DIRECTモードで接続（GUIなし）
physicsClient = p.connect(p.DIRECT)
if physicsClient < 0:
    print("接続に失敗しました")
    exit()

# データパスの設定
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 接続情報の確認
info = p.getConnectionInfo(physicsClient)
print(f"接続状態: {info}")

p.disconnect()
```

### 1.2 世界の創造とオブジェクトの配置

**学習項目：**
- 重力の設定: `setGravity(x, y, z)`
- URDFファイルの読み込み: `loadURDF()`
- オブジェクトの配置: `basePosition`, `baseOrientation`
- 標準データの利用: `pybullet_data`

**実践演習：**
```python
import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)  # または p.DIRECT
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 重力の設定（Z軸負方向に9.81m/s²）
p.setGravity(0, 0, -9.81)

# 地面をロード
planeId = p.loadURDF("plane.urdf")

# R2D2を配置（高さ1m）
startPos = [0, 0, 1]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF("r2d2.urdf", startPos, startOrientation)

# シミュレーションを実行
for i in range(1000):
    p.stepSimulation()
    time.sleep(1./240.)  # 240Hz

p.disconnect()
```

### 1.3 座標系と位置・姿勢の操作

**学習項目：**
- 位置・姿勢の取得: `getBasePositionAndOrientation()`
- 位置・姿勢のリセット: `resetBasePositionAndOrientation()`
- クォータニオンとオイラー角の変換
  - `getQuaternionFromEuler()`: オイラー角 → クォータニオン
  - `getEulerFromQuaternion()`: クォータニオン → オイラー角
- 座標変換: `multiplyTransforms()`, `invertTransform()`

**実践演習：**
```python
# 位置と姿勢の取得
pos, orn = p.getBasePositionAndOrientation(robotId)
print(f"位置: {pos}")
print(f"姿勢（クォータニオン）: {orn}")

# オイラー角に変換
euler = p.getEulerFromQuaternion(orn)
print(f"姿勢（オイラー角）: {euler}")

# 位置をリセット
newPos = [1, 0, 1]
newOrn = p.getQuaternionFromEuler([0, 0, 1.57])  # 90度回転
p.resetBasePositionAndOrientation(robotId, newPos, newOrn)
```

### 1.4 時間の進め方と物理演算

**学習項目：**
- シミュレーションステップ: `stepSimulation()`
- タイムステップの設定: `setTimeStep()`
- リアルタイムシミュレーション: `setRealTimeSimulation()`
- 衝突検出: `performCollisionDetection()`

**実践演習：**
```python
# タイムステップの設定（デフォルトは1/240秒）
p.setTimeStep(1./240.)

# シミュレーションループ
for i in range(1000):
    p.stepSimulation()
    # 必要に応じて状態を取得・制御
    
# リアルタイムシミュレーション（GUIモードのみ）
p.setRealTimeSimulation(1)  # 有効化
# stepSimulation()を呼ぶ必要がなくなる
```

---

## 第2段階：構造理解編 - ロボットの階層構造と情報取得

### 学習目標
- ロボットの階層構造（Base, Joints, Links）を理解する
- ジョイント情報を取得し、ロボットの構造を解析する
- ボディ（オブジェクト）の管理方法を習得する

### 2.1 ロボットの階層構造の理解

**重要な概念：**
- **ベース（Base）**: すべてのロボットのルート親、インデックスは `-1`
- **リンク（Link）**: ベース以外の各リンク、インデックスは `0` 以上
- **ジョイント（Joint）**: リンクを接続する関節、**リンク数 = ジョイント数**
- **階層構造**: ツリー状の親子関係

**実践演習：**
```python
# ロボットをロード
robotId = p.loadURDF("panda.urdf", [0, 0, 0])

# ジョイント数の取得
numJoints = p.getNumJoints(robotId)
print(f"ジョイント数: {numJoints}")

# 各ジョイントの情報を取得
for i in range(numJoints):
    jointInfo = p.getJointInfo(robotId, i)
    jointName = jointInfo[1]
    jointType = jointInfo[2]
    parentIndex = jointInfo[16]  # 親リンクのインデックス
    
    print(f"ジョイント {i}: {jointName}")
    print(f"  タイプ: {jointType}")
    print(f"  親リンク: {parentIndex}")
```

### 2.2 ジョイント情報の詳細取得

**学習項目：**
- `getJointInfo()` の戻り値の理解
  - `jointName`: ジョイント名
  - `jointType`: ジョイントタイプ（回転、直動、固定など）
  - `jointLowerLimit`, `jointUpperLimit`: 可動範囲
  - `jointMaxForce`, `jointMaxVelocity`: 性能制限
  - `parentIndex`: 親リンクのインデックス

**実践演習：**
```python
def print_joint_info(robotId, jointIndex):
    """ジョイント情報を詳細に表示"""
    jointInfo = p.getJointInfo(robotId, jointIndex)
    
    print(f"ジョイント {jointIndex}:")
    print(f"  名前: {jointInfo[1]}")
    print(f"  タイプ: {jointInfo[2]}")
    print(f"  可動範囲: [{jointInfo[8]}, {jointInfo[9]}]")
    print(f"  最大力: {jointInfo[10]}")
    print(f"  最大速度: {jointInfo[11]}")
    print(f"  親リンク: {jointInfo[16]}")

# すべてのジョイント情報を表示
for i in range(p.getNumJoints(robotId)):
    print_joint_info(robotId, i)
```

### 2.3 ボディ（オブジェクト）の管理

**学習項目：**
- ボディ数の取得: `getNumBodies()`
- ボディ情報の取得: `getBodyInfo()`
- ボディの削除: `removeBody()`
- 情報の同期: `syncBodyInfo()`

**実践演習：**
```python
# 現在のボディ数を取得
numBodies = p.getNumBodies()
print(f"ボディ数: {numBodies}")

# 各ボディの情報を取得
for i in range(numBodies):
    bodyId = p.getBodyUniqueId(i)
    bodyInfo = p.getBodyInfo(bodyId)
    print(f"ボディ {bodyId}: {bodyInfo[1]}")  # ボディ名

# 不要なボディを削除
# p.removeBody(robotId)
```

---

## 第3段階：制御編 - ロボットの動的制御とセンシング

### 学習目標
- ロボットの関節を制御する方法を習得する
- ロボットの状態を取得し、フィードバック制御を実現する
- 外部から力を加える方法を理解する

### 3.1 モーター制御の基本

**学習項目：**
- 制御モードの種類
  - `POSITION_CONTROL`: 位置制御
  - `VELOCITY_CONTROL`: 速度制御
  - `TORQUE_CONTROL`: トルク制御
- `setJointMotorControl2()`: 単一ジョイントの制御
- `setJointMotorControlArray()`: 複数ジョイントの効率的な制御
- **重要**: デフォルトでは速度モーターが有効。トルク制御には最大力を0に設定

**実践演習：**
```python
# 位置制御
p.setJointMotorControl2(
    bodyIndex=robotId,
    jointIndex=0,
    controlMode=p.POSITION_CONTROL,
    targetPosition=1.57,  # 90度（ラジアン）
    force=100.0
)

# 速度制御
p.setJointMotorControl2(
    bodyIndex=robotId,
    jointIndex=1,
    controlMode=p.VELOCITY_CONTROL,
    targetVelocity=1.0,  # rad/s
    force=50.0
)

# トルク制御（速度モーターを無効化）
p.setJointMotorControl2(
    bodyIndex=robotId,
    jointIndex=2,
    controlMode=p.VELOCITY_CONTROL,
    targetVelocity=0,
    force=0  # 重要：最大力を0に設定
)
p.setJointMotorControl2(
    bodyIndex=robotId,
    jointIndex=2,
    controlMode=p.TORQUE_CONTROL,
    force=10.0  # トルク値
)

# 複数ジョイントの同時制御（効率的）
jointIndices = [0, 1, 2]
targetPositions = [0.5, 1.0, 1.5]
forces = [100.0, 100.0, 100.0]

p.setJointMotorControlArray(
    bodyIndex=robotId,
    jointIndices=jointIndices,
    controlMode=p.POSITION_CONTROL,
    targetPositions=targetPositions,
    forces=forces
)
```

### 3.2 状態のフィードバックとセンシング

**学習項目：**
- ジョイント状態の取得: `getJointState()`
  - 位置、速度、反力・トルク
- リンク状態の取得: `getLinkState()`
  - 重心位置、姿勢、速度
- ベース速度の取得: `getBaseVelocity()`
- 力・トルクセンサ: `enableJointForceTorqueSensor()`

**実践演習：**
```python
# ジョイント状態の取得
jointState = p.getJointState(robotId, 0)
jointPosition = jointState[0]
jointVelocity = jointState[1]
jointReactionForces = jointState[2]  # 反力・トルク
appliedMotorTorque = jointState[3]

print(f"ジョイント0:")
print(f"  位置: {jointPosition} rad")
print(f"  速度: {jointVelocity} rad/s")
print(f"  反力: {jointReactionForces}")

# リンク状態の取得
linkState = p.getLinkState(robotId, 0)
linkWorldPosition = linkState[0]  # 重心位置
linkWorldOrientation = linkState[1]  # 姿勢
linkLinearVelocity = linkState[6]  # 線速度
linkAngularVelocity = linkState[7]  # 角速度

# ベース速度の取得
linearVel, angularVel = p.getBaseVelocity(robotId)
print(f"ベース速度: {linearVel}, 角速度: {angularVel}")

# 力・トルクセンサの有効化
p.enableJointForceTorqueSensor(robotId, 0, enableSensor=True)
# その後、getJointState()で反力を取得可能
```

### 3.3 外部からの干渉

**学習項目：**
- 外部力の適用: `applyExternalForce()`
- 外部トルクの適用: `applyExternalTorque()`
- デカルト座標系での力の適用

**実践演習：**
```python
# ベースに外部力を適用
force = [0, 0, 100]  # Z軸方向に100N
position = [0, 0, 0]  # 力の適用点（ベースの重心）
p.applyExternalForce(
    objectUniqueId=robotId,
    linkIndex=-1,  # -1はベース
    forceObj=force,
    posObj=position,
    flags=p.WORLD_FRAME  # 世界座標系
)

# 特定のリンクに外部トルクを適用
torque = [0, 0, 10]  # Z軸周りに10Nm
p.applyExternalTorque(
    objectUniqueId=robotId,
    linkIndex=0,
    torqueObj=torque,
    flags=p.WORLD_FRAME
)
```

### 3.4 状態のリセット

**学習項目：**
- ジョイント状態のリセット: `resetJointState()`
- ベース速度のリセット: `resetBaseVelocity()`

**実践演習：**
```python
# ジョイント状態を強制的にリセット
p.resetJointState(robotId, 0, targetValue=0.0, targetVelocity=0.0)

# ベース速度をリセット
p.resetBaseVelocity(robotId, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, 0])
```

---

## 第4段階：応用編 - 物理特性のカスタマイズと拘束

### 学習目標
- 物理パラメータを動的に変更する方法を習得する
- 拘束（Constraint）を作成し、複雑なロボット構造を実現する
- 衝突検出と接触情報を活用する

### 4.1 動力学パラメータの変更

**学習項目：**
- `changeDynamics()` による物理パラメータの変更
  - 質量、摩擦、反発係数、減衰係数など

**実践演習：**
```python
# リンクの質量を変更
p.changeDynamics(
    bodyUniqueId=robotId,
    linkIndex=0,
    mass=2.0  # 2kgに変更
)

# 摩擦係数の変更
p.changeDynamics(
    bodyUniqueId=robotId,
    linkIndex=0,
    lateralFriction=0.8,  # 横摩擦
    spinningFriction=0.1,  # 回転摩擦
    rollingFriction=0.1   # 転がり摩擦
)

# 反発係数と減衰係数の変更
p.changeDynamics(
    bodyUniqueId=robotId,
    linkIndex=0,
    restitution=0.5,  # 反発係数（0-1）
    linearDamping=0.1,  # 線形減衰
    angularDamping=0.1   # 角減衰
)
```

### 4.2 拘束（Constraint）の作成

**学習項目：**
- `createConstraint()` による拘束の作成
  - 固定拘束、点対点拘束、ギア接続など
- 拘束の変更: `changeConstraint()`
- 拘束の削除: `removeConstraint()`

**実践演習：**
```python
# 点対点拘束の作成（2つのボディを接続）
constraintId = p.createConstraint(
    parentBodyUniqueId=robotId,
    parentLinkIndex=0,
    childBodyUniqueId=objectId,
    childLinkIndex=-1,
    jointType=p.JOINT_FIXED,  # 固定関節
    jointAxis=[0, 0, 0],
    parentFramePosition=[0, 0, 0],
    childFramePosition=[0, 0, 0]
)

# 拘束のパラメータを変更
p.changeConstraint(
    constraintUniqueId=constraintId,
    maxForce=1000.0
)

# 拘束を削除
# p.removeConstraint(constraintId)
```

### 4.3 衝突検出と接触情報

**学習項目：**
- 接触点の取得: `getContactPoints()`
- 最近接点の取得: `getClosestPoints()`
- 衝突検出の実行: `performCollisionDetection()`

**実践演習：**
```python
# 衝突検出を実行
p.performCollisionDetection()

# 接触点を取得
contactPoints = p.getContactPoints(robotId, objectId)
for point in contactPoints:
    contactPosition = point[5]  # 接触位置
    contactNormal = point[7]  # 接触法線
    contactForce = point[9]  # 接触力
    print(f"接触位置: {contactPosition}")
    print(f"接触力: {contactForce}")

# 最近接点を取得（衝突していなくても）
closestPoints = p.getClosestPoints(
    bodyA=robotId,
    bodyB=objectId,
    maxDistance=0.1  # 最大距離（m）
)
```

---

## 第5段階：高度編 - 物理エンジンの調整とデータ記録

### 学習目標
- 物理エンジンのパラメータを調整し、精度と速度のバランスを最適化する
- シミュレーションの状態を保存・復元する方法を習得する
- データを記録し、後で分析できるようにする

### 5.1 タイムステップと物理エンジンパラメータ

**学習項目：**
- タイムステップの設定: `setTimeStep()`
- 物理エンジンパラメータ: `setPhysicsEngineParameter()`
  - ソルバーの反復回数
  - 接触処理のしきい値
  - ERP（誤差修正パラメータ）

**実践演習：**
```python
# タイムステップの設定（デフォルトは1/240秒）
p.setTimeStep(1./240.)

# 物理エンジンパラメータの調整
p.setPhysicsEngineParameter(
    numSolverIterations=10,  # ソルバーの反復回数（増やすと精度↑、速度↓）
    numSubSteps=1,  # サブステップ数
    fixedTimeStep=1./240.,  # 固定タイムステップ
    solverResidualThreshold=1e-7,  # ソルバー残差のしきい値
    contactBreakingThreshold=0.0001,  # 接触切断しきい値
    enableFileCaching=1  # ファイルキャッシュの有効化
)
```

### 5.2 状態の保存と復元

**学習項目：**
- 状態の保存: `saveState()`, `saveBullet()`
- 状態の復元: `restoreState()`
- 状態の削除: `removeState()`

**実践演習：**
```python
# メモリ内に状態を保存
stateId = p.saveState()

# ディスクに状態を保存
p.saveBullet("simulation_state.bullet")

# 状態を復元（メモリから）
p.restoreState(stateId)

# 状態を復元（ファイルから）
p.restoreState(fileName="simulation_state.bullet")

# 保存した状態を削除
p.removeState(stateId)
```

### 5.3 シミュレーションの記録とデータ保存

**学習項目：**
- 状態ロギング: `startStateLogging()`, `stopStateLogging()`
- シミュレーションのリセット: `resetSimulation()`
- 世界の保存: `saveWorld()`

**実践演習：**
```python
# 状態ロギングの開始
logId = p.startStateLogging(
    loggingType=p.STATE_LOGGING_ROBOT_TIMEOUT,
    fileName="robot_trajectory.bullet",
    robotUniqueId=robotId,
    maxLogDof=7  # 最大自由度
)

# シミュレーションを実行
for i in range(1000):
    p.stepSimulation()
    # ロボットの状態が自動的に記録される

# ロギングを停止
p.stopStateLogging(logId)

# 世界全体をPythonファイルとして保存
p.saveWorld("my_world.py")

# シミュレーションをリセット（すべてのオブジェクトを削除）
p.resetSimulation()
```

---

## 実践プロジェクト例

### プロジェクト1: R2D2の歩行制御
- ホイール関節の速度制御
- カメラによる自動追従
- 動画録画機能

### プロジェクト2: ロボットアームの逆運動学制御
- `calculateInverseKinematics()` の使用
- グリッパーによる物体把持
- 衝突検出と回避

### プロジェクト3: 複数ロボットの協調制御
- 複数のボディの管理
- 通信と同期
- タスクの分散実行

---

## 学習の進め方

### 推奨される学習順序

1. **第1段階**を完全に理解し、Hello Worldプログラムを動作させる
2. **第2段階**で、実際のロボット（r2d2, pandaなど）の構造を解析する
3. **第3段階**で、簡単な制御プログラムを作成する（例：関節を動かす）
4. **第4段階**で、より複雑なシナリオに挑戦する
5. **第5段階**で、最適化とデータ分析を行う

### 各段階でのチェックポイント

- **第1段階**: ロボットを表示し、重力で落下させる
- **第2段階**: ロボットの全ジョイント情報を表示する
- **第3段階**: ロボットの関節を目標位置に移動させる
- **第4段階**: 2つのロボットを拘束で接続する
- **第5段階**: シミュレーションを保存・復元できる

### トラブルシューティング

- **ロボットが動かない**: ジョイントモーターの設定を確認
- **衝突が検出されない**: `performCollisionDetection()`を呼ぶ
- **シミュレーションが不安定**: タイムステップを小さくする、またはソルバー反復回数を増やす
- **メモリ不足**: 不要なオブジェクトを`removeBody()`で削除

---

## 参考リソース

- **PyBullet公式ドキュメント**: https://docs.google.com/document/d/10sXEhzFRSnvFcl3xQ9XPy6bOe5Z7x6Q4
- **PyBullet GitHub**: https://github.com/bulletphysics/bullet3
- **URDFチュートリアル**: http://wiki.ros.org/urdf/Tutorials
- **プロジェクト内のサンプルコード**: `srcs/` ディレクトリ

---

## まとめ

この学習プランに沿って進めることで、PyBulletを使ったロボットシミュレーションの基礎から応用までを段階的に習得できます。各段階で実践的な演習を行うことで、理論と実装の両方を理解できるようになります。

重要なのは、**「情報の取得（get）」と「命令の送信（set/apply）」のサイクル**を意識することです。ロボットの状態を取得し、それに基づいて制御命令を送る、この繰り返しがPyBulletでのロボット制御の基本です。
