"""
Minitaurロボットの歩行パターン評価モジュール

PyBullet公式サンプル: 様々な歩行パターンを生成し、評価する機能を提供
- 歩行パターンの生成関数（8Amplitude8Phase, 2Amplitude4Phase, hop）
- パラメータ評価関数（移動距離とエネルギー消費を考慮）
"""
from minitaur import Minitaur
import pybullet as p
import numpy as np
import time
import sys
import math

# グローバル変数: Minitaurインスタンス
minitaur = None

# 評価関数のマッピング（関数名文字列 -> 関数オブジェクト）
evaluate_func_map = dict()


def current_position():
  """
  現在のロボット位置を取得
  
  Returns:
    位置配列 [x, y, z]
  """
  global minitaur
  position = minitaur.getBasePosition()
  return np.asarray(position)


def is_fallen():
  """
  ロボットが転倒しているかチェック
  
  ベースの姿勢から上方向ベクトルを計算し、
  それが下向き（Z軸と内積が負）の場合、転倒と判定します。
  
  Returns:
    True: 転倒している、False: 正常
  """
  global minitaur
  orientation = minitaur.getBaseOrientation()
  # クォータニオンから回転行列を取得
  rotMat = p.getMatrixFromQuaternion(orientation)
  # 回転行列の最後の3要素がローカル座標系の上方向ベクトル
  localUp = rotMat[6:]
  # ワールド座標系のZ軸（上方向）との内積が負なら転倒
  return np.dot(np.asarray([0, 0, 1]), np.asarray(localUp)) < 0


def evaluate_desired_motorAngle_8Amplitude8Phase(i, params):
  """
  8つの振幅と8つの位相を持つ歩行パターンを生成
  
  各モーターに異なる振幅と位相を設定します。
  パラメータ: 16個（振幅8個 + 位相8個）
  
  Args:
    i: ステップ数
    params: パラメータリスト（16要素）
      - params[0-7]: 各モーターの振幅
      - params[8-15]: 各モーターの位相オフセット
  
  Returns:
    8つのモーターの目標角度リスト（ラジアン）
  """
  nMotors = 8
  speed = 0.35  # 歩行速度
  joint_values = [0] * nMotors  # 初期化
  for jthMotor in range(nMotors):
    # sin波ベースの角度計算: sin(時間 * 速度 + 位相) * 振幅 + 基本角度(π/2)
    joint_values[jthMotor] = math.sin(i * speed +
                                      params[nMotors + jthMotor]) * params[jthMotor] * +1.57
  return joint_values


def evaluate_desired_motorAngle_2Amplitude4Phase(i, params):
  """
  2つの振幅と4つの位相を持つ歩行パターンを生成（公式サンプルで使用）
  
  より少ないパラメータで歩行パターンを生成します。
  パラメータ: 6個
    - params[0]: 振幅1
    - params[1]: 振幅2
    - params[2]: 位相差
    - params[3]: 位相オフセット1
    - params[4]: 位相オフセット2
    - params[5]: 位相オフセット3
  
  Args:
    i: ステップ数
    params: パラメータリスト（6要素）
  
  Returns:
    8つのモーターの目標角度リスト（ラジアン）
    順序: [front_left_L, front_left_R, back_left_L, back_left_R,
           front_right_L, front_right_R, back_right_L, back_right_R]
  """
  speed = 0.35  # 歩行速度
  phaseDiff = params[2]  # 位相差
  base_angle = 1.57  # π/2 = 90度（基本角度）
  
  # 各モーターの角度を計算（sin波ベース）
  # モーター0: front_left L
  a0 = math.sin(i * speed) * params[0] + base_angle
  # モーター1: front_left R（位相差を加算）
  a1 = math.sin(i * speed + phaseDiff) * params[1] + base_angle
  # モーター2: back_left L（位相オフセット1を加算）
  a2 = math.sin(i * speed + params[3]) * params[0] + base_angle
  # モーター3: back_left R（位相オフセット1 + 位相差）
  a3 = math.sin(i * speed + params[3] + phaseDiff) * params[1] + base_angle
  # モーター4: front_right L（位相オフセット2 + 位相差）
  a4 = math.sin(i * speed + params[4] + phaseDiff) * params[1] + base_angle
  # モーター5: front_right R（位相オフセット2）
  a5 = math.sin(i * speed + params[4]) * params[0] + base_angle
  # モーター6: back_right L（位相オフセット3 + 位相差）
  a6 = math.sin(i * speed + params[5] + phaseDiff) * params[1] + base_angle
  # モーター7: back_right R（位相オフセット3）
  a7 = math.sin(i * speed + params[5]) * params[0] + base_angle
  
  joint_values = [a0, a1, a2, a3, a4, a5, a6, a7]
  return joint_values


def evaluate_desired_motorAngle_hop(i, params):
  """
  ホップ（跳躍）パターンを生成
  
  前後の脚を交互に動かして跳躍するパターンです。
  パラメータ: 2個
    - params[0]: 振幅
    - params[1]: 速度
  
  Args:
    i: ステップ数
    params: パラメータリスト（2要素）
  
  Returns:
    8つのモーターの目標角度リスト（ラジアン）
  """
  amplitude = params[0]  # 振幅
  speed = params[1]      # 速度
  base_angle = 1.57      # π/2 = 90度
  
  # 前後の脚を180度（π）ずらして交互に動かす
  a1 = math.sin(i * speed) * amplitude + base_angle
  a2 = math.sin(i * speed + 3.14) * amplitude + base_angle  # 3.14 = π
  
  # モーター順序: [front_left_L, front_left_R, back_left_L, back_left_R,
  #                front_right_L, front_right_R, back_right_L, back_right_R]
  # 前脚と後脚を交互に動かすパターン
  joint_values = [a1, base_angle, a2, base_angle, base_angle, a1, base_angle, a2]
  return joint_values


# 評価関数をマッピングに登録（関数名文字列 -> 関数オブジェクト）
evaluate_func_map[
    'evaluate_desired_motorAngle_8Amplitude8Phase'] = evaluate_desired_motorAngle_8Amplitude8Phase
evaluate_func_map[
    'evaluate_desired_motorAngle_2Amplitude4Phase'] = evaluate_desired_motorAngle_2Amplitude4Phase
evaluate_func_map['evaluate_desired_motorAngle_hop'] = evaluate_desired_motorAngle_hop


def evaluate_params(evaluateFunc,
                    params,
                    objectiveParams,
                    urdfRoot='',
                    timeStep=0.01,
                    maxNumSteps=10000,
                    sleepTime=0):
  """
  歩行パターンのパラメータを評価
  
  指定された歩行パターン関数とパラメータでシミュレーションを実行し、
  移動距離とエネルギー消費を考慮した評価値を返します。
  
  評価値 = 移動距離 - α * 総エネルギー消費
  （移動距離が大きく、エネルギー消費が少ないほど良い）
  
  Args:
    evaluateFunc: 評価関数名（文字列）
    params: 歩行パターンのパラメータリスト
    objectiveParams: 評価関数のパラメータ [α]（エネルギー消費の重み）
    urdfRoot: URDFファイルのルートパス
    timeStep: シミュレーションのタイムステップ（秒）
    maxNumSteps: 最大ステップ数
    sleepTime: 各ステップ後の待機時間（可視化用、秒）
  
  Returns:
    評価値（final_distance - alpha * total_energy）
  """
  print('start evaluation')
  beforeTime = time.time()
  
  # シミュレーションをリセット
  p.resetSimulation()
  
  # タイムステップを設定
  p.setTimeStep(timeStep)
  # 地面をロード
  p.loadURDF("%s/plane.urdf" % urdfRoot)
  # 重力を設定（Z軸負方向に-10 m/s²）
  p.setGravity(0, 0, -10)

  # Minitaurロボットを初期化
  global minitaur
  minitaur = Minitaur(urdfRoot)
  start_position = current_position()  # 開始位置を記録
  last_position = None  # トレース用（未使用）
  total_energy = 0  # 総エネルギー消費を初期化

  # シミュレーションループ
  for i in range(maxNumSteps):
    # 現在のトルクと速度を取得
    torques = minitaur.getMotorTorques()
    velocities = minitaur.getMotorVelocities()
    # エネルギー消費を計算: |トルク| * |速度| * タイムステップ
    # これは各モーターの消費電力の合計に相当
    total_energy += np.dot(np.fabs(torques), np.fabs(velocities)) * timeStep

    # 歩行パターン関数を呼び出してモーター角度を計算
    joint_values = evaluate_func_map[evaluateFunc](i, params)
    # モーターコマンドを適用
    minitaur.applyAction(joint_values)
    # シミュレーションを1ステップ進める
    p.stepSimulation()
    
    # 転倒チェック（転倒したら終了）
    if (is_fallen()):
      break

    # 100ステップごとに進捗を表示
    if i % 100 == 0:
      sys.stdout.write('.')
      sys.stdout.flush()
    # 可視化のための待機
    time.sleep(sleepTime)

  print(' ')

  # 評価値を計算
  alpha = objectiveParams[0]  # エネルギー消費の重み
  final_distance = np.linalg.norm(start_position - current_position())  # 移動距離
  # 評価値 = 移動距離 - α * 総エネルギー消費
  # （移動距離が大きく、エネルギー消費が少ないほど良い）
  finalReturn = final_distance - alpha * total_energy
  elapsedTime = time.time() - beforeTime
  
  # 結果を表示
  print("trial for ", params, " final_distance", final_distance, "total_energy", total_energy,
        "finalReturn", finalReturn, "elapsed_time", elapsedTime)
  return finalReturn
