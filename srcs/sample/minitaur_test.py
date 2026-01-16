"""
Minitaurロボットのテストスクリプト

PyBullet公式サンプル: 最適化されたパラメータで歩行パターンを評価
- 2Amplitude4Phase歩行パターンを使用
- 最適化されたパラメータで実行
- 評価値を表示
"""
import sys
# 一部のPythonインタープリタでは '.' を追加する必要がある
sys.path.append(".")

import pybullet as p
from minitaur import Minitaur
from minitaur_evaluate import *

import time
import math
import numpy as np
import pybullet_data


def main(unused_args):
  """
  メイン関数
  
  最適化されたパラメータで2Amplitude4Phase歩行パターンを評価します。
  """
  # シミュレーションのタイムステップ（秒）
  timeStep = 0.01
  
  # PyBulletに接続（SHARED_MEMORYを試し、失敗したらGUIモード）
  # SHARED_MEMORY: 既存のPyBulletプロセスと共有（高速）
  # GUI: 新しいGUIウィンドウを開く
  c = p.connect(p.SHARED_MEMORY)
  if (c < 0):
    c = p.connect(p.GUI)
  
  # pybullet_dataのパスを追加（URDFファイルを探すため）
  p.setAdditionalSearchPath(pybullet_data.getDataPath())
  
  # 最適化されたパラメータ（2Amplitude4Phase用）
  # これらのパラメータは進化計算などで最適化された値です
  params = [
      0.1903581461951056,   # 振幅1
      0.0006732219568880068,  # 振幅2
      0.05018085615283363,   # 位相差
      3.219916795483583,     # 位相オフセット1
      6.2406418167980595,    # 位相オフセット2
      4.189869754607539      # 位相オフセット3
  ]
  
  # 使用する評価関数名
  evaluate_func = 'evaluate_desired_motorAngle_2Amplitude4Phase'
  
  # エネルギー消費の重み（評価値計算で使用）
  # 小さい値 = 移動距離を重視、大きい値 = エネルギー効率を重視
  energy_weight = 0.01

  # パラメータを評価
  finalReturn = evaluate_params(evaluateFunc=evaluate_func,
                                params=params,
                                objectiveParams=[energy_weight],
                                timeStep=timeStep,
                                sleepTime=timeStep)  # sleepTime=timeStepで可視化

  # 評価値を表示
  print(finalReturn)


# スクリプトが直接実行された場合にmain()を呼び出す
main(0)
