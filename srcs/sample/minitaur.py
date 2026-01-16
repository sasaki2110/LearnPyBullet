"""
Minitaurロボットの制御クラス

PyBullet公式サンプル: Minitaur四足ロボットの制御を実装
- 8つのモーター（各脚に2つずつ、L/R）
- PD制御による位置制御
- モーター方向の考慮（左側-1、右側+1）
"""
import pybullet as p
import numpy as np


class Minitaur:
  """
  Minitaur四足ロボットの制御クラス
  
  各脚に2つのモーター（L/R）があり、合計8つのモーターで制御します。
  モーター方向を考慮して、左右で異なる符号を適用します。
  """

  def __init__(self, urdfRootPath=''):
    """
    初期化
    
    Args:
      urdfRootPath: URDFファイルのルートパス（空文字列の場合はpybullet_dataを使用）
    """
    self.urdfRootPath = urdfRootPath
    self.reset()

  def buildJointNameToIdDict(self):
    """
    ジョイント名からIDへのマッピングを構築
    
    すべてのジョイントを走査し、ジョイント名をキー、IDを値とする辞書を作成します。
    その後、初期姿勢を設定し、物理シミュレーションを100ステップ進めて安定化させます。
    """
    nJoints = p.getNumJoints(self.quadruped)
    self.jointNameToId = {}
    for i in range(nJoints):
      jointInfo = p.getJointInfo(self.quadruped, i)
      # ジョイント名をUTF-8でデコードして辞書に追加
      self.jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
    # 初期姿勢を設定
    self.resetPose()
    # 物理シミュレーションを100ステップ進めて安定化
    for i in range(100):
      p.stepSimulation()

  def buildMotorIdList(self):
    """
    モーターIDリストを構築
    
    8つのモーター（各脚に2つずつ）のIDを順番にリストに追加します。
    順序: front_left(L,R), back_left(L,R), front_right(L,R), back_right(L,R)
    """
    self.motorIdList.append(self.jointNameToId['motor_front_leftL_joint'])
    self.motorIdList.append(self.jointNameToId['motor_front_leftR_joint'])
    self.motorIdList.append(self.jointNameToId['motor_back_leftL_joint'])
    self.motorIdList.append(self.jointNameToId['motor_back_leftR_joint'])
    self.motorIdList.append(self.jointNameToId['motor_front_rightL_joint'])
    self.motorIdList.append(self.jointNameToId['motor_front_rightR_joint'])
    self.motorIdList.append(self.jointNameToId['motor_back_rightL_joint'])
    self.motorIdList.append(self.jointNameToId['motor_back_rightR_joint'])

  def reset(self):
    """
    ロボットをリセット
    
    URDFファイルをロードし、PD制御パラメータを設定します。
    モーター方向: 左側4つは-1、右側4つは+1（左右で回転方向が逆のため）
    """
    # URDFファイルをロード（高さ0.2mの位置に配置）
    self.quadruped = p.loadURDF("%s/quadruped/minitaur.urdf" % self.urdfRootPath, 0, 0, .2)
    
    # PD制御パラメータ
    self.kp = 1          # 位置ゲイン（Pゲイン）
    self.kd = 0.1        # 速度ゲイン（Dゲイン）
    self.maxForce = 3.5  # 最大トルク（N・m）
    
    # モーター設定
    self.nMotors = 8
    self.motorIdList = []
    # モーター方向: 左側4つは-1、右側4つは+1
    # これは左右でモーターの回転方向が逆のため
    self.motorDir = [-1, -1, -1, -1, 1, 1, 1, 1]
    
    # ジョイント名とIDのマッピングを構築
    self.buildJointNameToIdDict()
    # モーターIDリストを構築
    self.buildMotorIdList()

  def setMotorAngleById(self, motorId, desiredAngle):
    """
    モーターIDを指定して角度を設定（PD制御）
    
    Args:
      motorId: モーターのジョイントID
      desiredAngle: 目標角度（ラジアン）
    """
    p.setJointMotorControl2(bodyIndex=self.quadruped,
                            jointIndex=motorId,
                            controlMode=p.POSITION_CONTROL,  # 位置制御モード
                            targetPosition=desiredAngle,      # 目標角度
                            positionGain=self.kp,             # Pゲイン
                            velocityGain=self.kd,             # Dゲイン
                            force=self.maxForce)              # 最大トルク

  def setMotorAngleByName(self, motorName, desiredAngle):
    """
    モーター名を指定して角度を設定
    
    Args:
      motorName: モーター名（例: 'motor_front_leftL_joint'）
      desiredAngle: 目標角度（ラジアン）
    """
    self.setMotorAngleById(self.jointNameToId[motorName], desiredAngle)

  def resetPose(self):
    """
    初期姿勢を設定（立ち姿勢）
    
    各脚のモーターを90度（π/2）に設定し、膝の角度も計算された値に設定します。
    また、各脚の左右の膝リンク間に制約（constraint）を作成して、膝の動きを同期させます。
    膝リンクは速度0で固定し、モーターによって制御されるようにします。
    """
    kneeFrictionForce = 0  # 膝の摩擦（0 = 摩擦なし）
    halfpi = 1.57079632679  # π/2 = 90度（基本角度）
    # 膝の角度（計算された値: π/2 - arccos(上脚の長さ / 下脚の長さ)）
    kneeangle = -2.1834  #halfpi - acos(upper_leg_length / lower_leg_length)

    # 左前脚（front_left）の設定
    p.resetJointState(self.quadruped, self.jointNameToId['motor_front_leftL_joint'],
                      self.motorDir[0] * halfpi)
    p.resetJointState(self.quadruped, self.jointNameToId['knee_front_leftL_link'],
                      self.motorDir[0] * kneeangle)
    p.resetJointState(self.quadruped, self.jointNameToId['motor_front_leftR_joint'],
                      self.motorDir[1] * halfpi)
    p.resetJointState(self.quadruped, self.jointNameToId['knee_front_leftR_link'],
                      self.motorDir[1] * kneeangle)
    p.createConstraint(self.quadruped, self.jointNameToId['knee_front_leftR_link'], self.quadruped,
                       self.jointNameToId['knee_front_leftL_link'], p.JOINT_POINT2POINT, [0, 0, 0],
                       [0, 0.005, 0.2], [0, 0.01, 0.2])
    self.setMotorAngleByName('motor_front_leftL_joint', self.motorDir[0] * halfpi)
    self.setMotorAngleByName('motor_front_leftR_joint', self.motorDir[1] * halfpi)
    p.setJointMotorControl2(bodyIndex=self.quadruped,
                            jointIndex=self.jointNameToId['knee_front_leftL_link'],
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=0,
                            force=kneeFrictionForce)
    p.setJointMotorControl2(bodyIndex=self.quadruped,
                            jointIndex=self.jointNameToId['knee_front_leftR_link'],
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=0,
                            force=kneeFrictionForce)

    # 左後脚（back_left）の設定（同様の処理）
    p.resetJointState(self.quadruped, self.jointNameToId['motor_back_leftL_joint'],
                      self.motorDir[2] * halfpi)
    p.resetJointState(self.quadruped, self.jointNameToId['knee_back_leftL_link'],
                      self.motorDir[2] * kneeangle)
    p.resetJointState(self.quadruped, self.jointNameToId['motor_back_leftR_joint'],
                      self.motorDir[3] * halfpi)
    p.resetJointState(self.quadruped, self.jointNameToId['knee_back_leftR_link'],
                      self.motorDir[3] * kneeangle)
    p.createConstraint(self.quadruped, self.jointNameToId['knee_back_leftR_link'], self.quadruped,
                       self.jointNameToId['knee_back_leftL_link'], p.JOINT_POINT2POINT, [0, 0, 0],
                       [0, 0.005, 0.2], [0, 0.01, 0.2])
    self.setMotorAngleByName('motor_back_leftL_joint', self.motorDir[2] * halfpi)
    self.setMotorAngleByName('motor_back_leftR_joint', self.motorDir[3] * halfpi)
    p.setJointMotorControl2(bodyIndex=self.quadruped,
                            jointIndex=self.jointNameToId['knee_back_leftL_link'],
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=0,
                            force=kneeFrictionForce)
    p.setJointMotorControl2(bodyIndex=self.quadruped,
                            jointIndex=self.jointNameToId['knee_back_leftR_link'],
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=0,
                            force=kneeFrictionForce)

    # 右前脚（front_right）の設定（同様の処理）
    p.resetJointState(self.quadruped, self.jointNameToId['motor_front_rightL_joint'],
                      self.motorDir[4] * halfpi)
    p.resetJointState(self.quadruped, self.jointNameToId['knee_front_rightL_link'],
                      self.motorDir[4] * kneeangle)
    p.resetJointState(self.quadruped, self.jointNameToId['motor_front_rightR_joint'],
                      self.motorDir[5] * halfpi)
    p.resetJointState(self.quadruped, self.jointNameToId['knee_front_rightR_link'],
                      self.motorDir[5] * kneeangle)
    p.createConstraint(self.quadruped, self.jointNameToId['knee_front_rightR_link'],
                       self.quadruped, self.jointNameToId['knee_front_rightL_link'],
                       p.JOINT_POINT2POINT, [0, 0, 0], [0, 0.005, 0.2], [0, 0.01, 0.2])
    self.setMotorAngleByName('motor_front_rightL_joint', self.motorDir[4] * halfpi)
    self.setMotorAngleByName('motor_front_rightR_joint', self.motorDir[5] * halfpi)
    p.setJointMotorControl2(bodyIndex=self.quadruped,
                            jointIndex=self.jointNameToId['knee_front_rightL_link'],
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=0,
                            force=kneeFrictionForce)
    p.setJointMotorControl2(bodyIndex=self.quadruped,
                            jointIndex=self.jointNameToId['knee_front_rightR_link'],
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=0,
                            force=kneeFrictionForce)

    # 右後脚（back_right）の設定（同様の処理）
    p.resetJointState(self.quadruped, self.jointNameToId['motor_back_rightL_joint'],
                      self.motorDir[6] * halfpi)
    p.resetJointState(self.quadruped, self.jointNameToId['knee_back_rightL_link'],
                      self.motorDir[6] * kneeangle)
    p.resetJointState(self.quadruped, self.jointNameToId['motor_back_rightR_joint'],
                      self.motorDir[7] * halfpi)
    p.resetJointState(self.quadruped, self.jointNameToId['knee_back_rightR_link'],
                      self.motorDir[7] * kneeangle)
    p.createConstraint(self.quadruped, self.jointNameToId['knee_back_rightR_link'], self.quadruped,
                       self.jointNameToId['knee_back_rightL_link'], p.JOINT_POINT2POINT, [0, 0, 0],
                       [0, 0.005, 0.2], [0, 0.01, 0.2])
    self.setMotorAngleByName('motor_back_rightL_joint', self.motorDir[6] * halfpi)
    self.setMotorAngleByName('motor_back_rightR_joint', self.motorDir[7] * halfpi)
    p.setJointMotorControl2(bodyIndex=self.quadruped,
                            jointIndex=self.jointNameToId['knee_back_rightL_link'],
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=0,
                            force=kneeFrictionForce)
    p.setJointMotorControl2(bodyIndex=self.quadruped,
                            jointIndex=self.jointNameToId['knee_back_rightR_link'],
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=0,
                            force=kneeFrictionForce)

  def getBasePosition(self):
    """
    ベース（胴体）の位置を取得
    
    Returns:
      位置タプル (x, y, z)
    """
    position, orientation = p.getBasePositionAndOrientation(self.quadruped)
    return position

  def getBaseOrientation(self):
    """
    ベース（胴体）の姿勢を取得
    
    Returns:
      クォータニオンタプル (x, y, z, w)
    """
    position, orientation = p.getBasePositionAndOrientation(self.quadruped)
    return orientation

  def applyAction(self, motorCommands):
    """
    モーターコマンドを適用
    
    モーター方向を考慮してコマンドを変換し、各モーターに設定します。
    
    Args:
      motorCommands: 8つのモーターの目標角度リスト（ラジアン）
    """
    # モーター方向を考慮してコマンドを変換
    # 左側は-1倍、右側は+1倍される
    motorCommandsWithDir = np.multiply(motorCommands, self.motorDir)
    for i in range(self.nMotors):
      self.setMotorAngleById(self.motorIdList[i], motorCommandsWithDir[i])

  def getMotorAngles(self):
    """
    現在のモーター角度を取得
    
    Returns:
      8つのモーターの角度配列（ラジアン、モーター方向を考慮）
    """
    motorAngles = []
    for i in range(self.nMotors):
      jointState = p.getJointState(self.quadruped, self.motorIdList[i])
      motorAngles.append(jointState[0])  # ジョイント状態の0番目が角度
    # モーター方向を考慮して変換（左側は-1倍、右側は+1倍）
    motorAngles = np.multiply(motorAngles, self.motorDir)
    return motorAngles

  def getMotorVelocities(self):
    """
    現在のモーター速度を取得
    
    Returns:
      8つのモーターの速度配列（rad/s、モーター方向を考慮）
    """
    motorVelocities = []
    for i in range(self.nMotors):
      jointState = p.getJointState(self.quadruped, self.motorIdList[i])
      motorVelocities.append(jointState[1])  # ジョイント状態の1番目が速度
    # モーター方向を考慮して変換
    motorVelocities = np.multiply(motorVelocities, self.motorDir)
    return motorVelocities

  def getMotorTorques(self):
    """
    現在のモータートルクを取得
    
    Returns:
      8つのモーターのトルク配列（N・m、モーター方向を考慮）
    """
    motorTorques = []
    for i in range(self.nMotors):
      jointState = p.getJointState(self.quadruped, self.motorIdList[i])
      motorTorques.append(jointState[3])  # ジョイント状態の3番目がトルク
    # モーター方向を考慮して変換
    motorTorques = np.multiply(motorTorques, self.motorDir)
    return motorTorques
