# PyBullet URDF/SDF/XML ファイル一覧

このドキュメントは、PyBulletのGitHubリポジトリ（[bulletphysics/bullet3/data](https://github.com/bulletphysics/bullet3/tree/master/data)）から取得したURDF、SDF、XMLファイルの一覧です。

## 統計

### GitHubリポジトリ（bulletphysics/bullet3/data）
- **URDFファイル**: 95個
- **SDFファイル**: 17個
- **XMLファイル**: 30個（MJCF形式含む）
- **合計**: 142個

### 実際にインストールされているファイル（pybullet_dataパッケージ）
- **URDFファイル**: 約1,095個（GitHubリポジトリより多い）
- 注: pybullet_dataパッケージには、GitHubリポジトリに加えて追加のファイルが含まれています

## カテゴリ別一覧

### 基本オブジェクト

#### 平面・床
- `plane.urdf` - 基本的な平面
- `plane100.urdf` - 大きな平面（100x100）
- `planeMesh.urdf` - メッシュ形式の平面
- `plane_implicit.urdf` - 暗黙的な平面
- `plane_transparent.urdf` - 透明な平面
- `plane_with_collision_audio.urdf` - 衝突音付き平面
- `plane_with_restitution.urdf` - 反発係数付き平面

#### 基本形状
- `block.urdf` - ブロック
- `capsule.urdf` - カプセル
- `cube.urdf` - 立方体
- `cube_collisionfilter.urdf` - 衝突フィルター付き立方体
- `cube_concave.urdf` - 凹面立方体
- `cube_convex.urdf` - 凸面立方体
- `cube_no_friction.urdf` - 摩擦なし立方体
- `cube_small.urdf` - 小さな立方体
- `cube_soft.urdf` - 柔らかい立方体
- `sphere2.urdf` - 球体
- `sphere2_rolling_friction.urdf` - 転がり摩擦付き球体
- `sphere2red.urdf` - 赤い球体
- `sphere_1cm.urdf` - 1cmの球体
- `sphere_small.urdf` - 小さな球体
- `sphere_small_zeroinertia.urdf` - 慣性なし小さな球体
- `sphere_transparent.urdf` - 透明な球体
- `sphere_with_restitution.urdf` - 反発係数付き球体
- `marble_cube.urdf` - 大理石の立方体

### ロボット

#### ヒューマノイド
- `humanoid.urdf` - ヒューマノイドロボット
- `humanoid/nao.urdf` - NAOロボット

#### アームロボット
- `kuka_iiwa/model.urdf` - KUKA LBR iiwa ロボットアーム
- `kuka_iiwa/model_free_base.urdf` - 自由ベースのKUKA
- `kuka_iiwa/model_vr_limits.urdf` - VR制限付きKUKA
- `kuka_iiwa/model_for_sdf.urdf` - SDF用KUKA
- `kuka_iiwa/kuka_with_gripper.sdf` - グリッパー付きKUKA (SDF)
- `kuka_iiwa/kuka_with_gripper2.sdf` - グリッパー付きKUKA v2 (SDF)
- `kuka_iiwa/kuka_world.sdf` - KUKAワールド (SDF)
- `kuka_iiwa/model.sdf` - KUKAモデル (SDF)
- `kuka_iiwa/model2.sdf` - KUKAモデル v2 (SDF)
- `kuka_lwr/kuka.urdf` - KUKA LWR ロボットアーム
- `franka_panda/panda.urdf` - Franka Panda ロボットアーム（pybullet_dataに含まれる）
- `widowx/widowx.urdf` - WidowX ロボットアーム

#### 四足ロボット
- `quadruped/minitaur.urdf` - Minitaur 四足ロボット
- `quadruped/minitaur_derpy.urdf` - Minitaur Derpy版
- `quadruped/minitaur_fixed_all.urdf` - 全関節固定Minitaur
- `quadruped/minitaur_fixed_knees.urdf` - 膝固定Minitaur
- `quadruped/minitaur_rainbow_dash.urdf` - Rainbow Dash版Minitaur
- `quadruped/minitaur_rainbow_dash_v1.urdf` - Rainbow Dash v1
- `quadruped/minitaur_single_motor.urdf` - 単一モーターMinitaur
- `quadruped/minitaur_v1.urdf` - Minitaur v1
- `quadruped/quadruped.urdf` - 汎用四足ロボット
- `quadruped/vision/vision60.urdf` - Vision60 四足ロボット

#### その他のロボット
- `r2d2.urdf` - R2-D2ロボット
- `cartpole.urdf` - カートポール
- `Pendulum_Tendon_1_Cart_Rail.urdf` - 振り子テンドンカートレール
- `TwoJointRobot_w_fixedJoints.urdf` - 2関節ロボット（固定関節付き）
- `TwoJointRobot_wo_fixedJoints.urdf` - 2関節ロボット（固定関節なし）

### 車両

- `racecar/racecar.urdf` - レースカー
- `racecar/racecar_differential.urdf` - 差動駆動レースカー
- `husky/husky.urdf` - Husky ロボット
- `Quadrotor/quadrotor.urdf` - クアッドロータードローン

### グリッパー

- `gripper/wsg50_one_motor_gripper.sdf` - WSG50 グリッパー (SDF)
- `gripper/wsg50_one_motor_gripper_free_base.sdf` - 自由ベースWSG50 (SDF)
- `gripper/wsg50_one_motor_gripper_left_finger.urdf` - WSG50左指 (URDF)
- `gripper/wsg50_one_motor_gripper_new.sdf` - 新WSG50 (SDF)
- `gripper/wsg50_one_motor_gripper_new_free_base.sdf` - 新自由ベースWSG50 (SDF)
- `gripper/wsg50_one_motor_gripper_no_finger.sdf` - 指なしWSG50 (SDF)
- `gripper/wsg50_one_motor_gripper_right_finger.urdf` - WSG50右指 (URDF)
- `gripper/wsg50_with_r2d2_gripper.sdf` - R2D2グリッパー付きWSG50 (SDF)
- `pr2_gripper.urdf` - PR2グリッパー
- `cube_gripper_left.urdf` - 左グリッパー（立方体）
- `cube_gripper_right.urdf` - 右グリッパー（立方体）

### 家具・環境

#### テーブル
- `table/table.urdf` - テーブル
- `table/table2.urdf` - テーブル v2
- `table_square/table_square.urdf` - 四角いテーブル

#### トレイ・食器
- `tray/tray.urdf` - トレイ
- `tray/tray_textured2.urdf` - テクスチャ付きトレイ v2
- `tray/traybox.urdf` - トレイボックス
- `dinnerware/cup/cup_small.urdf` - 小さなコップ
- `dinnerware/pan_tefal.urdf` - テファルパン
- `dinnerware/plate.urdf` - プレート

#### その他
- `door.urdf` - ドア
- `jenga/jenga.urdf` - ジェンガ
- `lego/lego.urdf` - レゴ
- `kiva_shelf/model.sdf` - Kiva棚 (SDF)
- `kitchens/1.sdf` - キッチン (SDF)
- `stadium.sdf` - スタジアム (SDF)

### 関節・アクチュエータ

- `hinge.urdf` - ヒンジ関節
- `prismatic.urdf` - プリズマティック関節
- `pantilt.urdf` - パン・チルト機構
- `pole.urdf` - ポール
- `wheel.urdf` - ホイール
- `test_joints_MB.urdf` - 関節テスト

### 変形可能オブジェクト

- `cloth_z_up.urdf` - 布（Z軸上向き）
- `reduced_beam/reduced_beam.urdf` - 縮約ビーム
- `reduced_cube/deform_cube.urdf` - 変形可能立方体
- `reduced_cube/reduced_cube.urdf` - 縮約立方体
- `reduced_torus/reduced_torus.urdf` - 縮約トーラス
- `torus_deform.urdf` - 変形可能トーラス

### トーラス

- `torus/torus.urdf` - トーラス
- `torus/torus_with_plane.urdf` - 平面付きトーラス
- `torus/torus_with_separate_plane.urdf` - 分離平面付きトーラス

### その他のオブジェクト

- `Base_1.urdf` - ベース1
- `Base_2.urdf` - ベース2
- `duck_vhacd.urdf` - アヒル（VHACD）
- `teddy_large.urdf` - 大きなテディベア
- `teddy_vhacd.urdf` - テディベア（VHACD）
- `samurai.urdf` - サムライ
- `terrain.urdf` - 地形
- `user_data_test_object.urdf` - ユーザーデータテストオブジェクト

### 複数オブジェクト

- `threecubes/threecubes.urdf` - 3つの立方体
- `threecubes/newsdf.sdf` - 新SDF (SDF)
- `two_cubes.sdf` - 2つの立方体 (SDF)

### 差動駆動

- `differential/diff_ring.urdf` - 差動リング

### MJCF形式（MuJoCo XML）

#### 環境
- `mjcf/ant.xml` - アリ
- `mjcf/half_cheetah.xml` - ハーフチーター
- `mjcf/hopper.xml` - ホッパー
- `mjcf/humanoid.xml` - ヒューマノイド
- `mjcf/humanoid_fixed.xml` - 固定ヒューマノイド
- `mjcf/humanoid_symmetric.xml` - 対称ヒューマノイド
- `mjcf/humanoid_symmetric_no_ground.xml` - 地面なし対称ヒューマノイド
- `mjcf/inverted_double_pendulum.xml` - 逆二重振り子
- `mjcf/inverted_pendulum.xml` - 逆振り子
- `mjcf/pusher.xml` - プッシャー
- `mjcf/reacher.xml` - リーチャー
- `mjcf/striker.xml` - ストライカー
- `mjcf/swimmer.xml` - スイマー
- `mjcf/thrower.xml` - スローワー
- `mjcf/walker2d.xml` - 2Dウォーカー

#### 基本形状
- `mjcf/capsule.xml` - カプセル
- `mjcf/capsule_fromtoX.xml` - カプセル（X軸）
- `mjcf/capsule_fromtoY.xml` - カプセル（Y軸）
- `mjcf/capsule_fromtoZ.xml` - カプセル（Z軸）
- `mjcf/cylinder.xml` - 円柱
- `mjcf/cylinder_fromtoX.xml` - 円柱（X軸）
- `mjcf/cylinder_fromtoY.xml` - 円柱（Y軸）
- `mjcf/cylinder_fromtoZ.xml` - 円柱（Z軸）
- `mjcf/sphere.xml` - 球体

#### 地面
- `mjcf/ground.xml` - 地面
- `mjcf/ground_plane.xml` - 地面平面
- `mjcf/hello_mjcf.xml` - Hello MJCF

### その他のXML

- `MPL/MPL.xml` - MPL
- `MPL/mpl2.xml` - MPL2
- `kuka_lwr/materials.xml` - KUKA LWR マテリアル

## 使用方法

PyBulletでこれらのファイルを使用するには：

```python
import pybullet as p
import pybullet_data

# PyBulletデータパスを追加
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# URDFファイルをロード
robot_id = p.loadURDF("franka_panda/panda.urdf")
object_id = p.loadURDF("tray/tray.urdf", basePosition=[0.5, 0.0, 0.01])
```

## 注意事項

- 一部のファイルは`pybullet_data`パッケージに含まれていない場合があります
- SDFファイルは`loadSDF()`関数を使用します
- MJCFファイルは`loadMJCF()`関数を使用します
- 実際に使用可能なファイルは、インストールされている`pybullet_data`パッケージの内容を確認してください

## 参考リンク

- [PyBullet GitHub Repository](https://github.com/bulletphysics/bullet3)
- [PyBullet Data Directory](https://github.com/bulletphysics/bullet3/tree/master/data)
- [PyBullet Documentation](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit)

## 更新履歴

- 2026-01-13: 初版作成（GitHub APIから取得）
