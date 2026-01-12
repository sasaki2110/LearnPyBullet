# はじめに

PyBulletは、ロボティクスシミュレーションと機械学習のための高速で使いやすいPythonモジュールであり、シミュレーションから実機への転移（sim-to-real transfer）に焦点を当てています。PyBulletを使用すると、URDF、SDF、MJCF、その他のファイル形式から関節を持つ物体（articulated bodies）を読み込むことができます。PyBulletは順動力学シミュレーション、逆動力学計算、順運動学・逆運動学、衝突検出、レイ交差クエリを提供します。Bullet Physics SDKには、シミュレートされたMinitaur四足ロボット、TensorFlow推論を使用して動作するヒューマノイド、物体を把持するKUKAアームなどのPyBulletロボティクス例が含まれています。縮約座標マルチボディ、剛体、変形可能な物体は、統一されたLCP制約ソルバーによって処理され、これは論文のArticulated Islands Algorithmと同様です。線形時間順動力学とソルバーA行列の作成には、Articulated Body Algorithmが使用されます。

物理シミュレーションに加えて、レンダリングへのバインディングがあり、CPUレンダラー（TinyRenderer）とOpenGL 3.xレンダリング・可視化、HTC ViveやOculus Riftなどのバーチャルリアリティヘッドセットのサポートが含まれています。PyBulletには、衝突検出クエリ（最近接点、重複ペア、レイ交差テストなど）を実行し、デバッグレンダリング（デバッグラインとテキスト）を追加する機能もあります。PyBulletには、共有メモリ、UDP、TCPネットワーキング用のクロスプラットフォーム組み込みクライアント-サーバーサポートがあります。そのため、Linux上でPyBulletを実行し、Windows VRサーバーに接続することができます。

PyBulletは、新しいBullet C-APIをラップしており、これは基盤となる物理エンジンとレンダーエンジンから独立するように設計されているため、Bulletの新しいバージョンに簡単に移行したり、異なる物理エンジンやレンダーエンジンを使用したりできます。デフォルトでは、PyBulletはCPU上でBullet 2.x APIを使用します。OpenCLを使用してGPU上で実行されるBullet 3.xも公開する予定です。PyBulletと同様のC++ APIもあり、b3RobotSimulatorClientAPIを参照してください。

PyBulletは、TensorFlowやOpenAI Gymと簡単に使用できます。Google Brain [1,2,3,4]、X[1,2]、Stanford AI Lab [1,2,3]、OpenAI、INRIA [1]など、多くの研究室の研究者がPyBulletを使用しています。研究でPyBulletを使用する場合は、引用を追加してください。

PyBulletのインストールは、(sudo) pip install PyBullet（Python 2.x）、またはpip3 install PyBulletと同様に簡単です。これにより、PyBulletモジュールとpybullet_envs Gym環境が公開されます。
3

## Hello PyBullet World

以下は、ステップバイステップで説明するPyBulletの紹介スクリプトです：
import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("r2d2.urdf",startPos, startOrientation)
#set the center of mass frame (loadURDF sets base link frame)
startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos,
startOrientation)
for i in range (10000):
p.stepSimulation()
time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()

## connect, disconnect, bullet_client

PyBulletモジュールをインポートした後、最初に行うことは物理シミュレーションに「接続」することです。
PyBulletは、クライアントがコマンドを送信し、物理サーバーがステータスを返すクライアント-サーバー駆動のAPIを中心に設計されています。PyBulletには、いくつかの組み込み物理サーバーがあります：DIRECTとGUIです。GUIとDIRECTの両方の接続は、PyBulletと同じプロセス内で物理シミュレーションとレンダリングを実行します。

DIRECTモードでは、「Virtual Reality」と「Debug GUI, Lines, Text, Parameters」の章で説明されているように、OpenGLとVRハードウェア機能にアクセスできないことに注意してください。DIRECTモードでは、組み込みのソフトウェアレンダラーを使用して'getCameraImage' APIを通じて画像をレンダリングすることができます。これは、GPUのないサーバー上でクラウドでシミュレーションを実行する場合に役立ちます。

独自のデータファイルを提供することも、PyBulletに付属するPyBullet_dataパッケージを使用することもできます。このためには、pybullet_dataをインポートし、pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())を使用してディレクトリを登録します。
### getConnectionInfo

physicsClientIdを指定すると、リスト[isConnected, connectionMethod]を返します。

### isConnected

physicsClientIdを指定すると、接続されている場合はtrue、それ以外の場合はfalseを返します。

### setTimeOut

サーバーが特定のタイムアウト値内にコマンドを処理しない場合、クライアントは切断されます。setTimeOutを使用して、この値を秒単位で指定します。

### DIRECT、GUIを使用した接続

DIRECT接続は、トランスポート層を使用せず、グラフィックス可視化ウィンドウも使用せず、コマンドを物理エンジンに直接送信し、コマンドを実行した後にステータスを直接返します。

GUI接続は、PyBulletと同じプロセス空間内で、3D OpenGLレンダリングを使用した新しいグラフィカルユーザーインターフェース（GUI）を作成します。LinuxとWindowsでは、このGUIは別のスレッドで実行されますが、OSXではオペレーティングシステムの制限により同じスレッドで実行されます。Mac OSXでは、'stepSimulation'または他のPyBulletコマンドを実行するまで、OpenGLウィンドウにスピニングホイールが表示される場合があります。

コマンドとステータスメッセージは、通常のメモリバッファを使用してPyBulletクライアントとGUI物理シミュレーションサーバー間で送信されます。
同じマシン上の別のプロセス、またはリモートマシン上の物理サーバーに、SHARED_MEMORY、UDP、またはTCPネットワーキングを使用して接続することもできます。詳細については、Shared Memory、UDP、TCPに関するセクションを参照してください。

後方互換性のため、このメソッドは他のほとんどのメソッドとは異なり、キーワード引数を解析しません。

connectの入力引数は以下の通りです：
- **required connection mode** (integer): 
  - **DIRECT**: 新しい物理エンジンを作成し、直接通信します
  - **GUI**: グラフィカルGUIフロントエンドを持つ物理エンジンを作成し、それと通信します
  - **SHARED_MEMORY**: 同じマシン上の既存の物理エンジンプロセスに接続し、共有メモリを介して通信します
  - **UDP, TCP**: TCPまたはUDPネットワーキングを介して既存の物理サーバーに接続します
  - **GUI_SERVER**: GUIと同様ですが、外部SHARED_MEMORY接続を許可するサーバーとしても機能します
  - **SHARED_MEMORY_SERVER**: DIRECTと同様ですが、外部SHARED_MEMORY接続を許可するサーバーとしても機能します
  - **SHARED_MEMORY_GUI**: DIRECTと同様ですが、GUI表示のために外部グラフィックスサーバーに接続しようとします。Bullet ExampleBrowserには、Physics ServerまたはGraphics Serverとして機能するオプションがあります
- **optional key** (int): SHARED_MEMORYモードでは、オプションの共有メモリキー。ExampleBrowserまたはSharedMemoryPhysics_*を起動する際に、オプションのコマンドライン--shared_memory_keyを使用してキーを設定できます。これにより、同じマシン上で複数のサーバーを実行できます
- **optional hostName** (string): IPアドレスまたはホスト名、例えば"127.0.0.1"または"localhost"（UDPおよびTCP）、または"mymachine.domain.com"
- **optional port** (integer): UDPポート番号。デフォルトのUDPポートは1234、デフォルトのTCPポートは6667（サーバーのデフォルトと一致）
- **optional options** (string): GUIサーバーに渡されるコマンドラインオプション。背景色を設定できます。赤/緑/青のパラメータを範囲[0..1]で次のように設定します：
  ```
  p.connect(p.GUI, options="--background_color_red=1
  --background_color_blue=1 --background_color_green=1")
  ```
  その他のオプション：
  - `--mouse_move_multiplier=0.400000` (マウス感度)
  - `--mouse_wheel_multiplier=0.400000` (マウスホイール感度)
  - `--width=<int>` ウィンドウの幅（ピクセル単位）
  - `--height=<int>` ウィンドウの高さ（ピクセル単位）
  - `--mp4=moviename.mp4` (動画を記録、ffmpegが必要)
  - `--mp4fps=<int>` (動画記録用、フレームレートを設定)
  - `--nogui`
  - `--tracing`
  - `--png_skip_frames`
  - `--disable_retina`
  - `--render_device`
  - `--window_backend`

connectは、物理クライアントIDを返します。接続に失敗した場合は-1を返します。物理クライアントIDは、他のほとんどのPyBulletコマンドのオプション引数です。提供しない場合、物理クライアントID = 0と仮定されます。GUIを除いて、複数の異なる物理サーバーに接続できます。

例：
```python
pybullet.connect(pybullet.DIRECT)
pybullet.connect(pybullet.GUI, options="--opengl2")
pybullet.connect(pybullet.SHARED_MEMORY,1234)
pybullet.connect(pybullet.UDP,"192.168.0.1")
pybullet.connect(pybullet.UDP,"localhost", 1234)
pybullet.connect(pybullet.TCP,"localhost", 6667)
```

### 共有メモリを使用した接続

共有メモリ接続を許可する物理サーバーがいくつかあります：App_SharedMemoryPhysics、App_SharedMemoryPhysics_GUI、およびBullet Example Browserには、Experimental/Physics Serverの下に共有メモリ接続を許可する例があります。これにより、別のプロセスで物理シミュレーションとレンダリングを実行できます。

App_SharedMemoryPhysics_VR（ヘッドマウントディスプレイとHTC ViveやOculus Rift with Touchコントローラーなどの6自由度追跡コントローラーをサポートするバーチャルリアリティアプリケーション）に共有メモリを介して接続することもできます。Valve OpenVR SDKはWindowsでのみ適切に動作するため、App_SharedMemoryPhysics_VRはWindowsでのみビルドでき、premake（推奨）またはcmakeを使用します。

### UDPまたはTCPネットワーキングを使用した接続

UDPネットワーキングの場合、特定のUDPポートをリッスンするApp_PhysicsServerUDPがあります。信頼性の高いUDPネットワーキングには、オープンソースのenetライブラリを使用します。これにより、別のマシンで物理シミュレーションとレンダリングを実行できます。TCPの場合、PyBulletはclsocketライブラリを使用します。これは、ファイアウォールの背後にあるマシンからSSHトンネリングを使用してロボットシミュレーションに接続する場合に役立ちます。たとえば、LinuxでPyBulletを使用して制御スタックまたは機械学習を実行しながら、WindowsでHTC ViveまたはRiftを使用してバーチャルリアリティで物理サーバーを実行できます。

もう1つのUDPアプリケーションは、App_PhysicsServerSharedMemoryBridgeUDPアプリケーションで、既存の物理サーバーへのブリッジとして機能します：このブリッジにUDP経由で接続でき、ブリッジは共有メモリを使用して物理サーバーに接続します：ブリッジはクライアントとサーバー間でメッセージを渡します。同様に、TCPバージョンもあります（UDPをTCPに置き換えます）。

GRPCクライアントとサーバーのサポートもありますが、デフォルトでは有効になっていません。--enable_grpcオプションを使用してpremake4ビルドシステムで試すことができます（Bullet/build3/premake4を参照）。

注意：現時点では、クライアントとサーバーの両方が32ビットまたは64ビットビルドである必要があります！

### bullet_client

複数の独立したシミュレーションを並列で使用したい場合は、pybullet_utils.bullet_clientを使用できます。bullet_client.BulletClient(connection_mode=pybullet.GUI, options='')のインスタンスは、pybulletインスタンスと同じAPIを持ちます。各API呼び出しに適切なphysicsClientIdを自動的に追加します。PyBullet Gym環境は、bullet_clientを使用して複数の環境を並列でトレーニングできるようにします。実装についてはenv_bases.pyを参照してください。別の小さな例では、それぞれ独自のオブジェクトを持つ2つの別々のインスタンスを持つ方法を示しています。multipleScenes.pyを参照してください。

### disconnect

connect呼び出しによって返された物理クライアントID（非負の場合）を使用して、物理サーバーから切断できます。'DIRECT'または'GUI'物理サーバーはシャットダウンします。別の（プロセス外の）物理サーバーは実行を続けます。すべてのアイテムを削除するには、'resetSimulation'も参照してください。

disconnectのパラメータ：
- **optional physicsClientId** (int): 複数の物理サーバーに接続している場合、どれを選択するか指定できます

## setGravity

デフォルトでは、重力は有効になっていません。setGravityを使用すると、すべてのオブジェクトのデフォルトの重力を設定できます。

setGravityの入力パラメータは以下の通りです（戻り値なし）：

- **required graX** (float): X世界軸に沿った重力
- **required gravY** (float): Y世界軸に沿った重力
- **required gravZ** (float): Z世界軸に沿った重力
- **optional physicsClientId** (int): 複数の物理サーバーに接続している場合、どれを選択するか指定できます。

## loadURDF, loadSDF, loadMJCF

loadURDFは、物理サーバーにコマンドを送信して、Universal Robot Description File (URDF)から物理モデルを読み込みます。URDFファイルは、ROSプロジェクト（Robot Operating System）によってロボットやその他のオブジェクトを記述するために使用され、WillowGarageとOpen Source Robotics Foundation (OSRF)によって作成されました。多くのロボットには公開されているURDFファイルがあり、説明とチュートリアルはこちらで見つけることができます：http://wiki.ros.org/urdf/Tutorials

重要な注意：ほとんどの関節（スライダー、回転、連続）は、デフォルトでモーターが有効になっており、自由な動きを妨げます。これは、非常に高い摩擦を持つハーモニックドライブを備えたロボット関節と同様です。pybullet.setJointMotorControl2を使用して、関節モーター制御モードとターゲット設定を設定する必要があります。詳細については、setJointMotorControl2 APIを参照してください。

警告：デフォルトでは、PyBulletは読み込みを高速化するために一部のファイルをキャッシュします。setPhysicsEngineParameter(enableFileCaching=0)を使用してファイルキャッシュを無効にできます。
loadURDFの引数は以下の通りです：

- **required fileName** (string): 物理サーバーのファイルシステム上のURDFファイルへの相対パスまたは絶対パス
- **optional basePosition** (vec3): 世界空間座標[X,Y,Z]で指定された位置にオブジェクトのベースを作成します。この位置はURDFリンクの位置であることに注意してください。慣性フレームがゼロでない場合、これは重心位置とは異なります。重心の位置/姿勢を設定するには、resetBasePositionAndOrientationを使用してください。
- **optional baseOrientation** (vec4): 世界空間クォータニオン[X,Y,Z,W]として指定された姿勢でオブジェクトのベースを作成します。basePositionの注意事項を参照してください。
- **optional useMaximalCoordinates** (int): デフォルトでは、URDFファイル内の関節は縮約座標法を使用して作成されます：関節はFeatherstone Articulated Body Algorithm（ABA、Bullet 2.xのbtMultiBody）を使用してシミュレートされます。useMaximalCoordinatesオプションは、各リンクに対して6自由度の剛体を作成し、それらの剛体間の制約を使用して関節をモデル化します。リンク/関節を持たないボディに対してuseMaximalCoordinatesを有効にすると、ロボットが拾い上げたオブジェクトなどでパフォーマンスが大幅に向上します。
- **optional useFixedBase** (int): 読み込まれたオブジェクトのベースを静的（固定）にします
9
- **optional flags** (int): 以下のフラグは、ビット単位のOR演算子（|）を使用して組み合わせることができます：
  - **URDF_MERGE_FIXED_LINKS**: URDFファイルから固定リンクを削除し、結果のリンクをマージします。これはパフォーマンスに優れています。なぜなら、さまざまなアルゴリズム（関節ボディアルゴリズム、順運動学など）は、固定関節を含む関節の数に対して線形の複雑さを持つためです。
  - **URDF_USE_INERTIA_FROM_FILE**: デフォルトでは、Bulletは衝突形状の質量と体積に基づいて慣性テンソルを再計算します。より正確な慣性テンソルを提供できる場合は、このフラグを使用してください。
  - **URDF_USE_SELF_COLLISION**: デフォルトでは、Bulletは自己衝突を無効にします。このフラグを使用すると、自己衝突を有効にできます。以下のフラグを使用して、自己衝突の動作をカスタマイズできます：
    - **URDF_USE_SELF_COLLISION_INCLUDE_PARENT**: 子と親の間の衝突を有効にします。デフォルトでは無効です。URDF_USE_SELF_COLLISIONフラグと一緒に使用する必要があります。
    - **URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS**: 子リンクとその祖先（親、親の親、ベースまで）の間の自己衝突を破棄します。URDF_USE_SELF_COLLISIONと一緒に使用する必要があります。
  - **URDF_USE_IMPLICIT_CYLINDER**: 滑らかな暗黙の円柱を使用します。デフォルトでは、Bulletは円柱を凸包にテッセレートします。
  - **URDF_ENABLE_SLEEPING**: ボディがしばらく動いていない場合、シミュレーションを無効にすることができます。アクティブなボディとの相互作用により、シミュレーションが再度有効になります。
  - **URDF_INITIALIZE_SAT_FEATURES**: 凸形状の三角形メッシュを作成します。これにより、可視化が改善され、GJK/EPAの代わりに分離軸テスト（SAT）の使用が可能になります。setPhysicsEngineParameterを使用してenableSATを有効にする必要があります。
  - **URDF_USE_MATERIAL_COLORS_FROM_MTL**: URDFファイルではなく、Wavefront OBJファイルからRGB色を使用します。
  - **URDF_ENABLE_CACHED_GRAPHICS_SHAPES**: グラフィックス形状をキャッシュして再利用します。類似のグラフィックスアセットを持つファイルの読み込みパフォーマンスが向上します。
  - **URDF_MAINTAIN_LINK_ORDER**: URDFファイルからのリンクの順序を維持しようとします。URDFファイルで順序がParentLink0、ChildLink1（ParentLink0に接続）、ChildLink2（ParentLink0に接続）の場合、このフラグがないと、順序がP0、C2、C1になる可能性があります。
10
- **optional globalScaling** (float): globalScalingは、URDFモデルにスケールファクターを適用します。
- **optional physicsClientId** (int): 複数のサーバーに接続している場合、どれを選択するか指定できます。

loadURDFは、ボディの一意のID（非負の整数値）を返します。URDFファイルを読み込めない場合、この整数は負の値になり、有効なボディの一意のIDではありません。

デフォルトでは、loadURDFはメッシュ衝突検出に凸包を使用します。静的（質量=0、動かない）メッシュの場合、URDFにタグを追加することでメッシュを凹面にすることができます：`<link concave="yes" name="baseLink">` 例についてはsamurai.urdfを参照してください。URDF形式には他にもいくつかの拡張があり、例を参照して探索できます。PyBulletは、URDFファイルからのすべての情報を処理するわけではありません。サポートされている機能の概要については、例とURDFファイルを参照してください。通常、機能を制御するにはPython APIがあります。各リンクは単一のマテリアルしか持てないため、異なるマテリアルを持つ複数の視覚形状がある場合は、固定関節で接続された別々のリンクに分割する必要があります。これを行うには、Bulletの一部であるOBJ2SDFユーティリティを使用できます。

### loadSDF, loadMJCF

.bullet、.sdf、.mjcfなどの他のファイル形式からオブジェクトを読み込むこともできます。これらのファイル形式は複数のオブジェクトをサポートしているため、戻り値はオブジェクトの一意のIDのリストです。SDF形式の詳細については、http://sdformat.org を参照してください。loadSDFコマンドは、ロボットモデルとジオメトリに関連するSDFの一部の重要な部分のみを抽出し、カメラ、ライトなどに関連する多くの要素を無視します。loadMJCFコマンドは、OpenAI Gymで使用されるMuJoCo MJCF xmlファイルの基本的なインポートを実行します。デフォルトの関節モーター設定に関するloadURDFの重要な注意事項も参照し、setJointMotorControl2を使用することを確認してください。

- **required fileName** (string): 物理サーバーのファイルシステム上のURDFファイルへの相対パスまたは絶対パス
- **optional useMaximalCoordinates** (int): 実験的。詳細についてはloadURDFを参照してください。
- **optional globalScaling** (float): globalScalingはSDFとURDFでサポートされていますが、MJCFではサポートされていません。すべてのオブジェクトは、このスケールファクターを使用してスケーリングされます（リンク、リンクフレーム、関節の取り付け、線形関節の制限を含む）。これは質量には影響せず、ジオメトリにのみ影響します。必要に応じて、changeDynamicsを使用して質量を変更してください。
- **optional physicsClientId** (int): 複数のサーバーに接続している場合、どれを選択するか指定できます。

loadBullet、loadSDF、loadMJCFは、オブジェクトの一意のIDの配列を返します：
- **objectUniqueIds** (list of int): リストには、読み込まれた各オブジェクトのオブジェクトの一意のIDが含まれます。

## saveState, saveBullet, restoreState

以前に保存した状態に復元した後に決定論的なシミュレーションが必要な場合、接触点を含むすべての重要な状態情報を保存する必要があります。saveWorldコマンドではこれには不十分です。restoreStateコマンドを使用して、saveState（メモリ内）またはsaveBullet（ディスク上）で取得したスナップショットから復元できます。

saveStateコマンドは、オプションのclientServerIdを入力として受け取り、状態IDを返します。
saveBulletコマンドは、状態をディスク上の.bulletファイルに保存します。

restoreStateコマンドの入力引数は以下の通りです：
- **optional fileName** (string): saveBulletコマンドを使用して作成された.bulletファイルのファイル名
- **optional stateId** (int): saveStateによって返された状態ID
- **optional clientServerId** (int): 複数のサーバーに接続している場合、どれを選択するか指定できます

ファイル名または状態IDのいずれかが有効である必要があります。restoreStateは、オブジェクトの位置と関節角度を保存された状態にリセットし、接触点情報も復元することに注意してください。restoreStateを呼び出す前に、オブジェクトと制約が設定されていることを確認する必要があります。例についてはsaveRestoreState.pyを参照してください。

### removeState

removeStateは、メモリから以前に保存された状態を削除できます。

### saveWorld

現在の世界の近似スナップショットをPyBullet Pythonファイルとして作成し、サーバーに保存できます。saveWorldは、基本的な編集機能として役立ちます。たとえば、VRでロボット、関節角度、オブジェクトの位置、環境を設定します。後で、PyBullet Pythonファイルを読み込んで世界を再作成できます。Pythonスナップショットには、loadURDFコマンドと、関節角度とオブジェクト変換の初期化が含まれています。すべての設定がワールドファイルに保存されるわけではないことに注意してください。

入力引数は以下の通りです：
- **required fileName** (string): PyBulletファイルのファイル名
- **optional clientServerId** (int): 複数のサーバーに接続している場合、どれを選択するか指定できます

## createCollisionShape/VisualShape

世界にオブジェクトを作成する推奨される最も簡単な方法は、読み込み関数（loadURDF/SDF/MJCF/Bullet）を使用することですが、衝突形状と視覚形状をプログラムで作成し、createMultiBodyを使用してマルチボディを作成することもできます。Bullet Physics SDKのcreateMultiBodyLinks.pyとcreateVisualShape.pyの例を参照してください。
createCollisionShapeの入力パラメータは以下の通りです：

- **required shapeType** (int): GEOM_SPHERE、GEOM_BOX、GEOM_CAPSULE、GEOM_CYLINDER、GEOM_PLANE、GEOM_MESH、GEOM_HEIGHTFIELD
- **optional radius** (float): デフォルト0.5：GEOM_SPHERE、GEOM_CAPSULE、GEOM_CYLINDER用
- **optional halfExtents** (vec3, list of 3 floats): デフォルト[1,1,1]：GEOM_BOX用
- **optional height** (float): デフォルト1：GEOM_CAPSULE、GEOM_CYLINDER用
- **optional fileName** (string): GEOM_MESH用のファイル名。現在はWavefront .objのみ。.objファイル内の各オブジェクト（'o'でマーク）の凸包を作成します
- **optional meshScale** (vec3, list of 3 floats): デフォルト[1,1,1]：GEOM_MESH用
- **optional planeNormal** (vec3, list of 3 floats): デフォルト[0,0,1]：GEOM_PLANE用
- **optional flags** (int): GEOM_FORCE_CONCAVE_TRIMESH：GEOM_MESHの場合、凹面の静的三角形メッシュを作成します。これは動的/移動オブジェクトには使用せず、静的（質量=0）地形のみに使用してください
- **optional collisionFramePosition** (vec3): リンクフレームに対する衝突形状の並進オフセット
- **optional collisionFrameOrientation** (vec4): リンクフレームに対する衝突形状の回転オフセット（クォータニオンx,y,z,w）
- **optional vertices** (list of vec3): ハイトフィールドの定義。heightfield.pyの例を参照してください
- **optional indices** (list of int): ハイトフィールドの定義
- **optional heightfieldTextureScaling** (float): ハイトフィールドのテクスチャスケーリング
- **optional numHeightfieldRows** (int): ハイトフィールドの定義
- **optional numHeightfieldColumns** (int): ハイトフィールドの定義
- **optional replaceHeightfieldIndex** (int): 既存のハイトフィールドを置き換える（高さを更新）（削除して再作成するよりもはるかに高速）
- **optional physicsClientId** (int): 複数のサーバーに接続している場合、どれを選択するか指定できます

戻り値は、衝突形状の非負の整数の一意のID、または呼び出しが失敗した場合は-1です。

### createCollisionShapeArray

createCollisionShapeArrayは、createCollisionShapeの配列バージョンです。使用方法については、snake.pyまたはcreateVisualShapeArray.pyの例を参照してください。

### removeCollisionShape

removeCollisionShapeは、衝突形状の一意のIDを使用して、既存の衝突形状を削除します。

### createVisualShape

衝突形状を作成するのと同様の方法で視覚形状を作成でき、拡散色や鏡面反射色など、視覚的な外観を制御する追加の引数があります。GEOM_MESHタイプを使用する場合、Wavefront OBJファイルを指定でき、視覚形状はマテリアルファイル（.mtl）から一部のパラメータを解析し、テクスチャを読み込みます。大きなテクスチャ（1024x1024ピクセル以上）は、読み込みと実行時のパフォーマンスを低下させる可能性があることに注意してください。

例については、examples/pybullet/examples/addPlanarReflection.pyとcreateVisualShape.pyを参照してください。

入力パラメータは以下の通りです：

- **required shapeType** (int): GEOM_SPHERE、GEOM_BOX、GEOM_CAPSULE、GEOM_CYLINDER、GEOM_PLANE、GEOM_MESH
- **optional radius** (float): デフォルト0.5：GEOM_SPHERE、GEOM_CAPSULE、GEOM_CYLINDERのみ
- **optional halfExtents** (vec3, list of 3 floats): デフォルト[1,1,1]：GEOM_BOXのみ
- **optional length** (float): デフォルト1：GEOM_CAPSULE、GEOM_CYLINDERのみ（length = height）
- **optional fileName** (string): GEOM_MESH用のファイル名。現在はWavefront .objのみ。.objファイル内の各オブジェクト（'o'でマーク）の凸包を作成します
- **optional meshScale** (vec3, list of 3 floats): デフォルト[1,1,1]：GEOM_MESHのみ
- **optional planeNormal** (vec3, list of 3 floats): デフォルト[0,0,1]：GEOM_PLANEのみ
- **optional flags** (int): 未使用/決定待ち
- **optional rgbaColor** (vec4, list of 4 floats): 赤、緑、青、アルファの色成分。それぞれ範囲[0..1]
- **optional specularColor** (vec3, list of 3 floats): 鏡面反射色。赤、緑、青成分。範囲[0..1]
- **optional visualFramePosition** (vec3, list of 3 floats): リンクフレームに対する視覚形状の並進オフセット
- **optional vertices** (list of vec3): objファイルからメッシュを作成する代わりに、頂点、インデックス、uv、法線を提供できます
- **optional indices** (list of int): 三角形インデックス。3の倍数である必要があります
- **optional uvs** (list of vec2): 頂点のuvテクスチャ座標。changeVisualShapeを使用してテクスチャ画像を選択します。uvの数は頂点の数と等しい必要があります
- **optional normals** (list of vec3): 頂点法線。数は頂点の数と等しい必要があります
- **optional visualFrameOrientation** (vec4, list of 4 floats): リンクフレームに対する視覚形状の回転オフセット（クォータニオンx,y,z,w）
- **optional physicsClientId** (int): 複数のサーバーに接続している場合、どれを選択するか指定できます

戻り値は、視覚形状の非負の整数の一意のID、または呼び出しが失敗した場合は-1です。

例については、createVisualShape、createVisualShapeArray、createTexturedMeshVisualShapeを参照してください。

### createVisualShapeArray

createVisualShapeArrayは、createVisualShapeの配列バージョンです。createVisualShapeArray.pyの例を参照してください。

## createMultiBody

世界にオブジェクトを作成する最も簡単な方法は、読み込み関数（loadURDF/SDF/MJCF/Bullet）を使用することですが、createMultiBodyを使用してマルチボディを作成することもできます。
Bullet Physics SDKのcreateMultiBodyLinks.pyの例を参照してください。createMultiBodyのパラメータは、URDFとSDFのパラメータと非常によく似ています。

関節/子リンクなしで単一のベースのみを持つマルチボディを作成することも、関節/子リンクを持つマルチボディを作成することもできます。リンクを提供する場合は、すべてのリストのサイズが同じであることを確認してください（len(linkMasses) == len(linkCollisionShapeIndices)など）。createMultiBodyの入力パラメータは以下の通りです：

- **optional baseMass** (float): ベースの質量（kg単位、SI単位を使用する場合）
- **optional baseCollisionShapeIndex** (int): createCollisionShapeからの一意のID、または-1。複数のマルチボディで衝突形状を再利用できます（インスタンシング）
- **optional baseVisualShapeIndex** (int): createVisualShapeからの一意のID、または-1。視覚形状を再利用できます（インスタンシング）
- **optional basePosition** (vec3, list of 3 floats): ベースのデカルト世界位置
- **optional baseOrientation** (vec4, list of 4 floats): クォータニオン[x,y,z,w]としてのベースの姿勢
- **optional baseInertialFramePosition** (vec3, list of 3 floats): 慣性フレームのローカル位置
- **optional baseInertialFrameOrientation** (vec4, list of 4 floats): 慣性フレームのローカル姿勢、[x,y,z,w]
- **optional linkMasses** (list of float): 質量値のリスト。各リンクに1つ
- **optional linkCollisionShapeIndices** (list of int): 一意のIDのリスト。各リンクに1つ
- **optional linkVisualShapeIndices** (list of int): 各リンクの視覚形状の一意のIDのリスト
- **optional linkPositions** (list of vec3): 親に対するローカルリンク位置のリスト
- **optional linkOrientations** (list of vec4): 親に対するローカルリンク姿勢のリスト
- **optional linkInertialFramePositions** (list of vec3): リンクフレーム内のローカル慣性フレーム位置のリスト
- **optional linkInertialFrameOrientations** (list of vec4): リンクフレーム内のローカル慣性フレーム姿勢のリスト
- **optional linkParentIndices** (list of int): 親リンクのリンクインデックス、またはベースの場合は0
- **optional linkJointTypes** (list of int): 関節タイプのリスト。各リンクに1つ。現在、JOINT_REVOLUTE、JOINT_PRISMATIC、JOINT_SPHERICAL、JOINT_FIXEDタイプがサポートされています
- **optional linkJointAxis** (list of vec3): ローカルフレーム内の関節軸
- **optional useMaximalCoordinates** (int): 実験的。0/falseのままにしておくのが最適です
- **optional flags** (int): loadURDFに渡されるフラグと同様。たとえばURDF_USE_SELF_COLLISION。フラグの説明についてはloadURDFを参照してください
- **optional batchPositions** (list of vec3): ベース位置の配列。多くのマルチボディの高速バッチ作成用。例を参照してください
- **optional physicsClientId** (int): 複数のサーバーに接続している場合、どれを選択するか指定できます

createMultiBodyの戻り値は、非負の一意のID、または失敗の場合は-1です。

例：
```python
cuid = pybullet.createCollisionShape(pybullet.GEOM_BOX, halfExtents = [1, 1, 1])
mass = 0  # static box
pybullet.createMultiBody(mass, cuid)
```

Bullet/examples/pybullet/examplesフォルダのcreateMultiBodyLinks.py、createObstacleCourse.py、createVisualShape.pyも参照してください。

### getMeshData

getMeshDataは、三角形メッシュのメッシュ情報（頂点、インデックス）を返す実験的な未文書化APIです。

- **required bodyUniqueId** (int): ボディの一意のID
- **optional linkIndex** (int): リンクインデックス
- **optional collisionShapeIndex** (int): 複合形状のインデックス。リンクに複数の衝突形状がある場合（getCollisionShapeDataを参照）
- **optional flags** (int): デフォルトでは、PyBulletはグラフィックスレンダリング頂点を返します。異なる法線を持つ頂点は複製されるため、元のメッシュよりも多くの頂点がある場合があります。flags = pybullet.MESH_DATA_SIMULATION_MESHを使用して、シミュレーション頂点を受信できます
- **optional physicsClientId** (int): 複数のサーバーに接続している場合、どれを選択するか指定できます

## stepSimulation, performCollisionDetection

stepSimulationは、衝突検出、制約解決、統合などの単一の順動力学シミュレーションステップですべてのアクションを実行します。デフォルトのタイムステップは1/240秒で、setTimeStepまたはsetPhysicsEngineParameter APIを使用して変更できます。

stepSimulationの入力引数はオプションです：
- **optional physicsClientId** (int): 複数のサーバーに接続している場合、どれを選択するか指定できます

デフォルトでは、stepSimulationに戻り値はありません。

実験的/高度な使用のみ：setPhysicsEngineParameter APIを通じてreportSolverAnalyticsが有効になっている場合、以下の情報が島情報のリストとして返されます：
- **islandId** (int): 島の一意のID
- **numBodies** (list of body unique ids): この島内のボディの一意のID
- **numIterationsUsed** (int): 使用されたソルバーの反復回数
- **remainingResidual** (float): 残差制約エラー

物理サーバーがリアルタイムクロックに基づいて順動力学シミュレーションを自動的に実行できるようにするには、setRealTimeSimulationも参照してください。

### performCollisionDetection

performCollisionDetection APIは、stepSimulationの衝突検出ステージのみを実行します。唯一の引数はphysicsClientIdです。この呼び出しを行った後、getContactPointsを使用できます。

### setRealTimeSimulation

デフォルトでは、物理サーバーは、明示的に'stepSimulation'コマンドを送信しない限り、シミュレーションをステップしません。これにより、シミュレーションの制御決定論を維持できます。setRealTimeSimulationコマンドを使用して、物理サーバーがリアルタイムクロック（RTC）に従ってシミュレーションを自動的にステップするようにすることで、リアルタイムでシミュレーションを実行することができます。リアルタイムシミュレーションを有効にした場合、'stepSimulation'を呼び出す必要はありません。

setRealTimeSimulationはDIRECTモードでは効果がないことに注意してください：DIRECTモードでは、物理サーバーとクライアントが同じスレッドで実行され、すべてのコマンドをトリガーします。GUIモード、バーチャルリアリティモード、TCP/UDPモードでは、物理サーバーはクライアント（PyBullet）とは別のスレッドで実行され、setRealTimeSimulationにより、物理サーバースレッドがstepSimulationへの追加の呼び出しを追加できます。

入力パラメータは以下の通りです：
- **required enableRealTimeSimulation** (int): 0でリアルタイムシミュレーションを無効にし、1で有効にします
- **optional physicsClientId** (int): 複数のサーバーに接続している場合、どれを選択するか指定できます

## getBasePositionAndOrientation

getBasePositionAndOrientationは、デカルト世界座標でボディのベース（またはルートリンク）の現在の位置と姿勢を報告します。姿勢は[x,y,z,w]形式のクォータニオンです。

getBasePositionAndOrientationの入力パラメータは以下の通りです：
- **required objectUniqueId** (int): loadURDFから返されたオブジェクトの一意のID
- **optional physicsClientId** (int): 複数のサーバーに接続している場合、どれを選択するか指定できます

getBasePositionAndOrientationは、位置を3つのfloatのリストとして、姿勢を[x,y,z,w]の順序で4つのfloatのリストとして返します。必要に応じて、getEulerFromQuaternionを使用してクォータニオンをオイラー角に変換できます。

オブジェクトの位置と姿勢をリセットするには、resetBasePositionAndOrientationも参照してください。
これで最初のPyBulletスクリプトが完成します。Bulletには、Bullet/dataフォルダにいくつかのURDFファイルが付属しています。

## resetBasePositionAndOrientation

各オブジェクトのベース（ルート）の位置と姿勢をリセットできます。このコマンドはすべての物理シミュレーションの効果を上書きするため、実行中のシミュレーション中ではなく、開始時のみに実行するのが最適です。線形速度と角速度はゼロに設定されます。非ゼロの線形速度および/または角速度にリセットするには、resetBaseVelocityを使用できます。

resetBasePositionAndOrientationの入力引数は以下の通りです：
- **required bodyUniqueId** (int): loadURDFから返されたオブジェクトの一意のID
- **required posObj** (vec3): 世界空間座標[X,Y,Z]で指定された位置にオブジェクトのベースをリセットします
- **required ornObj** (vec4): 世界空間クォータニオン[X,Y,Z,W]として指定された姿勢にオブジェクトのベースをリセットします
- **optional physicsClientId** (int): 複数のサーバーに接続している場合、どれを選択するか指定できます

戻り値はありません。

## 変換: 位置と姿勢

オブジェクトの位置は、デカルト世界空間座標[x,y,z]で表現できます。オブジェクトの姿勢（または回転）は、クォータニオン[x,y,z,w]、オイラー角[roll, pitch, yaw]、または3x3行列を使用して表現できます。PyBulletは、クォータニオン、オイラー角、3x3行列の間を変換するためのいくつかのヘルパー関数を提供します。さらに、変換を乗算および反転する関数もあります。
### getQuaternionFromEuler と getEulerFromQuaternion

PyBullet APIは、姿勢を表現するためにクォータニオンを使用します。クォータニオンは人間にとってあまり直感的ではないため、クォータニオンとオイラー角の間を変換する2つのAPIがあります。

getQuaternionFromEulerの入力引数は以下の通りです：
- **required eulerAngle** (vec3: list of 3 floats): X、Y、Zオイラー角はラジアン単位で、X軸周りのロール、Y軸周りのピッチ、Z軸周りのヨーを表現する3つの回転を累積します
- **optional physicsClientId** (int): 未使用、APIの一貫性のために追加されました

getQuaternionFromEulerは、クォータニオン、4つの浮動小数点値[X,Y,Z,W]のvec4リストを返します。

### getEulerFromQuaternion

getEulerFromQuaternionの入力引数は以下の通りです：
- **required quaternion** (vec4: list of 4 floats): クォータニオンの形式は[x,y,z,w]です
- **optional physicsClientId** (int): 未使用、APIの一貫性のために追加されました

getEulerFromQuaternionは、3つの浮動小数点値のリスト、vec3を返します。回転順序は、ROS URDF rpy規則と同様に、最初にX軸周りのロール、次にY軸周りのピッチ、最後にZ軸周りのヨーです。

### getMatrixFromQuaternion

getMatrixFromQuaternionは、クォータニオンから3x3行列を作成するユーティリティAPIです。入力はクォータニオンで、出力は行列を表す9つのfloatのリストです。

### getAxisAngleFromQuaternion

getAxisAngleFromQuaternionは、指定されたクォータニオン姿勢の軸と角度表現を返します。
- **required quaternion** (list of 4 floats): 姿勢
- **optional physicsClientId** (int): 未使用、APIの一貫性のために追加されました

### multiplyTransforms, invertTransform

PyBulletは、変換を乗算および反転するためのいくつかのヘルパー関数を提供します。これは、ある座標系から別の座標系に座標を変換するのに役立ちます。

multiplyTransformsの入力パラメータは以下の通りです：
- **required positionA** (vec3, list of 3 floats)
- **required orientationA** (vec4, list of 4 floats): クォータニオン[x,y,z,w]
- **required positionB** (vec3, list of 3 floats)
- **required orientationB** (vec4, list of 4 floats): クォータニオン[x,y,z,w]
- **optional physicsClientId** (int): 未使用、APIの一貫性のために追加されました

戻り値は、位置（vec3）と姿勢（vec4、クォータニオンx,y,z,w）のリストです。

invertTransformの入力および出力パラメータは以下の通りです：
- **required position** (vec3, list of 3 floats)
- **required orientation** (vec4, list of 4 floats): クォータニオン[x,y,z,w]

invertTransformの出力は、位置（vec3）と姿勢（vec4、クォータニオンx,y,z,w）です。

### getDifferenceQuaternion

getDifferenceQuaternionは、開始姿勢から終了姿勢に補間するクォータニオンを返します。
- **required quaternionStart** (list of 4 floats): 開始姿勢
- **required quaternionEnd** (list of 4 floats): 終了姿勢
- **optional physicsClientId** (int): 未使用、APIの一貫性のために追加されました

### getAPIVersion

APIバージョンを年-月-0-日の形式で照会できます。同じAPIバージョンで、同じビット数（32ビット/64ビット）の物理クライアント/サーバー間でのみ接続できます。
オプションの未使用引数physicsClientIdがあり、APIの一貫性のために追加されました。
- **optional physicsClientId** (int): 未使用、APIの一貫性のために追加されました
22

