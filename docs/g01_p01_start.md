既存のPythonコンテナがあり、その中のプロジェクトフォルダで作業されるとのこと、承知いたしました。実務に近い構成で、ステップ1の詳細を再構築します。

---

## ステップ 1：プロジェクト初期化とPyBullet接続確認

このステップでは、コンテナ内のディレクトリに仮想環境（venv）を構築し、PyBulletをインストールして、物理エンジンとの接続をテストします。

### 1. 依存関係の定義 (`requirements.txt`)

プロジェクトフォルダの直下に作成してください。PyBullet本体と、数値計算やデータ管理に便利なライブラリをセットにします。

```text
# requirements.txt
pybullet==3.2.6
numpy          # 物理演算の結果を扱う際にほぼ必須となります

```

### 2. 環境構築の手順（ターミナル操作）

コンテナ内のプロジェクトフォルダに移動して、以下のコマンドを実行します。

```bash
# 1. 仮想環境の作成
python -m venv venv

# 2. 仮想環境の有効化
source venv/bin/activate

# 3. pip自体のアップグレードとインストール
pip install --upgrade pip
pip install -r requirements.txt

```

---

### 3. 初回テストコード：`check_connection.py`

仮想環境が有効な状態で、以下のコードを実行します。ここでは「画面なし（DIRECTモード）」で物理サーバーを起動し、正常にライブラリがロードできるかを確認します。

```python
import pybullet as p
import pybullet_data
import time

def main():
    # --- 物理サーバーへの接続 ---
    # コンテナ内ではGUIがないため、まずは p.DIRECT を使用します
    client_id = p.connect(p.DIRECT)
    
    if client_id < 0:
        print("物理サーバーへの接続に失敗しました。")
        return

    # --- 基本設定 ---
    # PyBulletに付属している標準データ（床やロボットモデル）のパスを追加
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # 重力を地球（-9.81）に設定
    p.setGravity(0, 0, -9.81)

    # --- データのロード ---
    # 床(plane)を召喚
    plane_id = p.loadURDF("plane.urdf")
    # ロボット(R2D2)を空中(高さ1m)に召喚
    robot_id = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 1.0])

    print(f"接続成功！ Client ID: {client_id}")
    print(f"標準データパス: {pybullet_data.getDataPath()}")

    # --- 物理シミュレーションの進展テスト ---
    for i in range(50):
        p.stepSimulation()
        if i % 10 == 0:
            pos, _ = p.getBasePositionAndOrientation(robot_id)
            print(f"ステップ {i}: ロボットの現在の高さ = {pos[2]:.4f}m")

    # --- 切断 ---
    p.disconnect()
    print("シミュレーションを正常に終了しました。")

if __name__ == "__main__":
    main()

```

---

### 4. このステップの学習ポイント

#### ① クライアント・サーバー・モデル（PDF 4ページ参照）

PyBulletは、あなたの書くPythonコード（**クライアント**）が、裏側で動く物理演算エンジン（**サーバー**）に命令を出す構造になっています。

* `p.connect(p.DIRECT)` は、画面を表示する機能を持たないサーバーを同じプロセス内で立ち上げる命令です。

#### ② URDFデータの所在

`pybullet_data.getDataPath()` を使うことで、自分でモデルデータを用意しなくても、練習用のロボットや床をすぐに呼び出せます。

---

### 次のアクション

まずは上記手順で `check_connection.py` を実行し、**「ロボットの現在の高さ」が徐々に減っていく（＝重力で落下している）** ログが出ることを確認してください。

これが確認できたら、次は **「② DockerコンテナからWindowsへGUIを転送する（VcXsrv）」** の設定に進み、実際にロボットが落ちる様子を目で確認できる環境を作っていきましょう。準備ができたら教えてください。