# 1. ビルドツールのインストール（Python 3.13用）
apt-get update && apt-get install -y python3.13-dev build-essential

# 2. 仮想環境の作成（Python 3.13）
python3.13 -m venv venv

# 3. 仮想環境の有効化
source venv/bin/activate

# 4. pip自体のアップグレードとインストール
pip install --upgrade pip setuptools wheel
pip install -r requirements.txt

# 5. OpenGL関連ライブラリのインストール（Debian 13対応）
# 注意: libgl1-mesa-glxはDebian 13で廃止され、libglx-mesa0に置き換えられました
apt-get update && apt-get install -y libosmesa6 libgl1-mesa-dri libglx-mesa0 libgl1

# 5. どこまで必要か解らない環境変数

```bash
# Windowsホストを指す特殊なアドレスを設定 とりあえず、これで動く。
export DISPLAY=host.docker.internal:0.0

# 間接レンダリングを許可する設定
export LIBGL_ALWAYS_INDIRECT=0
# ソフトウェアレンダラ（Mesa）を優先的に使用する設定
export GALLIUM_DRIVER=llvmpipe

# よみ見ると、SOFTWARE が 0 か 1 かの違いか。
export DISPLAY=host.docker.internal:0.0
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe

```