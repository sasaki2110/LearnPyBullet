# 1. 仮想環境の作成
python -m venv venv

# 2. 仮想環境の有効化
source venv/bin/activate

# 3. pip自体のアップグレードとインストール
pip install --upgrade pip
pip install -r requirements.txt

# 4. 不足分のインストール
apt-get update && apt-get install -y libosmesa6 libgl1-mesa-glx libgl1-mesa-dri

# 5. どこまで必要か解らない環境変数

```bash
# Windowsホストを指す特殊なアドレスを設定
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