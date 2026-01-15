"""
main.pyのテスト

main()関数が正常に実行できることを確認
"""
import sys
from pathlib import Path

# p08_vision60ディレクトリをパスに追加
p08_dir = Path(__file__).parent.parent
sys.path.insert(0, str(p08_dir))

from my_agent.main import main


def test_main():
    """main()関数が正常に実行できることを確認"""
    print("=" * 60)
    print("main()関数のテスト開始")
    print("=" * 60)
    
    try:
        main()
        print("\n" + "=" * 60)
        print("✓ main()関数の実行が完了しました")
        print("=" * 60)
    except Exception as e:
        print(f"\n❌ エラーが発生しました: {e}")
        import traceback
        traceback.print_exc()
        raise


if __name__ == "__main__":
    test_main()
