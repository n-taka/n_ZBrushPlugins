ZBrush のプラグイン集です。

# I just want to Use! / 使いたい人
Please access to https://github.com/n-taka/n_ZBrushPlugins/releases and download .zip archive(s).

https://github.com/n-taka/n_ZBrushPlugins/releases にアクセス後、zipアーカイブをダウンロードしてご利用ください。

# I want to extend (develop) plugin(s)! / 改造したい人(開発者の方)
## Dependencies
- [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [libigl](https://libigl.github.io/)
- [CGAL](https://www.cgal.org/) (not all the plugins)

Other indirect dependencies (e.g. boost) will be also installed via package management system.

その他の依存関係(boostなど)に関してはパッケージ管理システムを通じて自動的にインストールされます。

## Installation / プログラミング環境のセットアップ
### Windows
```
# Repository itself and submodules (Eigen, libigl)
git clone --recursive https://github.com/n-taka/n_ZBrushPlugins.git

# If you are new to vcpkg
vcpkg integrate install

# Other dependencies (cgal)
vcpkg install cgal cgal:x64-windows
```
The instruction for installing [vcpkg](https://github.com/Microsoft/vcpkg) is found in [their repository](https://github.com/Microsoft/vcpkg)

[vcpkg](https://github.com/Microsoft/vcpkg)の使い方はMicrosoft公式の[レポジトリ](https://github.com/Microsoft/vcpkg)を参照してください。

### macOS
```
# Repository itself and submodules (Eigen, libigl)
git clone --recursive https://github.com/n-taka/n_ZBrushPlugins.git

# Other dependencies (cgal)
brew install cgal
```
The instruction for installing [homebrew](https://brew.sh/) is found in [their website](https://brew.sh/)

[homebrew](https://brew.sh/)の使い方は公式の[Webサイト](https://brew.sh/)を参照してください。

## Build and run (this part is under construction) / ビルドと実行 (現在手順の確認中です)
### Windows / macOS
We use [cmake](https://cmake.org/) for build.

[cmake](https://cmake.org/)を利用してビルドを行います。
```
cd /path/to/repository/root
cd plugin_you_want_to_build
mkdir build
cd build
cmake ..
make
```

# Contact / 連絡先
If you encounter some trouble (e.g. plugin doesn't work) or some feature requests, please contact me via github issue. Contact via Twitter is also welcomed.

「アプリが動かない！」「意図した挙動をしない！」「こういう機能があるといいよね！」等ありましたらgithub issueからお気軽にどうぞ。Twitterでも構いません

twitter [@kazutaka_nakash](https://twitter.com/kazutaka_nakash)

# Licenses / ライセンス
* MagicaVoxelizer: MPL2
* ThicknessChecker: GPL/LGPL (derived from CGAL)
* ClearanceChekcer: MPL2
