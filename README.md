ZBrush のプラグイン集です。

# I just want to Use! / 使いたい人
Please access to https://github.com/n-taka/n_ZBrushPlugins/releases and download .zip archive(s).

https://github.com/n-taka/n_ZBrushPlugins/releases にアクセス後、zipアーカイブをダウンロードしてご利用ください。

# I want to extend (develop) plugin(s)! / 改造したい人(開発者の方)
* Clone this repository, then also clone submodules
  * libigl, Eigen
* Build with your preferred IDE
  * Currently we have Visual Studio solution file and Xcode project file.

* このレポジトリをクローン後、submoduleもクローンします
  * libigl, Eigen
* お好きな環境でビルドしてください
  * 現在はVisual StudioとXcode向けのプロジェクトファイルを用意しています。

# Contact / 連絡先
If you encounter some trouble (e.g. plugin doesn't work) or some feature requests, please contact me via github issue. Contact via Twitter is also welcomed.

「アプリが動かない！」「意図した挙動をしない！」「こういう機能があるといいよね！」等ありましたらgithub issueからお気軽にどうぞ。Twitterでも構いません

twitter [@kazutaka_nakash](https://twitter.com/kazutaka_nakash)

## External Dependencies / 外部ライブラリ・依存関係
* libigl
  * https://libigl.github.io/
  * linked with submodule
  * MPL2
* Eigen
  * http://eigen.tuxfamily.org/
  * linked with submodule
  * MPL2
* boost
  * https://www.boost.org/
  * linked with submodule
  * boost software license https://www.boost.org/users/license.html
* CGAL
  * https://www.cgal.org/
  * GPL/LGPL https://www.cgal.org/license.html
* GMP
  * http://gmplib.org/
  * CGAL dependency
  * For windows, use CGAL installer
  * For max, use homebrew
  * LGPL v3/GPL v2
* MPFR
  * http://www.mpfr.org/
  * CGAL dependency
  * For windows, use CGAL installer
  * For max, use homebrew
  * LGPL

  
# Licenses / ライセンス
* MagicaVoxelizer: MPL2
* ThicknessChecker: GPL/LGPL (derived from CGAL)
