# nakalab CRANE X7 Packages
　マニピュレーション勉強用のパッケージ。今後実機のCRANE X7 に対応させる予定。

## Install
　このパッケージは **[sciurus17_docker](https://github.com/GAI-313/sciurus17_docker/tree/main)** と互換性があるパッケージです。Docker 環境ですぐにこのパッケージを使用したい場合は、
sciurus17_docker 直下にこのパッケージをクローンすることで使用することができます。詳しくは sciurus17_docker を参照してください。<br>
　一方、通常環境で使用したい場合は、このパッケージに加えて

- [crane_x7_ros](https://github.com/rt-net/crane_x7_ros/tree/humble-devel)
- [crane_x7_description](https://github.com/rt-net/crane_x7_description)

をクローンしてください。
```bash
git clone -b humble-devel https://github.com/rt-net/crane_x7_ros.git
git clone -b ros2 https://github.com/rt-net/crane_x7_description.git
rosdep install -r -y -i --from-paths .
```
必要なパッケージと依存関係を解決したら以下のコマンドを実行してください。
```bash
colcon build --packages-up-to cx7_bringup
```

## Packages

- **[cx7_bringup](cx7_bringup)**<br>
    CRANEX7 の起動パッケージ。現在はシミュレーションのみサポートしています。
