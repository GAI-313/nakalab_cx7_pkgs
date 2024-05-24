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

## Quick Start
すぐに使用したい場合は以下のコマンドを実行してください。
```bash
ros2 launch cx7_bringup cranex7_bringup.launch.py
```

## move_to_go service
　`move_to_go` サービスは事前に設定したロボットアームの姿勢へ遷移させるサービスです。以下のコマンドを実行するとロボットアームは`home`姿勢へ遷移します。
```
ros2 service call /move_to_pose cx7_interface/srv/MoveToPose "{goal_state: home}"
```
　他の姿勢へ遷移させたい場合はコマンドの `home` を別の姿勢名に変えてください。例えば、`push_hand` 姿勢にしたい場合は以下のコマンドを実行します。
```
ros2 service call /move_to_pose cx7_interface/srv/MoveToPose "{goal_state: push_hand}"
```
現在本パッケージで使用できる姿勢は以下のとおりです。

- **Group arm**
    - home
    - gaze
    - push_hand
    - picking

新たに姿勢を追加したい場合は[cx7_config](cx7_config)を参照してください。

## Packages
- **[cx7_bringup](cx7_bringup)**<br>
    CRANEX7 の起動パッケージ。現在はシミュレーションのみサポートしています。
- **[cx7_config](cx7_config)**<br>
    CRANE X7 のコンフィグファイルをまとめるパッケージ。姿勢設定などをまとめています。
- **[cx7_controller](cx7_controller)**<br>
    CRANE X7 を動作させるために必要なノードをまとめるパッケージ
- **[cx7_interface](cx7_interface)**<br>
    `cx7_...` パッケージで使用するインターフェースファイルをまとめるパッケージ
