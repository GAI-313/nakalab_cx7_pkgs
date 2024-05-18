# cx7_bringup
　CRANE X7 を起動するパッケージです。現在は**シミュレーションのみ対応しています**。すぐに使用したい場合はこのパッケージをビルドした後に以下のコマンドを実行してください。
```bash
ros2 launch cx7_bringup cranex7_bringup.launch.py
```

## launch_arguments
　`args`を確認したい場合は以下のコマンドを実行してください。
```bash
ros2 launch cx7_bringup cranex7_bringup.launch.py --show-args
```

- **`is_sim`**<br>
    現在は **シミュレータのみ** の対応となります。実機環境ではまだしようできません。<br>
    デフォルト値は`true`で、シミュレーション環境でロボットを起動するか実機環境でロボットを起動するかを選択します。この引数が `true` である場合、シミュレーション環境でロボットが起動します。<br>
    この引数は今後デフォルト値を `false` にする予定です。

- **`use_d435`**<br>
    RealSense D435 をロボットにマウントしている場合はこの引数に `true` を与えてください。
