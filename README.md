# gnss2map
GNSSからの情報をマップ内の座標に変換するパッケージ

# パラメータ
- p0(double_array, default:[164.29392211251695, 47.40068430031194]) 
    - マップ内の任意の座標[x, y]

- gnss0(double_array, default: [35.68933509666667, 140.02148523333332])
    - p0にいるときに取得するGNSSからの座標[緯度, 経度]

- p1(double_array, default:[59.33165740966797, 159.38681030273438]) 
    - p0以外のマップ内の任意の座標[x, y]

- gnss1(double_array, default: [35.688686698333335, 140.02004203333334])
    - p1にいるときに取得するGNSSからの座標[緯度, 経度]

# トピック
## Subscribed Topics
- gnss/fix([sensor_msgs::msg::NavSatFix](https://docs.ros2.org/latest/api/sensor_msgs/msg/NavSatFix.html))
    - GNSSのデータ
## Published Topics
- odom/gnss([nav_msgs::msg::Odometry](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html))
    - /gnss/fixから変換したmap内の座標

# 実行方法
1. 適切なパラメータを設定する
2. 以下のコマンドを実行する \
```ros2 launch gnss2map gauss_kruger.launch.py```
