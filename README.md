# gnss2map [![build-test](https://github.com/makotoyoshigoe/gnss2map/actions/workflows/build-test.yaml/badge.svg)](https://github.com/CIT-Autonomous-Robot-Lab/gnss2map/actions/workflows/build-test.yaml)
## package overview
GNSSからの位置情報（緯度、経度、高度）をマップ内の座標（x, y, z）に変換するパッケージです。

## Input/Output
### Input
|Name (Topic)|Type|Description|
|----|----|-----------|
|```/gnss/fix```|```sensor_msgs::msg::NavSatFix```|GNSSの受信情報|

### Output
|Name (Topic)|Type|Description|
|----|----|-----------|
|```/gnss_pose```|```geometry_msgs::msg::PoseWithCovarianceStamped```|マップ座標系におけるGNSSの位置情報|

### Parameter
|Name |Type|Description|
|----|----|-----------|
|```p0```|```std::vector<double>(3)```|マップ内の任意の座標（x, y, z）|
|```gnss0```|```std::vector<double>(3)```|```p0```で取得するGNSSの位置情報(latitude, longitude, altitude)|
|```p1```|```std::vector<double>(2)```|```p0```以外のマップ内の任意の座標（x, y）|
|```gnss1```|```std::vector<double>(2)```|```p1```で取得するGNSSの位置情報(latitude, longitude)|
|```a```|```double```|楕円体の長半径（変更の必要なし）|
|```F```|```double```|楕円体の逆扁平率（変更の必要なし）|
|```m0```|```double```|縮尺係数（変更の必要なし）|
|```ignore_th_cov```|```double```|この値以上の共分散が観測された時```NAN```を出力する|

## Usage
1. ```config/params/gauss_kruger.param.yaml```に適切なパラメータを記述
2. 以下のコマンドを実行する

```
ros2 launch gnss2map gauss_kruger.launch.py
```

## Reference
- [経緯度を換算して平面直角座標、子午線収差角及び縮尺係数を求める計算](https://vldb.gsi.go.jp/sokuchi/surveycalc/surveycalc/algorithm/bl2xy/bl2xy.htm)(last visited: 2024/08/13)