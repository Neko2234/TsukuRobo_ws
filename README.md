# TsukuRobo_ws
授業ロボコン2023用のROSワークスペース

## 依存パッケージ
sudo apt-get install ros-noetic-gmapping ros-noetic-laser-scan-matcher ros-noetic-map-server ros-noetic-navigation

## 参考
### 差動二輪計算
duty比決定：[ロボットの運動学と制御の基礎](https://openrtm.org/openrtm/sites/default/files/6910/191114-06.pdf)  
運動学計算：[高知工科大学の資料](https://www.lab.kochi-tech.ac.jp/robotics/textbook/Robotics/IR2.pdf)

### TF
TF型：[【ROS】TFのbroadcast,lookupと型変換まとめ(C++)](https://ppdr.softether.net/tf-conversion-cpp)

### 地図作成・SLAM
gmappingとかhectorSLAMとか：[【自律型ロボット製作記】#8 amclとlaser_scan_matcherで自己位置推定を試す](https://inomacreate.com/original-robot8/)

###	move_base
costmapのパラメータ：[ROS講座115 costmapの設定を書く](https://qiita.com/srs/items/fba2c83b96d17b2680e6)

## ログメッセージ
それぞれ別の端末で以下のコマンドを実行
```
rosrun rqt_console rqt_console
rosrun rqt_logger_level rqt_logger_level
```
