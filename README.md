# TsukuRobo_ws
授業ロボコン2023用のROSワークスペース

## 依存パッケージ
sudo apt-get install ros-noetic-gmapping ros-noetic-laser-scan-matcher

## 参考
### 差動二輪計算
[運動学計算](https://openrtm.org/openrtm/sites/default/files/6910/191114-06.pdf)  
[duty比決定](https://www.lab.kochi-tech.ac.jp/robotics/textbook/Robotics/IR2.pdf)

### TF
[TF型](https://ppdr.softether.net/tf-conversion-cpp)

### 地図作成・SLAM
[gmappingとかhectorSLAMとか](https://inomacreate.com/original-robot8/)

## ログメッセージ
それぞれ別の端末で以下のコマンドを実行
```
rosrun rqt_console rqt_console
rosrun rqt_logger_level rqt_logger_level
```
