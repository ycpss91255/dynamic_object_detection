# 操作步驟

## Network config

- IP and Make
  - IP: 192.168.1.77
  - MakeL: 255.255.255.0

- Restart Network
- check VLP-16 3D LiDAR connect
  - Ping

    ```shell
    $ ping 192.168.1.201
    ```

  - web browser
    <http://192.168.1.201/>

## Reality

- Test VLP-16 3D LiDAR
  - Open VLP-16 3D LiDAR

    ```shell
    $ roslaunch velodyne_pointcloud VLP16_points.launch
    ```

  - Run rviz look VLP-16 3D LiDAR

    ```shell
    $ rosrun rviz rviz -f velodyne
    ```

- Run VLP-16 3D LiDAR and Rviz

    ```shell
    $ roslaunch aws_robomaker_small_warehouse_world small_warehouse_new.launch
    ```

### Reference

- [ubuntu-16.04 ROS-kinetic 下 VLP-16 激光雷達測試詳細教程](https://blog.csdn.net/weixin_44387339/article/details/110350326)

# Old

使用data set

pub kitti data set  : $ rosrun pub_test_data pub_tset_data.py

open rviz           : $ rosrun rviz rviz -d src/velodyne_simulator/velodyne_description/rviz/final.rviz

使用Gazebo
open simple gzworld :$ roslaunch velodyne_description example.launch
open factory gzworld:$ roslaunch aws_robomaker_small_warehouse_world small_warehouse_new.launch sim:=true

kill gazebo         :$ killall gzserver
                    :$ killall gzclient

運行動態偵測
objects detection   : $ rosrun dynamic_object_detection pointCloud.py
objects detection   : $ rosrun dynamic_object_detection point_processor

################################################################

連接實體VLP-16 網路線設定

IPv4 手動
地址	：192.168.1.77
網路遮罩：255.255.255.0
通訊閘	：192.168.1.1
DNS    :0.0.0.0

gsettings Lidar : http://192.168.1.201/

################################################################

資料包介紹

velodyne                    :運行實體VLP-16
dynamic_object_detection    :運行動態物件偵測
pub_test_data               :使用kitti data set
velodyne_simulator          :虛擬環境模擬
obstacle_motion             :虛擬障礙物移動
web                         :控制vlp-16移動

################################################################

參考資料

SLAM之鐳射雷達Velodyne vlp-16使用
https://www.itread01.com/content/1547352842.htmla

velodyne Tutorials
http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16

AI葵 yuotube
https://www.youtube.com/channel/UC7UlsMUu_gIgpqNGB4SqSwQ

工廠 Gazebo github
https://github.com/aws-robotics/aws-robomaker-small-warehouse-world

################################################################

下載 import

run my python file
    pip install pandas

    pip install scikit-learn

    pip install fuzzy-c-means

    pip install cython

run my cpp file
    sudo apt-get install libpcap0.8-dev

python讀中文字
# -*- coding: UTF-8 -*-

