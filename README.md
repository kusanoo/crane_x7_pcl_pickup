# Crane_x7 PCL Pick_up

CRANE-X7のためのパッケージ、PCL(Point Cloud Library)を使用してモノを掴む

## Usage

### CRANE-X7のパッケージをインストール

[CRANE-X7のパッケージ](https://github.com/rt-net/crane_x7_ros)のインストール手順に従う


crane_x7_description/urdf/crane_x7_mounting_plate.xacro を my_crane_x7/urdf/crane_x7_mounting_plate.xacro と入れ替える

### 起動するか確認する

```sh
roslaunch crane_x7_description display.launch
```
<img src="https://github.com/kusanoo/my_crane_x7/blob/master/image/rviz.png" width="620px">

上のようになったら成功

### Gazebo,Rvizの起動

```sh
roslaunch crane_x7_gazebo crane_x7_with_table.launch use_effort_gripper:=true　
```
 
### 立ち上がったら、次のコマンドで点群の処理を実行する

```sh
rosrun my_crane_x7 pcl_clustering
``` 

### Rvizで maker を追加する

addを押す

<img src="https://github.com/kusanoo/my_crane_x7/blob/master/image/add.png" width="620px">

By topic を押す

<img src="https://github.com/kusanoo/my_crane_x7/blob/master/image/by_topic.png" width="620px">

MarkerArrayを選択し,OKを押す

<img src="https://github.com/kusanoo/my_crane_x7/blob/master/image/marker.png" width="620px">

Rviz上に紫のクラスタが表示される
  
<img src="https://github.com/kusanoo/my_crane_x7/blob/master/image/marker_2.png" width="620px">
   
### モノを掴むスクリプトの実行

```sh
rosrun my_crane_x7 pick_up_object.py
``` 

動作させると[こちら](https://youtu.be/ZMpj_mBggjw)のような動きになる

## 参考サイト

- rt-netさん
https://github.com/rt-net/crane_x7_ros 

- pcl
http://www.pointclouds.org

- からあげさん
https://karaage.hatenadiary.jp/entry/2017/09/11/073000

- Masaki Hayashiさん
https://www.slideshare.net/payashim/20141004cvsaisentanpclandwhy3dvision

