# Crane_x7 PCL Pick_up

CRANE-X7のためのパッケージ、PCL(Point Cloud Library)を使用してモノを掴む

CRANE-X7のパッケージ

https://github.com/rt-net/crane_x7_ros

## Usage

### CRANE-X7のパッケージをインストール

その中の crane_x7_description/urdf/crane_x7_mounting_plate.xacro を my_crane_x7/urdf/x7_mounting_plate.xacro と入れ替える

### 起動するか確認する

```sh
roslaunch crane_x7_description display.launch
```
<img src="https://github.com/kusanoo/my_crane_x7/blob/master/image/crane_rviz.png" width="420px">

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

<img src="https://github.com/kusanoo/my_crane_x7/blob/master/image/add.png" width="420px">

By topic を押す

<img src="https://github.com/kusanoo/my_crane_x7/blob/master/image/by_topic.png" width="420px">

MarkerArrayを選択し,OKを押す

<img src="https://github.com/kusanoo/my_crane_x7/blob/master/image/marker.png" width="420px">

Rviz上に紫のクラスタが表示される
  
<img src="https://github.com/kusanoo/my_crane_x7/blob/master/image/marker_2.png" width="420px">
   
### モノを掴むスクリプトの実行

```sh
rosrun my_crane_x7 pick_up_object.py
``` 

動作させると[こちら](https://youtu.be/ZMpj_mBggjw)のような動きになる

