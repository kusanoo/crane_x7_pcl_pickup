# crane_x7_pcl_pickup

CRANE-X7のためのパッケージ、PCL(Point Cloud Library)を使用してモノを掴みます

![demo](https://github.com/kusanoo/crane_x7_pcl_pickup/blob/master/image/demo.gif "demo")


## Usage

### CRANE-X7のパッケージのインストール方法

[CRANE-X7のパッケージ](https://github.com/rt-net/crane_x7_ros)のインストール手順に従います

Gazebo上で動くことを確認してください

### 本パッケージのダウンロード方法

  ```bash
  cd ~/catkin_ws/src
  git clone https://github.com/kusanoo/crane_x7_pcl_pickup.git
  ```

- `catkin_make`を使用してビルドします

  ```bash
  cd ~/catkin_ws && catkin_make
  source ~/catkin_ws/devel/setup.bash
  ```

### ロボットのCameraの設定方法
  
`crane_x7_description/urdf/crane_x7_mounting_plate.xacro` を `crane_x7_pcl_pickup/urdf/crane_x7_mounting_plate.xacro` と入れ替える

以下のコマンドを実行し、確認します

```sh
roslaunch crane_x7_description display.launch
```

<img src="https://github.com/kusanoo/crane_x7_pcl_pickup/blob/master/image/rviz.png" width="640px">

上のようになったら成功です

### Gazebo,Rvizの起動

```sh
roslaunch crane_x7_gazebo crane_x7_with_table.launch use_effort_gripper:=true　
```
 
立ち上がったら、次のコマンドで点群の処理を実行します

```sh
rosrun crane_x7_pcl_pickup pcl_clustering
``` 

### Rvizで maker を追加する

addを押します

<img src="https://github.com/kusanoo/crane_x7_pcl_pickup/blob/master/image/add.png" width="640px">

By topic を押します

<img src="https://github.com/kusanoo/crane_x7_pcl_pickup/blob/master/image/by_topic.png" width="640px">

MarkerArrayを選択し,OKを押します

<img src="https://github.com/kusanoo/crane_x7_pcl_pickup/blob/master/image/marker.png" width="640px">

Rviz上に紫のクラスタが表示されます
  
<img src="https://github.com/kusanoo/crane_x7_pcl_pickup/blob/master/image/marker_2.png" width="640px">
   
### モノを掴むスクリプトの実行

```sh
rosrun crane_x7_pcl_pickup pick_up_object.py
``` 

動作させると[こちらの動画](https://youtu.be/ZMpj_mBggjw)のような動きになります

## 参考サイト

- rt-netさん
https://github.com/rt-net/crane_x7_ros 

- pcl
http://www.pointclouds.org

- からあげさん
https://karaage.hatenadiary.jp/entry/2017/09/11/073000

- Masaki Hayashiさん
https://www.slideshare.net/payashim/20141004cvsaisentanpclandwhy3dvision

