# Crane_x7 PCL Pick_up

CRANE-X7のためのパッケージ、PCL(Point Cloud Library)を使用してモノを掴む

CRANE-X7のパッケージ
https://github.com/rt-net/crane_x7_ros

## Usage

1. CRANE-X7のパッケージをインストール

2. その中の crane_x7_description/urdf/crane_x7_mounting_plate.xacro を my_crane_x7/urdf/x7_mounting_plate.xacro と入れ替える

3. 起動するか確認する

   ```
   roslaunch crane_x7_description display.launch
   ```

   <img src="https://github.com/my_crane_x7/master/image/crane_rviz.png" width="420px">

4. Gazebo,rvizの起動

   ```
   roslaunch crane_x7_gazebo crane_x7_with_table.launch use_effort_gripper:=true　
   ```
 
5. 立ち上がったら、次のコマンドで点群の処理を実行する

   ```
   rosrun my_crane_x7 pcl_clustering
   ``` 

6. rvizでmakerを追加する
　 6.1 addを押す
