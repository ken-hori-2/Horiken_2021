# Horiken_2021

# Summary

ゴールを目指しながら各経由地を巡回し，予め指定した交差点・探索エリアで必要なノードを起動，それらの状況に対応する行動をさせるプログラムです．（PCLによる歩道と車道の区別や三次元障害物の検出・回避機能の作成中です．）

# DEMO_real
シミュレーションでの実験から実機への移植
Waypoint走行による巡回

https://user-images.githubusercontent.com/73274492/147322778-e60b61c8-b9f0-4158-bf63-67a393f461ea.mp4

画像認識による人物追従

https://user-images.githubusercontent.com/73274492/147325718-6d3a2067-7f2d-4407-975c-6c68806a974c.mp4


# DEMO_sim
つくばチャレンジ(自律移動ロボットの大会会場)を模した　Gazebo(シミュレーター環境)での実行

![image](https://user-images.githubusercontent.com/73274492/147322845-ed88291e-90a8-41ea-bb45-aed94761cca4.png)

<img width="1102" alt="スクリーンショット 2021-12-24 15 04 02" src="https://user-images.githubusercontent.com/73274492/147323019-547ab20b-1208-49ae-aaec-b1865368bb81.png">


https://user-images.githubusercontent.com/73274492/147323239-27b9b485-d4db-4376-99de-3351ca54aeb8.mp4



https://user-images.githubusercontent.com/73274492/147323887-36a394b9-4178-4cbd-ba43-bb70cfa39663.mp4


# DEMO_sim(specific measures)
1.動的環境での障害物回避  
2.csvファイル/actionlibを用いた複数目的地(waypoint)の巡回  
3.長距離走行  
4.実環境を想定した自律移動中の信号認識/物体探索  






https://user-images.githubusercontent.com/73274492/120099695-df816400-c177-11eb-8ee5-2986dd2176b9.mp4

https://user-images.githubusercontent.com/73274492/120099311-ba8bf180-c175-11eb-9e3c-b4f3d039daeb.mp4

https://user-images.githubusercontent.com/73274492/120099565-186d0900-c177-11eb-8936-1bdf1f128073.mp4

https://user-images.githubusercontent.com/73274492/120099497-af859100-c176-11eb-9463-85854a33fd8c.mp4

# 不明領域探索





# 機能の呼び出しの概略図  
<img width="271" alt="2021-03-01 (1)" src="https://user-images.githubusercontent.com/73274492/109474570-00beaf80-7ab8-11eb-995d-d6e1e00171c0.png"><img width="172" alt="2021-03-01" src="https://user-images.githubusercontent.com/73274492/109474914-5b580b80-7ab8-11eb-856d-1d19a7ebc8bf.png">


Crossroad1/2(交差点1/2)の入力  
DetectArea（物体検出エリア）の入力  
![図1](https://user-images.githubusercontent.com/73274492/120101653-093f8880-c182-11eb-9b1e-4c6170b94c66.png)


状態の移行  
![1](https://user-images.githubusercontent.com/73274492/120101584-aea62c80-c181-11eb-858f-e407c06563d8.png)







# Features
自律移動の途中でその都度必要なノード呼び出しにはactionlibを用いてServer/Clientによる構造を用いて呼び出しています．
Serverは常に起動させておき，smachでactionlibを用いた特定の位置指定をしたエリアにClientを配置することでノードを途中起動させています．


交差点では事前に指定したWPによって交差点領域内に入る前に一時停止，信号認識ノード起動(A_srv03.py)

 →信号機が赤の間は停止し，青になったら自律移動再開．

探索エリアでは事前に指定したポイントで探索ノード起動(Practice_server033.py)

 →探索対象が見つかるまで何度か探索．発見したら探索対象に0.5mまで接近，検知完了とし本来のゴールを目指す（制限時間と再試行回数設定 ：制限時間内では見つけるまで車体回転し続け，それを設定回数分だけ繰り返し，検知できたら次の行動へ移る．検知できなくても探索を諦め次の行動へ移る）．
# Requirement
 
動かすのに必要なライブラリなど
 
(ros-melodic)

boundingbox_msgs

darknet_ros

navigation

slam_gmapping

openni_camera

gazebo_ros_demos

robot_arm_ros/diff_drive

executive_smach

executive_smach_visualization


 
# Installation
 
Requirementで列挙したライブラリなどのインストール方法
 
```bash
sudo apt-get install ros-melodic-navigation
sudo apt-get install ros-melodic-slam-gmapping
git clone --recursive https://github.com/leggedrobotics/darknet_ros.git
git clone https://github.com/ros-simulation/gazebo_ros_demos.git
git clone https://github.com/parambeernegi/robot_arm_ros
git clone https://github.com/ros-drivers/openni_camera.git
git clone https://github.com/ros/executive_smach.git
git clone https://github.com/ros-visualization/executive_smach_visualization.git
```
 
# Usage
 
DEMOの実行方法など、"Horiken_2021"の基本的な使い方
 
```bash
cd ~/catkin_ws/src
git clone https://github.com/ken-hori-2/Horiken_2021.git
cd ~/catkin_ws
catkin build
roslaunch PF1 diff_drive.launch
roslaunch ros_beginner mighty.launch
roslaunch ros_beginner CSV.launch
rosrun depth_estimater test.py
rosrun ros_beginner sm.py
rosrun smach_viewer smach_viewer.py

PF1 つくばチャレンジ シミュレーション/地図あり
roslaunch PF1_tukuba PF1_final_amcl.launch
roslaunch PF1_tukuba diff_drive_amcl.launch
roslaunch PF1_tukuba PF1_final.launch

不明領域探索（地図無し）
roslaunch PF1 explore_lite.launch
```
 
# Note
 sm.pyを実行すると動き始めてしまうのでまとめられるもの以外は一つにまとめずに別々で起動させています．
# Author

* 作成者 ken
* 所属 Klab
* E-mail
 
# License
