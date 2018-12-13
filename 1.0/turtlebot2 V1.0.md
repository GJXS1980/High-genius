## turtlebot2
##### kobuki依赖:
```bash
#下载kobuki
git clone https://github.com/yujinrobot/kobuki.git
###############################################################################
#安装依赖
sudo apt-get install ros-kinetic-ecl-exceptions
sudo apt-get install ros-kinetic-ecl-threads
sudo apt-get install ros-kinetic-kobuki-msgs 
sudo apt-get install ros-kinetic-yocs-controllers 
sudo apt-get install ros-kinetic-ecl-geometry
sudo apt-get install ros-kinetic-kobuki-dock-drive
sudo apt-get install ros-kinetic-kobuki-driver
sudo apt-get install ros-kinetic-ecl-streams
###############################################################################
###############################要安装的#######################################
sudo apt-get install ros-kinetic-realsense-camera
sudo apt-get install ros-kinetic-librealsense
sudo apt-get install linux-headers-generic
sudo sh -c 'echo "deb-src http://us.archive.ubuntu.com/ubuntu/ xenial main restricted
deb-src http://us.archive.ubuntu.com/ubuntu/ xenial-updates main restricted
deb-src http://us.archive.ubuntu.com/ubuntu/ xenial-backports main restricted universe multiverse
deb-src http://us.archive.ubuntu.com/ubuntu/ xenial-security main restricted" > \ /etc/apt/sources.list.d/official-source-repositories.list'
sudo apt-get update
sudo apt-get install build-essential g++
sudo apt-get install ros-kinetic-librealsense
sudo apt-get install ros-kinetic-realsense-camera
###############################################################################
```

```bash
#终端安装
sudo apt-get install ros-kinetic-turtlebot-gazebo \
ros-kinetic-turtlebot-actions \
ros-kinetic-turtlebot-apps \
ros-kinetic-turtlebot-calibration \
ros-kinetic-turtlebot-create \
ros-kinetic-turtlebot-follower \
ros-kinetic-turtlebot-interactions \
ros-kinetic-turtlebot-msgs \
ros-kinetic-turtlebot-navigation \
ros-kinetic-turtlebot-rapps
###############################################################################
```

##### kobuki底盘控制
###### 键盘控制
```
roslaunch kobuki_node minimal.launch --screen

roslaunch kobuki_keyop keyop.launch --screen
```

###### 手柄控制
罗技手柄连接:
驱动安装:
```bash
sudo apt-get install ros-kinetic-joy
sudo apt-get install ros-kinetic-joystick-drivers
rosdep install joy
rosmake joy

#更改js0的root群组为dialout群组
sudo chgrp dialout /dev/input/js0

#测试(按下手柄按键，终端上会显示按键信息)
sudo jstest /dev/input/js0

```
安装教程:[罗技F710无线手柄在ROS下的安装使用](https://blog.csdn.net/hengheng_51/article/details/80246918)

修改logitech.launch文件:
```launch
<launch>
  <!--  smooths inputs from cmd_vel_mux/input/teleop_raw to cmd_vel_mux/input/teleop -->
  <include file="$(find turtlebot_teleop)/launch/includes/velocity_smoother.launch.xml"/>

  <node pkg="turtlebot_teleop" type="turtlebot_teleop_joy" name="turtlebot_teleop_joystick">
    <param name="scale_angular" value="1.5"/>
    <param name="scale_linear" value="0.5"/>
    <param name="axis_deadman" value="4"/>
    <param name="axis_linear" value="1"/>
    <param name="axis_angular" value="0"/>

    <remap from="turtlebot_teleop_joystick/cmd_vel" to="teleop_velocity_smoother/raw_cmd_vel"/>
  </node>

  <node pkg="joy" type="joy_node" name="joystick" />

</launch>
```

启动手柄控制:
```bash
roslaunch turtlebot_bringup minimal.launch
roslaunch turtlebot_teleop logitech.launch

```
按住LB键激活然后左摇杆控制机器人移动

教程:
[ROS indigo 使用游戏手柄控制turtlebot2](https://blog.csdn.net/qq_25349629/article/details/80076273)
[使用罗技f710控制turtlebot小车](https://blog.csdn.net/lxn9492878lbl/article/details/80308351)

#### turtlebot2仿真模型的加载
1.turtlebot2仿真模型的加载 
```
roslaunch turtlebot_gazebo turtlebot_world.launch

roslaunch turtlebot_teleop logitech.launch
```
2.将kinect V2 模型替换kinect V1模型
3.仿真导航建图

##### kinect V2驱动安装
链接:[ubuntu16.04安装kinect驱动](https://www.gjxslisa.club/2018/07/18/Kinect-driver/)

```bash

sudo apt-get install nvidia-opencl-dev nvidia-modprobe nvidia-libopencl1-384 nvidia-opencl-icd-384

sudo apt-get install nvidia-cuda-toolkit
```

启动kinect
```bash
roslaunch kinect2_bridge kinect2_bridge.launch
rosrun kinect2_viewer kinect2_viewer
```

参考链接:[SurfaceBook + Ubuntu16.04 安装 Kinect V2 & libfreenect2pclgrabber](https://blog.csdn.net/weixin_41737678/article/details/81091717)

##### kinect V2的标定
标定教程:[calibrating](https://github.com/code-iai/iai_kinect2/tree/master/kinect2_calibration#calibrating-the-kinect-one)

教程 2: [Ros环境下 kinect标定 -- ROS功能包](https://blog.csdn.net/Siyuada/article/details/78981555)

```bash
#创建校准数据文件
mkdir ~/kinect_cal_data; cd ~/kinect_cal_data

#[ INFO] [Kinect2Bridge::initDevice] device serial:034011551247   
#在我的/home/robot/catkin_ws/src/iai_kinect2/kinect2_bridge/data的文件夹里建立一个文件夹，取名叫 034011551247 
rosrun kinect2_bridge kinect2_bridge _fps_limit:=2

###标定彩色摄像头：
#记录彩色摄像机的图像
rosrun kinect2_calibration kinect2_calibration chess5x7x0.03 record color
#校准内在函数
rosrun kinect2_calibration kinect2_calibration chess5x7x0.03 calibrate color

####标定红外：
#记录红外摄像机的图像
rosrun kinect2_calibration kinect2_calibration chess5x7x0.03 record ir
#校准红外摄像机的内在因素
rosrun kinect2_calibration kinect2_calibration chess5x7x0.03 calibrate ir

###帧同步标定：
rosrun kinect2_calibration kinect2_calibration chess5x7x0.02 record sync
#按几次空格键记录
rosrun kinect2_calibration kinect2_calibration chess5x7x0.02 calibrate sync

###深度标定：
####采集100张图
rosrun kinect2_calibration kinect2_calibration chess5x7x0.02 record depth
rosrun kinect2_calibration kinect2_calibration chess5x7x0.02 calibrate depth

```
然后再把calib_color.yaml calib_ir.yaml calib_pose.yaml calib_depth.yaml拷贝到/home/robot/catkin_ws/src/iai_kinect2/kinect2_bridge/data/034011551247文件夹中

教程链接;[kinect2标定 Ubuntu16.04 ROS kinetic](https://blog.csdn.net/qingdu007/article/details/79204115)

### 方法1修改kinect2的slam
参考链接:[使用Kinect v2进行导航建图（gmapping+amcl)](https://blog.csdn.net/qq_16481211/article/details/84763291)
[turtlebot2结合Kinect2导航](https://blog.csdn.net/sinat_29754847/article/details/80660512)

建图:
```bash
roslaunch turtlebot_bringup minimal.launch
roslaunch turtlebot_slam kinect2_gmapping.launch
roslaunch turtlebot_slam kinect2_gmapping_rviz_view.launch
roslaunch turtlebot_teleop logitech.launch

```

保存地图
```bash
rosrun map_server map_saver -f mymap
```

导航
```bash
roslaunch turtlebot_navigation amcl_demo.launch map_file:=/path/my_map.yaml
```


### 方法2turtlebot_k2_exploration_3d
安装依赖 
```bash
sudo apt-get install ros-kinetic-octomap*
```

在turtlebot上运行
```bash
roslaunch turtlebot_exploration_3d minimal_explo.launch
roslaunch turtlebot_exploration_3d turtlebot_gmapping.launch
rosrun turtlebot_exploration_3d turtlebot_exploration_3d
```

```bash
roslaunch turtlebot_exploration_3d exploration_rviz.launch
```

源码:[turtlebot_k2_exploration_3d](https://github.com/QinZiwen/turtlebot_k2_exploration_3d)

### turtlebot2_roam
源码:[turtlebot2_roam](https://github.com/QinZiwen/turtlebot2_roam)

网站:[RTAB-Map](http://introlab.github.io/rtabmap/)


### 问题解决
问题1:出现未定义应用
```bash
#1.重新安装ROS
#2.更新一下
sudo apt-get upgrade
```

问题2:[Error] [Freenect2Impl] failed to open Kinect v2: @2:6 LIBUSB_ERROR_ACCESS Access denied (insufficient permissions)
```bash
cd ~/libfreenect2
sudo cp platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
```
重新插入kinect
```bash
cd ~/libfreenect2/build/bin
./Protonect 
```

问题3:Failed to load nodelet '/kinect2_bridge` of type `kinect2_bridge/kinect2_bridge_nodelet` to manager 

```bash
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE="Release"
rospack profile

#打开.bashrc文件,添加下面命令
source /home/gjxs/catkin_ws/devel/setup.bash
```

测试:
```bash
roslaunch kinect2_bridge kinect2_bridge.launch

rosrun kinect2_viewer kinect2_viewer
```


问题4:ERROR: cannot launch node of type [base_controller/base_controller]: base_controller
```bash


```


## 激光雷达
安装激光雷达功能包(A2,A3)
```bash
mkdir -p ~/rplidar_ws/src
cd ~/rplidar_ws/src
git clone https://github.com/Slamtec/rplidar_ros.git
cd rplidar_ros && git checkout 1.7.0
cd ~/rplidar_ws && catkin_make

echo "source ~/rplidar_ws/devel/setup.bash" >> ~/.bashrc 
#使环境生效
source ~/.bashrc
```

##### 设置串口访问权限
创建turtlebot的串口别名
新建/etc/udev/rules.d/57-kobuki.rules
```bash
cd /etc/udev/rules.d
sudo touch 57-kobuki.rules && sudo gedit 57-kobuki.rules
```

内容如下:
```
# On precise, for some reason, USER and GROUP are getting ignored.
# So setting mode = 0666 for now.
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="kobuki*", MODE:="0666", GROUP:="dialout", SYMLINK+="kobuki"
```

创建rplidar的串口别名
新建 /etc/udev/rules.d/rplidar.rules文件
```bash
cd /etc/udev/rules.d/
sudo touch rplidar.rules && sudo gedit rplidar.rules
```

内容如下：
```
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="rplidar"  
```

增加对串口的默认访问权限
```
sudo usermod -a -G dialout 用户名
```
用户名为终端@之前的名字

配置生效
使udev配置生效
```
sudo service udev reload
sudo service udev restart
```

检查rplidar的串口的权限：
```
ls -l /dev |grep ttyUSB
#添加写权限:(例如/dev/ttyUSB0）
sudo chmod 666 /dev/ttyUSB0
```
出现'crw-rw----  1 root dialout 188,   0 12月 12 09:18 ttyUSB0'就算成功了

I.运行rplidar节点并在rviz中查看
```bash
roslaunch rplidar_ros view_rplidar.launch
```

II.运行rplidar节点并使用测试应用程序查看
```bash
roslaunch rplidar_ros rplidar.launch
rosrun rplidar_ros rplidarNodeClient
```
应该在控制台中看到rplidar的扫描结果

##### 激光雷达应用到slam上
修改文件
修改turtlebot_apps/turtlebot_navigation/launch/gmapping_demo.launch，内容如下：
```launch
<launch>
  <include file="$(find turtlebot_bringup)/launch/minimal.launch">
  </include>
  <node name="rplidarNode"          pkg="rplidar_ros"  type="rplidarNode" output="screen">
  <param name="serial_port"         type="string" value="/dev/rplidar"/>  
  <param name="serial_baudrate"     type="int"    value="115200"/>
  <param name="frame_id"            type="string" value="laser_frame"/>
  <param name="inverted"            type="bool"   value="false"/>
  <param name="angle_compensate"    type="bool"   value="true"/>
  </node>
  <include file="$(find turtlebot_navigation)/launch/includes/gmapping.launch.xml"/>
  <include file="$(find turtlebot_navigation)/launch/includes/move_base.launch.xml"/>
 <node pkg="tf" type="static_transform_publisher" name="base_link_to_laser4" 
    args="0.0 0.0 0.2 0.0 3.1415926 0.0 /base_link /laser_frame 40" />
</launch>
```

修改turtlebot_apps/turtlebot_navigation/launch/includes/gmapping.launch.xml，内容如下：
```launch
<launch>
  <arg name="scan_topic"  default="scan" />
  <arg name="base_frame"  default="base_footprint"/>
  <arg name="odom_frame"  default="odom"/>

  <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
    <param name="base_frame" value="$(arg base_frame)"/>
    <param name="odom_frame" value="$(arg odom_frame)"/>
    <param name="map_update_interval" value="0.01"/>
    <param name="maxUrange" value="4.0"/>
    <param name="maxRange" value="5.0"/>
    <param name="sigma" value="0.05"/>
    <param name="kernelSize" value="3"/>
    <param name="lstep" value="0.05"/>
    <param name="astep" value="0.05"/>
    <param name="iterations" value="5"/>
    <param name="lsigma" value="0.075"/>
    <param name="ogain" value="3.0"/>
    <param name="lskip" value="0"/>
    <param name="minimumScore" value="30"/>
    <param name="srr" value="0.01"/>
    <param name="srt" value="0.02"/>
    <param name="str" value="0.01"/>
    <param name="stt" value="0.02"/>
    <param name="linearUpdate" value="0.05"/>
    <param name="angularUpdate" value="0.0436"/>
    <param name="temporalUpdate" value="-1.0"/>
    <param name="resampleThreshold" value="0.5"/>
    <param name="particles" value="8"/>
  <!--
    <param name="xmin" value="-50.0"/>
    <param name="ymin" value="-50.0"/>
    <param name="xmax" value="50.0"/>
    <param name="ymax" value="50.0"/>
  make the starting size small for the benefit of the Android client's memory...
  -->
    <param name="xmin" value="-1.0"/>
    <param name="ymin" value="-1.0"/>
    <param name="xmax" value="1.0"/>
    <param name="ymax" value="1.0"/>

    <param name="delta" value="0.05"/>
    <param name="llsamplerange" value="0.01"/>
    <param name="llsamplestep" value="0.01"/>
    <param name="lasamplerange" value="0.005"/>
    <param name="lasamplestep" value="0.005"/>
    <remap from="scan" to="$(arg scan_topic)"/>
  </node>
</launch>
```

运行launch,创建地图
```bash
roslaunch turtlebot_navigation gmapping_demo.launch

#启动下面的任意一个
roslaunch turtlebot_rviz_launchers view_navigation.launch
roslaunch turtlebot_slam kinect2_gmapping_rviz_view.launch

roslaunch turtlebot_teleop logitech.launch
```

保存地图
```bash
rosrun map_server map_saver -f 地图的保持位置
```

##### 录制数据包和建图
使用方法:
```bash
cd ~/turtlebot_ws/src/TurtleBot-Tutorial/rosbag
roslaunch recore_bag.launch
#记录数据
rosrun rosbag record -a
```
录制完成后，用CTL+C取消结束数据的记录 
修改对应的bag文件，根据录的rosbag进行gmapping生成地图(修改路径)
```bash
cd ~/turtlebot_ws/src/TurtleBot-Tutorial/rosbag
roslaunch gmapping_bag.launch 
```

##### 确定Rplidar和turtlebot的坐标关系
Rplidar的坐标系：三角形的电机为X轴的正向，顺时针90度为Y轴的正向，因为是二维激光，没有Z轴。

turtlebot坐标系：右手原则， 前方（带碰撞挡板）X轴的正向，逆时针90度为Y轴的正向，朝上为Z轴的正向。

由于Rplidar的坐标系和turtlebot坐标系不一致，所以需要沿X轴做180度旋转。

安装Rplidar的方法： 把Rplidar固定在turtlebot上方的最中心的位置，Rplidar三角形电机的朝向和turtlebot 前方碰撞挡板的中心位置一致，测量Rplidar和turtlebot底座的高度为20cm。

发布Rplidar和turtlebot的坐标关系有两种方式：
在turtlebot_description/urdf/turtlebot_library.urdf.xacro的</robot>标签之前加入如下内容
```xml
 <joint name="laser" type="fixed">
    <origin xyz="0.00 0.00 0.1704" rpy="0 3.1415926 0" />
    <parent link="base_link" />
    <child link="base_laser_link" />
  </joint>

  <link name="base_laser_link">
    <visual>
      <geometry>
        <box size="0.00 0.05 0.06" />
      </geometry>
      <material name="Green" />
    </visual>
    <inertial>
      <mass value="0.000001" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0"
        iyy="0.0001" iyz="0.0"
        izz="0.0001" />
    </inertial>

    
  </link>
```

源码地址:[rplidar_ros](https://github.com/Slamtec/rplidar_ros)
教程:[About RPLIDAR](https://github.com/robopeak/rplidar_ros/wiki)
参考教程: [turtlebot2+激光雷达](https://blog.csdn.net/eaibot/article/details/51219032)

### 语音控制
##### 英文语音控制
1.安装PocketSphinx语音识别包
```bash
sudo apt-get install gstreamer1.0-pocketsphinx
sudo apt-get install ros-kinetic-audio-common
sudo apt-get install libasound2
sudo apt-get install gstreamer0.10-gconf
sudo apt-get install libasound-dev
sudo apt-get install python-pyaudio

#安装pocketsphinx功能包
mkdir -p ~/sound_ws/src 
cd ~/sound_ws/src 
git clone https://github.com/Pankaj-Baranwal/pocketsphinx.git
cd ~/sound_ws && catkin_make

#环境设置
echo "source ~/sound_ws/devel/setup.bash" >> ~/.bashrc 
#使环境生效
source ~/.bashrc
```
测试
```bash
roslaunch turtlebot_gazebo turtlebot_world.launch

```

[Turtlebot Autonomous SLAM and Feature Tracking on ROS](https://github.com/lb5160482/Turtlebot-Autonomous-SLAM-and-Feature-Tracking-on-ROS)

[ros_voice_system](https://github.com/stefantasy/ros_voice_system)


教程:[pocketphinx中的“Okay Google”模式](https://medium.com/@PankajB96/okay-google-mode-in-pocketsphinx-6acdb5feafe9)

[turtlebot入门-语音控制](https://www.ncnynl.com/archives/201609/870.html)
[PocketSphinx语音识别和turtlebot的语音控制--18](https://www.cnblogs.com/zxouxuewei/p/5273178.html)
[Turtlebot语音遥操作](https://edu.gaitech.hk/turtlebot/speech-doc.html)

##### 中文语音控制
