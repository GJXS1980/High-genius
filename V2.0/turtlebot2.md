## turtlebot2
##### kinect V2驱动安装
链接:[ubuntu16.04安装kinect驱动](https://www.gjxslisa.club/2018/07/18/Kinect-driver/) <br>
启动kinect
```bash
roslaunch kinect2_bridge kinect2_bridge.launch
rosrun kinect2_viewer kinect2_viewer
```

##### kobuki依赖:
```bash
#安装依赖
sudo apt-get install ros-$ROS_DISTRO-ecl-exceptions \
                     ros-$ROS_DISTRO-ecl-threads \
                     ros-$ROS_DISTRO-ecl-geometry \
                     ros-$ROS_DISTRO-ecl-streams \
                     ros-$ROS_DISTRO-librealsense \
                     ros-$ROS_DISTRO-gazebo-ros-control \
                     ros-$ROS_DISTRO-kobuki-core \
                     ros-$ROS_DISTRO-linux-peripheral-interfaces \
                     ros-$ROS_DISTRO-moveit-* \
                     ros-$ROS_DISTRO-yujin-ocs \
                     ros-$ROS_DISTRO-bfl \
                     ros-$ROS_DISTRO-usb-cam \
                     ros-$ROS_DISTRO-open-manipulator \
                     ros-$ROS_DISTRO-dynamixel-workbench \
                     ros-$ROS_DISTRO-dynamixel-sdk \
                     ros-$ROS_DISTRO-realsense-camera \
                     ros-$ROS_DISTRO-joy

sudo apt-get install linux-headers-generic
sudo apt-get install build-essential g++
sudo apt-get install libasound2-dev 
sudo apt-get install mplayer

#编译依赖
rosdep where-defined bullet
sudo apt-get install libbullet-dev

rosdep where-defined sdl
sudo apt-get install libsdl1.2-dev

rosdep where-defined sdl-image
sudo apt-get install libsdl-image1.2-dev

```

#安装功能包
```bash
#创建文件夹
mkdir -p ~/High-genius_ws/src && cd ~/High-genius_ws/src
git clone https://github.com/GJXS1980/TurtleBot-Tutorial.git
sudo cp ~/High-genius_ws/src/TurtleBot-Tutorial/robot_voice/libs/x64/libmsc.so /usr/lib/libmsc.so

#编译
cd ~/High-genius_ws
catkin_make

#配置环境
echo "export High_genius_ws='$(pwd)'" >> ~/.bashrc
echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc
bash

sudo gedit ~/.mplayer/config
#添加下面代码
lirc=no
```

##### kobuki底盘控制
###### 键盘控制
```bash
roslaunch kobuki_node minimal.launch --screen

roslaunch kobuki_keyop keyop.launch --screen
```

###### 手柄控制
罗技手柄连接:
驱动安装:
```bash
sudo apt-get install ros-$ROS_DISTRO-joystick-drivers
rosdep install joy
rosmake joy

#更改js0的root群组为dialout群组
sudo chgrp dialout /dev/input/js0

#测试(按下手柄按键，终端上会显示按键信息)
sudo jstest /dev/input/js0

```
安装教程:[罗技F710无线手柄在ROS下的安装使用](https://blog.csdn.net/hengheng_51/article/details/80246918)

启动手柄控制:
```bash
roslaunch turtlebot_bringup minimal.launch
roslaunch turtlebot_teleop logitech.launch

```
按住LB键激活然后左摇杆控制机器人移动

#### turtlebot2仿真模型的加载
1.turtlebot2仿真模型的加载 
```bash
roslaunch turtlebot_gazebo turtlebot_world.launch

roslaunch turtlebot_teleop logitech.launch
```

2.实体机器人建图:
```bash
roslaunch turtlebot_bringup minimal.launch
roslaunch kinect2_slam kinect2_gmapping.launch
roslaunch kinect2_slam kinect2_gmapping_rviz_view.launch
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

## 激光雷达
##### 设置串口访问权限
创建turtlebot的串口别名
新建/etc/udev/rules.d/57-kobuki.rules
```bash
cd /etc/udev/rules.d
sudo touch 57-kobuki.rules && sudo gedit 57-kobuki.rules
```

内容如下:
```bash
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
```bash
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="rplidar"  
```

增加对串口的默认访问权限
```bash
sudo usermod -a -G dialout 用户名
```
用户名为终端@之前的名字

配置生效
使udev配置生效
```bash
sudo service udev reload
sudo service udev restart
```

检查rplidar的串口的权限：
```bash
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
修改文件（已经修改好）
（1）修改turtlebot_apps/turtlebot_navigation/launch/gmapping_demo.launch  <br>
（2）修改turtlebot_apps/turtlebot_navigation/launch/includes/gmapping.launch.xml

运行launch,创建地图
```bash
roslaunch turtlebot_navigation gmapping_demo.launch

#启动rviz
roslaunch turtlebot_rviz_launchers view_navigation.launch

#手柄控制
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

安装Rplidar的方法： 把Rplidar固定在turtlebot上方的最前面的位置，Rplidar三角形电机的朝向和turtlebot 前方碰撞挡板的中心位置一致，测量Rplidar和turtlebot底座的高度为17.04cm。

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

##### hector_mapping
运行 
```bash
roslaunch turtlebot_bringup minimal.launch    
roslaunch turtlebot_teleop logitech.launch
roslaunch rplidar_ros hector_mapping_demo.launch  
roslaunch rplidar_ros rplidar.launch  

```

### 人脸识别
##### kinect2人脸识别
方法一:基于Haar特征的级联分类器对象检测算法
```bash
roslaunch kinect2_bridge kinect2_bridge.launch

roslaunch vision_detector face_detector_kinect2.launch 

rqt_image_view 
```

##### kinect2物体跟踪
```bash
roslaunch kinect2_bridge kinect2_bridge.launch

roslaunch vision_detector motion_detector_kinect2.launch 

rqt_image_view 

```

### tensorflow in ROS
#安装tensorflow
1.安装Bazel
```bash
sudo apt-get install openjdk-8-jdk
echo "deb [arch=amd64] http://storage.googleapis.com/bazel-apt stable jdk1.8" | sudo tee /etc/apt/sources.list.d/bazel.list

curl https://bazel.build/bazel-release.pub.gpg | sudo apt-key add -
sudo apt-get update && sudo apt-get install bazel
sudo apt-get install --only-upgrade bazel


```
参考网站：[Installing Bazel on Ubuntu](https://docs.bazel.build/versions/master/install-ubuntu.html#install-with-installer-ubuntu)

2.安装CUDA和Cudnn
```bash
#查看CUDA版本
cat /usr/local/cuda/version.txt

#查看cudnn 版本 
cat /usr/local/cuda/include/cudnn.h | grep CUDNN_MAJOR -A 2
```

下载 cuDNN v6安装[官网](https://developer.nvidia.com/cudnn)
```bash
tar xvzf cudnn-8.0-linux-x64-v5.1-ga.tgz 
sudo cp -P cuda/include/cudnn.h /usr/local/cuda/include 
sudo cp -P cuda/lib64/libcudnn* /usr/local/cuda/lib64 
sudo chmod a+r /usr/local/cuda/include/cudnn.h /usr/local/cuda/lib64/libcudnn*
```

添加路径到.bashrc
```bash
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/local/cuda/lib64:/usr/local/cuda/extras/CUPTI/lib64" 
export CUDA_HOME=/usr/local/cuda
```

3.安装tensorflow
python 2.7版本:
```bash
sudo apt-get install python-pip python-dev
sudo pip install --upgrade pip
sudo pip install tensorflow
sudo pip install tensorflow-gpu
sudo pip install tensorflow-gpu==1.4

```

python 3.x版本
```bash
sudo apt-get install python3-pip python3-dev
sudo pip3 install --upgrade pip

sudo pip3 install tensorflow

sudo pip3 install tensorflow-gpu

sudo pip3 install tensorflow-gpu==1.4
```

>如果显示下面错误：Cannot uninstall 'enum34'. It is a distutils installed project and thus we cannot accurately determine which files belong to it which would lead to only a partial uninstall. <br>
>输入以下命令即可：
sudo pip install --ignore-installed enum34
然后继续安装TensorFlow就可以啦！

4.测试
```python
#在终端输入下面命令进入python
python

#在python环境下输入测试命令行
import tensorflow as tf
hello = tf.constant('Hello, TensorFlow!')
mysess = tf.Session()
print(mysess.run(hello))
```

5.安装cv-bridge和camera driver
```bash
sudo apt-get install ros-$ROS_DISTRO-cv-bridge ros-kinetic-cv-camera
```

6.运行vision_tensorflow
安装依赖：
```bash
sudo apt-get install protobuf-compiler python-pil python-lxml
sudo pip install jupyter
sudo pip install matplotlib
```

##### 摄像头识别
```bash
#启动摄像头
roslaunch usb_cam usb_cam-test.launch
```

识别数字：
```bash
roslaunch vision_tensorflow vision_tensorflow_camera_mnist.launch
rostopic echo /result_ripe
```

物体识别：
```bash
roslaunch vision_tensorflow vision_tensorflow_camera_classify.launch

rostopic echo /result_ripe
```

物体跟踪：
```bash
roslaunch vision_tensorflow vision_tensorflow_camera_detect.launch

rosrun image_view image_view image:=/result_ripe
```

##### kinect2识别
```bash
#启动摄像头
roscore
roslaunch kinect2_bridge kinect2_bridge.launch
```

识别数字：
```bash
roslaunch vision_tensorflow vision_tensorflow_kinect2_mnist.launch
rostopic echo /result_ripe
```

物体识别：
```bash
roslaunch vision_tensorflow vision_tensorflow_kinect2_classify.launch

rostopic echo /result_ripe
```

物体跟踪：
```bash
roslaunch vision_tensorflow vision_tensorflow_kinect2_detect.launch

rqt_image_view
```

### 语音控制
###### ROS与语音交互-科大讯飞语音SDK的ROS包使用（xf-ros ）
语音听写
```bash
roscore 
rosrun robot_voice iat_publish
rostopic pub -r 0.1 /voiceWakeup std_msgs/String "你好,小谷机器人"
```

语音合成
```bash
roscore
rosrun robot_voice tts_subscribe
rostopic pub /voiceWords std_msgs/String "你好,小谷机器人"
```

智能语音助手
```bash
roscore 
rosrun robot_voice iat_publish
rosrun robot_voice voice_assistant
rostopic pub /voiceWakeup std_msgs/String "你好,小谷机器人'"

rostopic pub -r 0.1 /voiceWakeup std_msgs/String "你好,小谷机器人'"

```















