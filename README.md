# uav_land

Pacote usadno para pouso autonomo de drone. A ideia é montar um código para utiulizar tanto com o tello_driver quanto com o mavros para pouso de um drone em uma base em movimento utilizando o magni.


# files

## shell_scripts

1. iris_sim: executa o gazebo com um magni e um iris uav e os pacotes necessários para o mng

2. tello_mng: executa os pacotes necessários para o land e para a comunicação com o tello

3. mavros_mng: executa os pacotes necessários para o land e para a comunicação com a pixhawlk 4

# tello

## install
```
cd catkin_ws/src
git clone https://github.com/lucianobonzatto/tello_control.git
cd ..
catkin_make
```

## dependencies

### tello_driver

foi feito baseado no seguinte pacote:

https://github.com/appie-17/tello_driver

```
sudo apt install ros-noetic-codec-image-transport

sudo apt-get install python3-wheel

cd tello_driver/src/tellopy/tellopy

python3 setup.py bdist_wheel

sudo apt-get install python3-pip

pip3 install dist/tellopy-*.dev*.whl --upgrade
```

### camera_info_manager_py
dependencie of tello_driver
https://github.com/ros-perception/camera_info_manager_py



## topics

### Subscribed topics
* ```/tello/cmd_vel``` [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
* ```/tello/emergency``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* ```/tello/fast_mode``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* ```/tello/flattrim``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* ```/tello/flip``` [std_msgs/Uint8](http://docs.ros.org/api/std_msgs/html/msg/UInt8.html)
* ```/tello/land``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* ```/tello/palm_land``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* ```/tello/takeoff``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* ```/tello/manual_takeoff``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* ```/tello/throw_takeoff``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)

### Published topics
* ```/tello/image_raw/camera_info``` [sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)
* ```/tello/image_raw/h264``` [h264_image_transport/H264Packet](https://github.com/tilk/h264_image_transport/blob/master/msg/H264Packet.msg)
* ```/tello/odom``` [nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)
* ```/tello/imu``` [sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
* ```/tello/status``` [tello_driver/TelloStatus](https://github.com/appie-17/tello_driver/blob/development/msg/TelloStatus.msg)
