# Ros2 Android Camera Stream
A Ros2 android app based on [ros2_java](https://github.com/ros2-java/ros2_java). Stream android camera images to ros2 topic.

基於[ros2_java](https://github.com/ros2-java/ros2_java)的Android App並串流相機影像到Ros2節點上

## Screenshot
![screenshot](https://github.com/winter3030/Ros2_Android_Camera_Stream/blob/master/screenshot/screenshot_gif.gif)

## Requirement
Device requires at least API 21 (Android 5.0).

## Usage
1.Download [apk](https://github.com/winter3030/Ros2_Android_Camera_Stream/releases)

2.Create a workspace and ros2 package

3.Write a [subscriber node](https://github.com/winter3030/Ros2_Android_Camera_Stream/tree/master/python%20subscriber%20node)

4.Connect mobile and desktop/laptop to the same network

5.Run

```
#Image
#QoS Default
ros2 run ros2_android_listener android_listener
or
#QoS Sensor data
ros2 run ros2_android_listener android_listener --qos sensor
```
```
#Text
#QoS Default
ros2 run ros2_android_listener android_listener --msg text
or
#QoS Sensor data
ros2 run ros2_android_listener android_listener --qos sensor --msg text
```

## Configuration
### Camera resolution
* VGA 640x480
* HD 1280x720
* FullHD 1920x1080
### Image compression
* JPG
* PNG
### ROS2 QoS
* Default
* Services
* Parameters
* Sensor data
