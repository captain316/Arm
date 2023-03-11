arm
# Arm
1. 打开仿真环境
>请确保已完成设备链接：网线？ip地址？

```xml
	roslaunch ur3_move realUR3Cartesian.launch
```
仿真环境中有一些预定义的姿态控制，如果需要同时控制实体，请打开机械臂操作面板，点击下方中间的`暂停`图标，如果关闭了仿真环境再重新打开仿真环境控制实体，请记得操作面板上也要同步`关闭`，再`打开`。

2. 打开摄像头
```xml
	roslaunch openni2_launch openni2.launch
```
请确保摄像头设备已接入，当看到`Device "1d27/6601@1/8" found.`时，打开另一个终端输入`rviz`，在rviz界面中左下角依次点击：
```xml
add -> By topic -> /rgb -> /image_raw -> Image
```
