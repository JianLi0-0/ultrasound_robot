## 超声3维重建图像采集

同时启动下面三个程序
### 启动机器人以及ur_free_drive.py(输入1，可自由拖动机器人10s)
```
roslaunch ultrasound_robot ur5_free_drive.launch
```

### 启动机器人扫描路径与控制程序（输入e记录一个waypoint，共6个）
```
rosrun ultrasound_robot us_image_acquisition
```

### 启动python代码（接收图像和tf，生成.mha）
```
cd /home/sunlab/Desktop/SVR/Slice_to_Volume_Registration_Python
source activate py36 && python us_image_acquisition.py
```


## Calibration