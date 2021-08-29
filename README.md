## Slice-to-volume Registration
#### 启动机器人
```
roslaunch ultrasound_robot ur5_real.launch
```

#### 启动python代码，读取并发布图像
```
cd /home/kuka/lee_ws/src/ultrasound_robot/src/python
conda activate base && python us_image_acquisition.py
```
#### 启动Bayesian SVR ROS Service
```
cd /home/kuka/SVR/Slice_to_Volume_Registration_Python
conda activate base && python BayesianSVR.py
```
#### 配准，导航
```
rosrun ultrasound_robot thyroid_biopsy
rosrun rqt_reconfigure rqt_reconfigure
```



## 超声3维重建图像采集

同时启动下面三个程序
#### 启动机器人以及ur_free_drive.py(输入1，可自由拖动机器人10s)
```
roslaunch ultrasound_robot ur5_free_drive.launch
```

#### 启动机器人扫描路径与控制程序（输入e记录一个waypoint，共6个）
```
rosrun ultrasound_robot us_image_acquisition
```

#### 启动python代码（接收图像和tf，生成.mha）
```
cd /home/kuka/lee_ws/src/ultrasound_robot/src/python
source activate py36 && python us_image_acquisition.py 2
```





## 查看超声图像起始点、高度和宽度
```
cd /home/kuka/lee_ws/src/ultrasound_robot/src/python
source activate py36 && python us_image_acquisition.py 1
```




## Force sensor calibration
```
roslaunch ultrasound_robot forece_sensor_calibration.launch
roslaunch force_torque_sensor_calib example_ft_calib.launch
```




## Visual Servo: Aruco Marker Tracking
```
roslaunch ultrasound_robot ur5_real.launch
roslaunch ultrasound_robot aruco_single_marker.launch
rosrun ultrasound_robot visual_servo_test
```