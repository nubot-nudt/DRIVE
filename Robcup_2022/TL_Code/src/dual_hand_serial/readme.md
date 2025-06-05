### 1、用虚拟环境的python 进行编译 ###
```
catkin_make -DPYTHON_EXECUTABLE=/home/nubot-11/anaconda3/envs/dexmv/bin/py
```

第一次使用后就已经换成上述解释器，当删掉build、devel后，解释器会变成最初的。

### 2、给串口赋权

```
sudo chmod 777 /dev/ttyUSB1

sudo chmod 777 /dev/ttyUSB0
```

### 3、运行代码（进入hand目录下，记得source）

```
roslaunch dual_hand_serial dual_hand.launch
```
```
roslaunch dual_hand_serial footkey.launch
```

