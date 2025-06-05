# DRIVE: A Unified Framework for Empowering Personalized Autonomous Driving with Humanoid Robots

### Project Page | [Video]()

This repo contains the implementation of our paper: 
> **DRIVE: A Unified Framework for Empowering Personalized Autonomous Driving with Humanoid Robots**
> 
> Junkai Ren, Jiawei Luo*, Chuang Cheng, Yansong Feng, Tuochang Wu, Huimin Lu, Zongtan Zhou*, Xin Xu
>

### Hyperparameters and Values

| Hyperparameter | Values |
| ----------- | ----------- |
| batch size | 2048 |
| minibatch size | 64 |
| learning rate | 0.0001 |
| GAE parameter $\lambda$ | 0.95 |
| discount factor $\gamma$ | 0.99 |
| clipping factor $\epsilon$ | 0.2 |
| number of optimization objectives $M$ | 2 (lateral error and target speed) |
| number of reward models $K$ | 3 |
| reward model initial weight $\beta_0$ | 0.05 |
| weight decay rate $\rho$ | 0.001 |
| smoothing factor $\alpha$ | 0.9 |
| interial coefficients $M_0$ | $[1,1,1,1,1,1]$ |
| damping coefficients $D_0$ | $[0.1,0.1,0.1,5.5,5.5,5.5]$ |
| stiffness coeefficients $K_0$ | $[500,500,500,500,500,500]$ |
| control period $\Delta t$ | 0.01 |
| PPO decision period | $1/15$ |
| steering radius $R$ | 0.23 |

### Install

Download

```
git clone https://github.com/nubot-nudt/DRIVE
```

1、ROS-Noetic

```
wget http://fishros.com/install -O fishros && . fishros
```

2、Third-party library

```
sudo ln -s /usr/bin/python3 /usr/bin/python
sudo apt-get install ros-noetic-tf2-sensor-msgs ros-noetic-serial ros-noetic-derived-object-msgs ros-noetic-ackermann-msgs
sudo apt install xsltproc
```

3、Conda Virtual Environment

```
conda create -n carla_py35 python==3.5.5
pip install --upgrade pip
pip install -r requirements.txt
```

4、Coppeliasim V4.1.0

```

```



