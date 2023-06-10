# Mobile-robot Collision Avoidance Learning

# Overview

---

- To navigation on the complex environment, the Deep Reinforcement Learning can be attractive method better than classical methods. `rl_local_planner` inherits MRCA(Multi-robot collision avidance) methodology, which is based on  PPO(Proximal Policy Optimization and Stage simulation. In our case, we adopt SAC as state-of-art Model-Free algorithm and trained on the physical simulation to be considered with real world properties. The training time are decreased by replacing with off-policy and entropy exploration. The off-policy could be more sample efficient because of multi agents environment.
- The `rl_local_planner` is already tested on real logistic robot and ported the ROS but not local planner planner. For practical usages, `rl_local_planner` included the global_planner and selected the sub goal concept as look ahead, called MCAL(Mobile robot Collision Avoidance Learning)  similar with Hybrid-MRCA.


[![Video Label](http://img.youtube.com/vi/ACG4TMenJDs/0.jpg)](https://www.youtube.com/watch?v=ACG4TMenJDs) 

# Requirement

---

- python 2.7
- ROS Kinetic & Melodic
- mpi4py
- Stage
- Gazebo
- PyTorch

# Mobile-robot Collision Avoidance Learning

---

# Stage world

---

![Mobile-robot%20Collision%20Avoidance%20Learning%20%E1%84%8C%E1%85%A5%E1%86%BC%E1%84%85%E1%85%B5%20f9e8c25cea84487abc62b4ff4d7b2d14/Untitled.png](./assets/stage.png)

## How to Train

---

Please use the stage_ros-add_pose_and_crash package instead of the default package provided by ROS.

```jsx
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/CzJaewan/rl_local_planner.git
cd ..
catkin_make
```

To train, modify the hyper-parameters in sac_main.py as you like, and running the following command:

```jsx
cd catkin_ws/src/rl_local_planner/stage_ro-add_pose_and_crach/
rosrun stage_ros_add_pose_and_crash stageros_w worlds/stage1.world
mpiexec -np 6 python sac_main.py
```

## How to Test

---

```jsx
rosrun stage_ros_add_pose_and_crash stageros worlds/circle.world
mpiexec -np 50 python circle_test.py
```

# Gazebo world

---

![Mobile-robot%20Collision%20Avoidance%20Learning%20%E1%84%8C%E1%85%A5%E1%86%BC%E1%84%85%E1%85%B5%20f9e8c25cea84487abc62b4ff4d7b2d14/Untitled%201.png](./assets/gazebo.png)

## How to Train

---

Please use the stage_ros-add_pose_and_crash package instead of the default package provided by ROS.

```jsx
mkdir -p catkin_ws/src
cp stage_ros-add_pose_and_crash catkin_ws/src
cd catkin_ws
catkin_make
source devel/setup.bash
```

To train, modify the hyper-parameters in sac_main.py as you like, and running the following command:

```jsx
rosrun stage_ros_add_pose_and_crash stageros worlds/stage1.world
mpiexec -np 6 python sac_main.py
```

## How to Test

---

### 1. Run Gazebo simulator  
导入机器人urdf文件，仿真两个激光雷达和一个双轮差速里程计，加载进入 gazebo 仿真环境
使用 ira_laser_tools 工具包将双激光雷达融合为一个360°的激光雷达。
```jsx
#empty world
roslaunch sp_gazebo scan_merged_sr7e_rl_gazebo.launch

#obstacle world
roslaunch sp_gazebo scan_merged_sr7e_rl_static_2_gazebo.launch
roslaunch sp_gazebo scan_merged_sr7e_rl_static_4_gazebo.launch
roslaunch sp_gazebo scan_merged_sr7e_rl_d35m_gazebo.launch
roslaunch sp_gazebo scan_merged_sr7e_rl_dc_gazebo.launch
roslaunch sp_gazebo scan_merged_sr7e_rl_df_gazebo.launch
# for haina
roslaunch sp_gazebo scan_merged_haina_rl_d35m_gazebo.launch

```
### 2. Run RL navigation for haina  
使用 dwa_local_planner
```bash  
roslaunch gazebo_rl_test haina_dwa.launch 
```
使用 base_local_planner
```bash  
roslaunch gazebo_rl_test haina_base_local.launch 
```
### 2. Run RL navigation

### without Look ahead (mcal)
调用 map_server 加载地图,  调用 amcl 进行定位， 调用 move_base 进行导航,  调用 rviz 可视化

**map_server 配置文件:** lsq_map_2.yaml

**amcl 配置文件:** 3amcl_params.yaml

**move_base 配置文件:** 
carlike/2costmap_common_params_local.yaml  
carlike/2costmap_common_params.yaml  
carlike/2local_costmap_params.yaml  
carlike/2global_costmap_params.yaml  

Global Planner: global_planner/GlobalPlanner,  global_planner_params.yaml  
Local Planner: base_local_planner/TrajectoryPlannerROS  
external controller: /fake_cmd  

1. run roslaunch file & runfile  
    ***for sr7e***  
    ```jsx
    roslaunch gazebo_rl_test sysconbot_rl.launch
    cd ~/catkin_ws/src/rl_local_planner/mcal_gazebo/GAZEBO_TEST_SAC/single
    mpiexec -np 1 python gazebo_test_sac.py
    ```
    ***for haina***  
    ```jsx
    roslaunch gazebo_rl_test syshaina_rl.launch
    cd ~/catkin_ws/src/rl_local_planner/mcal_gazebo/GAZEBO_TEST_SAC/single
    mpiexec -np 1 python gazebo_haina_test_sac.py
    ```

### with Look ahead (mcal_p)  

调用 map_server 加载地图,  调用 amcl 进行定位， 调用 move_base 进行导航,调用 Look_a_head_Node, 调用 rviz 可视化

**map_server 配置文件:** lsq_map_2.yaml

**amcl 配置文件:** 3amcl_params.yaml

**move_base 配置文件:** 
sr7c_ls_param/costmap_common_params.yaml  
sr7c_ls_param/costmap_common_params.yaml  
sr7c_ls_param/local_costmap_params.yaml  
sr7c_ls_param/global_costmap_params.yaml  

Global Planner: global_planner/GlobalPlanner,  global_planner_params.yaml  
Local Planner: base_local_planner/TrajectoryPlannerROS  
external controller: /fake_cmd  

**look a head 配置文件:** lookahead.yaml  

1. run roslaunch file & runfile  
    ***for sr7e***  
    ```jsx
    roslaunch gazebo_rl_test sysconbot_rl_lookahead_2.launch
    cd ~/catkin_ws/src/rl_local_planner/mcal_gazebo/GAZEBO_TEST_SAC/single
    mpiexec -np 1 python gazebo_test_sac1.py
    ```  
    ***for haina***  
    ```jsx
    roslaunch gazebo_rl_test syshaina_rl_lookahead.launch
    cd ~/catkin_ws/src/rl_local_planner/mcal_gazebo/GAZEBO_TEST_SAC/single
    mpiexec -np 1 python gazebo_haina_test_sac1.py
    ```

### with Look ahead & hybrid mode (hybrid mcal_p)  
**map_server 配置文件:** lsq_map_2.yaml

**amcl 配置文件:** 3amcl_params.yaml

**move_base 配置文件:** 
sr7c_ls_param/dwa_costmap_common_params.yaml  
sr7c_ls_param/dwa_costmap_common_params.yaml  
sr7c_ls_param/dwa_local_costmap_params.yaml  
sr7c_ls_param/dwa_global_costmap_params.yaml  

Global Planner: global_planner_params.yaml  
Local Planner: sr7c_ls_param/dwa_local_planner_params.yaml  
<remap from="cmd_vel" to="dwa_cmd_vel" />

**look a head 配置文件:** lookahead.yaml  

***for sr7e***   
1. run launch file

    ```jsx
    roslaunch gazebo_rl_test hybrid_syscon_navigation.launch
    ```

2. run cmd_vel_swich node

    ```jsx
    cd ~/catkin_ws/src/rl_local_planner/navigation/script
    python hybrid_cmd_vel_swich.py
    ```

3. run runfile 

    ```jsx
    cd ~/catkin_ws/src/rl_local_planner/mcal_gazebo/GAZEBO_TEST_SAC/single
    mpiexec -np 1 python gazebo_test_sac2.py
    ```  
    
***for haina***  
1. run launch file

    ```jsx
    roslaunch gazebo_rl_test hybrid_syshaina_navigation.launch
    ```

2. run cmd_vel_swich node

    ```jsx
    cd ~/catkin_ws/src/rl_local_planner/navigation/script
    python hybrid_cmd_vel_swich.py
    ```

3. run runfile 

    ```jsx
    cd ~/catkin_ws/src/rl_local_planner/mcal_gazebo/GAZEBO_TEST_SAC/single
    mpiexec -np 1 python gazebo_haina_test_sac2.py
    ```

## How to Test (multi-agent)

---

### 1. Run Gazebo simulator

```jsx
# 2 agent
roslaunch sp_gazebo scan_merged_sr7e_rl_2agent.launch

# 4 agent
roslaunch sp_gazebo scan_merged_sr7e_rl_4agent.launch
```

### 2. Run RL navigation

localization, global planner, costmap, map run

```jsx
# 2 agent
roslaunch gazebo_rl_test multi-sysconbot_rl_lookahead_2.launch

# 4 agent
roslaunch gazebo_rl_test multi-sysconbot_rl_lookahead_4.launchy
```

rl based collision avoidance and driving controller run

```jsx
# 2 agent
cd ~/catkin_ws/src/rl_local_planner/mcal_gazebo/GAZEBO_TEST_SAC/multi
mpiexec -np 2 python gazebo_1_multi_test_sac.py
or
mpiexec -np 1 python gazebo_1_multi_test_sac.py
mpiexec -np 1 python gazebo_2_multi_test_sac.py

# 4 agent
cd ~/catkin_ws/src/rl_local_planner/mcal_gazebo/GAZEBO_TEST_SAC/multi
mpiexec -np 4 python gazebo_1_multi_test_sac.py
or
mpiexec -np 1 python gazebo_1_multi_test_sac.py
mpiexec -np 1 python gazebo_2_multi_test_sac.py
mpiexec -np 1 python gazebo_3_multi_test_sac.py
mpiexec -np 1 python gazebo_4_multi_test_sac.py
```

### Real world test

### References:

[A study on dynamic object avoidance driving based on Reinforcement learning using mobile robot](http://snut.dcollection.net/srch/srchDetail/200000372623?ajax=false&start=0&query=%28ins_code%3A211034%29+AND++%2B%28%28all%3A%EC%B5%9C%EC%9E%AC%EC%99%84%29%29&sortDir=desc&pageSize=10&searchKeyWord1=%EC%B5%9C%EC%9E%AC%EC%99%84&searchWhere1=all&searchTotalCount=0&navigationSize=10&searchText=%5B%EC%A0%84%EC%B2%B4%3A%3Cspan+class%3D%22point1%22%3E%EC%B5%9C%EC%9E%AC%EC%99%84%3C%2Fspan%3E%5D&pageNum=1&rows=10&itemTypeCode=all&insCode=211034&searthTotalPage=0&sortField=score)
