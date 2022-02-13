# 说明

运行环境：
* ubuntu 16.04
* C++ 11

依赖以下ros package:
* navigation
* rbx1
* arbotix

为了方便调试以及代码复用，将RRT Connect算法和RRT\*算法写在了同一个ROS包中。若要改用不同的算法，只需要修改`src/my_planner/src/my_global_planner.cpp`里`MyPlannerROS::initialize`初始化函数中`whichAlgorithm`的值，若该变量为1,则使用RRT Connect算法，若该变量为2,则使用RRT*算法。


# 仿真

首先，将写有RRT、RRT Connect和RRT*算法的my_planner包放在`~/catkin_ws/src`中，然后将如下代码复制到`src/navigation/move_base/src/move_base.cpp`中

```cpp
try {
    int p;;
    cout<<"1:global_planner/GlobalPlanner"<<endl;;
    cout<<"2:dstar_global_planner/DStarPlannerROS"<<endl;;
    cout<<"3:GA_planner/GAPlanner"<<endl;;
    cout<<"4:RAstar_planner/RAstarPlannerROS"<<endl;;
    cout<<"5:rrt_plan/rrt_planner"<<endl;
    cout<<"6:my_global_planner/MyPlannerROS"<<endl;;
    cin>>p;;
    switch(p){
    case 1 :global_planner="global_planner/GlobalPlanner";;
    break;
    case 2 :global_planner="dstar_global_planner/DStarPlannerROS";;
    break;
    case 3 :global_planner="GA_planner/GAPlanner";;
    break;
    case 4 :global_planner="RAstar_planner/RAstarPlannerROS";;
    break;
    case 5 :global_planner="rrt_plan/rrt_planner";
    break;
    case 6 :global_planner="my_global_planner/MyPlannerROS";;
    break;
    default:cout<<"error input"<<endl;;
}
cout<<"global_planner="<<global_planner<<endl;;
```

然后将`my_planner`中的`fake_move_base_amcl.launch`放到`src/rbx1/rbx1_nav/launch`中，覆盖原文件，之后执行以下命令：

```sh
cd ~/catkin_ws
catkin_make
roslaunch rbx1_bringup fake_turtlebot.launch
roslaunch rbx1_nav fake_amcl.launch map:=test_map.yaml
rosrun rviz rviz -d `rospack find rbx1_nav`/nav.rviz
```


# 真机
在前面基础上，执行以下命令即可
```sh
roslaunch turtlebot_bringup minimal.launch

roslaunch kinect2_bridge kinect2_laser.launch

roslaunch turtlebot_navigation kinect2_amcl.launch map_file:=$HOME/kinect2map.yaml # 可以换成自己的地图

roslaunch turtlebot_rviz_launchers view_navigation.launch
```

# 效果

## RRT Connect

![](md_files/RRT_Connect.png)

## RRT*

![](md_files/RRT_Star.png)


