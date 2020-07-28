# adaptive_data_sharing

安装编译：
一. 安装编译hector_quadrotor_tutorial
data_sharing包依赖于hector_quadrotor_tutorial包，所以先安装它。目前最新版本是Kinetic版。
安装教程：http://wiki.ros.org/hector_quadrotor/Tutorials/Quadrotor%20outdoor%20flight%20demo ，使用Install from source安装编译。
安装命令：
mkdir ~/hector_quadrotor_tutorial
cd ~/hector_quadrotor_tutorial
wstool init src https://raw.github.com/tu-darmstadt-ros-pkg/hector_quadrotor/kinetic-devel/tutorials.rosinstall
编译命令：
catkin_make
source devel/setup.bash
启动命令：
roslaunch hector_quadrotor_demo outdoor_flight_gazebo.launch
rosservice call /enable_motors "enable: true"

二. 编译data_sharing包
可以将data_sharing包放在已有的catkin空间中进行编译。如果编译不成功，可将其放入hector_quadrotor_tutorial中的src文件夹下，再进行编译。



用法：
一. PER变化场景
在无人机群体运动时进行算法测试，在各个终端source后，再依次启动以下几个文件：
1. World文件：设置了在Gazebo中的运动背景，及无人机的初始位置。现在有4个运动场景，如world_XXX.launch。
2. Service文件：ROS Kinetic以后的hector_quadrotor_tutorial包，启动时还需要运行rosservice call /enable_motors "enable: true"。
   所以启动World文件后，还需要进入data_sharing/run_sh目录下，再运行命令sh run_service.sh。
3. PER计算文件：packet_error_dynamic.launch，根据无人机位置计算并发布PER信息。
4. UAV控制文件：控制无人机运动，和world文件相配套。现在有4个控制文件，如ctrl_XXX.launch。
5. 算法文件：lazy，eager和relay算法，如XXX_asyn.launch。
   自适应算法因为目前是集中式中继选择，需要在新的终端下先启动relay_control.launch，再启动relay_asyn.launch。

比如启动：
1. roslaunch data_sharing world_small_circle.launch
2. roslaunch data_sharing packet_error_dynamic.launch
3. roslaunch data_sharing ctrl_small_circle.launch
4. roslaunch data_sharing lazy_asyn.launch 或者 roslaunch data_sharing eager_asyn.launch 或者
   roslaunch data_sharing relay_control.launch + roslaunch data_sharing relay_asyn.launch

二. PER固定场景
只要启动PER计算文件+算法文件：
1. roslaunch data_sharing packet_error_fixed.launch（在launch文件中设置PER）
2. roslaunch data_sharing lazy_asyn.launch



计算
calculate_result.launch用来计算多次运行算法后，平均每个无人机的收敛轮数和总负载数。


注意
1. 根据需要修改算法、PER计算和calculate_result的launch文件和service文件中的无人机数目，消息传输频率，数据存储路径等参数。
2. 数据文件con_uav_0.txt存储0号无人机在算法达到收敛状态所需要的收敛轮数，文件pb_uav_0.txt存储0号无人机在算法达到收敛的过程中所传输的负载数。
3. 在测试过程中，Gazebo可能挂掉，部分无人机无法运动,从而导致此次测试失败，此时需要在数据文件中删除对应的con数据和pb数据，再重新启动程序。
