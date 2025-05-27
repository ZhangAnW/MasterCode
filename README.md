# 代码文档
# 环境配置

Ubuntu 20.04 LTS
ROS noetic

首先，安装 **nlopt v2.7.1**：

```jsx
git clone -b v2.7.1 https://github.com/stevengj/nlopt.git
cd nlopt
mkdir build
cd build
cmake ..
make
sudo make install
```

其次，安装其他所需工具

```jsx
sudo apt-get install libarmadillo-dev
```

最后，编译项目

```jsx
cd /home/zawubuntu/code/myCode   //进入对应项目工程文件夹
catkin_make -DCMAKE_BUILD_TYPE = Release //编译项目
```

> 其他版本依赖库不一致，可能无法匹配。
ROS 与 Ubuntu 版本是对应的
可以通过鱼香ros教程一键安装ros机器人操作系统。[https://fishros.org.cn/forum/topic/20/小鱼的一键安装系列](https://fishros.org.cn/forum/topic/20/%E5%B0%8F%E9%B1%BC%E7%9A%84%E4%B8%80%E9%94%AE%E5%AE%89%E8%A3%85%E7%B3%BB%E5%88%97)
> 

# 代码启动流程

```jsx
cd /home/zawubuntu/code/myCode   //进入对应项目工程文件夹
catkin_make -DCMAKE_BUILD_TYPE = Release //编译项目

source devel/setup.bash //刷新配置文件
roslaunch plan_manage rviz.launch  //启动rviz
*roslaunch plan_manage kino_replan.launch   //启动程序*
roslaunch plan_manage kino_replan.launch | tee kino_replan_tee_log.txt //记录日志的情况下启动程序
```

启动后可以在rviz界面点击设定终点，实现轨迹规划。

# 代码解读

## 关键轨迹规划模块流程

轨迹规划器在`src/planner/planner_manger`路径下，是规划器的核心代码，`src/planner`路径下的其他功能包都是为其服务的

```jsx
kinodynamicReplan
		//A* 轨迹，存储在了 KinodynamicAstar(kino_path_finder_).path_nodes_
	  int status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, true);
		//获得轨迹，维度3*n  固定时间间隔轨迹
		plan_data_.kino_path_ = kino_path_finder_->getKinoTraj(0.01);
		//获得固定时间间隔 ts 的 轨迹点和起点速度/加速度，终点速度/加速度
		kino_path_finder_->getSamples(ts, point_set, start_end_derivatives);
		//通过固定时间点集，起终点速度/加速度和ts，得到控制点ctrl_pts
		NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
		//通过ctrl_pts和阶次和时间间隔，创建实例init
		NonUniformBspline init(ctrl_pts, 3, ts);//control_points=2+point_set  控制点，3阶次，时间间隔ts
		//在bspline_optimizers_类实现了轨迹优化，ctrl_pts是引用，在这里启动后端优化，对ctrl point 优化
		bool flag_step_1_success = bspline_optimizers_[0]->BsplineOptimizeTrajRebound(ctrl_pts, ts);
		
		//优化结果在cps_.points和cps_.points中

		//通过ctrl_pts和阶次和时间间隔，创建实例pos
		NonUniformBspline pos = NonUniformBspline(ctrl_pts, 3, ts);
		//设置物理性能限制，进行可行性检测
		pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
	  bool feasible = pos.checkFeasibility(false);
		//迭代进行时间重分配
	  int iter_num = 0;
	  while (!feasible && ros::ok()) {
	    feasible = pos.reallocateTime();
	    if (++iter_num >= 3) break;
	  }
	  
//获得轨迹的信息，并push
```

## 其余部分代码流程

`traj_server`结点其实就是为了将规划器发布的轨迹`planner/bspline`转化为控制器指令`cmd`并上传到`position_cmd`话题上

```jsx
1->	创建结点  traj_server
2->	订阅规划轨迹 ：planner/bspline
	回调函数：bsplineCallback()
	2.1->	收到轨迹，创建两个变量pos_pts konts分别接收
				geometry_msgs/Point[]类型的pos_pts 和时间变量konts
	2.2->	用收到的pos_pts，konts创建新的均匀B样条曲线pos_traj
	2.3->	计算后面要用的变量：
		start_time_	//等于收到轨迹的成员变量:起始时间
		traj_id_	//等于收到轨迹的成员变量:id
		traj_		//插入pos_traj及其一阶导、二阶导
		traj_duration_	///总时间
	2.4->	receive_traj_=true
3->	创建发布者：pos_cmd_pub ，往 /position_cmd 话题上发布 PositionCommand 类型的数据
4->	创建定时器，间隔10ms进入回调函数一次	
	回调函数 cmdCallback()：
	4.1->	是否收到了轨迹receive_traj_=true，没有的话跳出函数
	4.2->	计算现在的时间和起始时间的间隔 t_cur = time_now-start_time_
	4.3->	判断 t_cur  在不在总时间区间内
		4.3.1->	t_cur < traj_duration_	
			计算当前t_cur的pos vel acc yaw(后面导航命令用)，再算一个pos_f(不知道干嘛用的)
		4.3.2->	t_cur > traj_duration_
			计算终点pos ，vel acc 设0，yaw不变； 
		4.3.3->	t_cur < 0
			报错
	4.4->	把pos vel acc yaw等信息装入cmd里，pos_cmd_pub 发布一次cmd到/position_cmd
5->	在cmd指令里设置控制器增益系数
6->	1.0sleep，ros::spin()
	traj_server.cpp 子函数calculate_yaw()
	->参数：double t_cur，vector3d &pos ，ros::Time time_now ，ros::Time time_last
	->功能：计算yaw角方向，变化率，并对输出进行限幅，把yaw输出限制在[-PI，PI]
	->输出：pair of <yaw，yaw_dot>
```

> 代码中包含了相关的具体注释，详情可阅读相关注释文件。
> 

# 其他

1. 如有问题，欢迎联系我获取更多细节。mail：zhanganwei@sjtu.edu.cn, anwei_zhang@outlook.com。
2. 本项目依托fast planner提供的框架，非轨迹规划细节部分可参考网上关于fast-planner的讲解。https://github.com/HKUST-Aerial-Robotics/Fast-Planner
