/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/




#include <plan_manage/kino_replan_fsm.h>

namespace fast_planner {

void KinoReplanFSM::init(ros::NodeHandle& nh) {
  current_wp_  = 0;
  exec_state_  = FSM_EXEC_STATE::INIT;
  have_target_ = false;
  have_odom_   = false;

  ROS_INFO("KinoReplanFSM: Initialize the fast planner.");
  /*  fsm param  */
  nh.param("fsm/flight_type", target_type_, -1);
  nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
  nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);

  nh.param("fsm/waypoint_num", waypoint_num_, -1);
  for (int i = 0; i < waypoint_num_; i++) {
    nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }

  /* initialize main modules */
  planner_manager_.reset(new FastPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);// 初始化规划模块
  visualization_.reset(new PlanningVisualization(nh));

  /* callback */
  exec_timer_   = nh.createTimer(ros::Duration(0.01), &KinoReplanFSM::execFSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &KinoReplanFSM::checkCollisionCallback, this);

  waypoint_sub_ =
      nh.subscribe("/waypoint_generator/waypoints", 1, &KinoReplanFSM::waypointCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &KinoReplanFSM::odometryCallback, this);

  replan_pub_  = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_     = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<plan_manage::Bspline>("/planning/bspline", 10);
}

void KinoReplanFSM::waypointCallback(const nav_msgs::PathConstPtr& msg) {
  if (msg->poses[0].pose.position.z < -0.1) return;

  std::cout << "waypointCallback:Triggered!" << std::endl;
  trigger_ = true;

  if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;

  } else if (target_type_ == TARGET_TYPE::PRESET_TARGET) {
    end_pt_(0)  = waypoints_[current_wp_][0];
    end_pt_(1)  = waypoints_[current_wp_][1];
    end_pt_(2)  = waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % waypoint_num_;
  }
  std::cout <<"end_pt_"<< end_pt_(0)<<" "<<end_pt_(1)<<" "<<end_pt_(2)<< std::endl;

  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(0, 0, 1, 1.0));//可视化  蓝色点  全局终点
  end_vel_.setZero();
  have_target_ = true;

  if (exec_state_ == WAIT_TARGET)
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  else if (exec_state_ == EXEC_TRAJ)
    changeFSMExecState(REPLAN_TRAJ, "TRIG");
}

void KinoReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  have_odom_ = true;
}

void KinoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call) {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
  int    pre_s        = int(exec_state_);
  exec_state_         = new_state;
  std::cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << std::endl;
}

void KinoReplanFSM::printFSMExecState() {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };

  std::cout << "[FSM]: state: " + state_str[int(exec_state_)] << std::endl;
}

void KinoReplanFSM::execFSMCallback(const ros::TimerEvent& e) {
  // 定义一个静态变量fsm_num，用于记录FSM执行的次数
  static int fsm_num = 0;
  fsm_num++;
  //定义double常量 replan_near_last_end_thresh_
  const double replan_near_last_end_thresh_ = 1;
  // 每执行100次，打印FSM执行状态
  if (fsm_num == 100) {
    printFSMExecState();
    // 如果没有odom，打印提示信息
    if (!have_odom_) std::cout << "no odom." << std::endl;
    // 如果没有触发，打印提示信息
    if (!trigger_) std::cout << "wait for goal." << std::endl;
    // 重置fsm_num
    fsm_num = 0;
  }

  // 根据exec_state_的值，执行不同的操作
  switch (exec_state_) {
    case INIT: {
      // 如果没有odom，直接返回
      if (!have_odom_) {
        return;
      }
      // 如果没有触发，直接返回
      if (!trigger_) {
        return;
      }
      // 将exec_state_设置为WAIT_TARGET，并打印提示信息
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET: {
      // 如果没有目标，直接返回
      if (!have_target_)
        return;
      // 否则，将exec_state_设置为GEN_NEW_TRAJ
      else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ: {
      // 设置起点、速度、加速度
      start_pt_  = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      // 计算起点yaw
      Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      start_yaw_(1) = start_yaw_(2) = 0.0;
      // 打印起点坐标
      std::cout<<"GEN_NEW_TRAJ start_pt_ "<< start_pt_(0)<<" "<<start_pt_(1)<<" "<<start_pt_(2)<< std::endl;
      std::cout<<"GEN_NEW_TRAJ end_pt_ "<< end_pt_(0)<<" "<<end_pt_(1)<<" "<<end_pt_(2)<< std::endl;

      // 调用callKinodynamicReplan()函数，如果成功，将exec_state_设置为EXEC_TRAJ，否则打印提示信息，并将exec_state_设置为GEN_NEW_TRAJ
      bool success = callKinodynamicReplan();
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        // have_target_ = false;
        // changeFSMExecState(WAIT_TARGET, "FSM");
        std::cout << "replan failed" << std::endl;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ: {
      /* determine if need to replan */

      // 获取当前轨迹信息
      LocalTrajData* info     = &planner_manager_->local_data_;
      ros::Time      time_now = ros::Time::now();
      double         t_cur    = (time_now - info->start_time_).toSec();
      t_cur                   = min(info->duration_, t_cur);

      // 计算当前位置
      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);
      // 获取当前轨迹的最后一个点作为上次规划的终点
      Eigen::Vector3d last_planned_end = info->position_traj_.evaluateDeBoorT(info->duration_);
      //输出当前位置和上次规划的终点
      std::cout<<"EXEC_TRAJ pos "<< pos(0)<<" "<<pos(1)<<" "<<pos(2)<< std::endl;
      std::cout<<"EXEC_TRAJ last_planned_end "<< last_planned_end(0)<<" "<<last_planned_end(1)<<" "<<last_planned_end(2)<< std::endl;
      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2) {//持续时间大于规划时间
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
        return;

      } else if ((end_pt_ - pos).norm() < no_replan_thresh_) {//快到达终点
        // std::cout << "near end" << std::endl;
        return;

      } else if ((info->start_pos_ - pos).norm() < replan_thresh_) {//很接近起点
        // std::cout << "near start" << std::endl;
        return;

      } else {
        //定周期的进行轨迹重规划，这里的逻辑可以改一下，根据当前四旋翼的位置，距离终点的距离，距离起点的距离，来决定是否需要重规划
        // changeFSMExecState(REPLAN_TRAJ, "FSM");
        if ((last_planned_end - pos).norm() < replan_near_last_end_thresh_) { // 快到达上次规划的终点时进行重规划
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
        // have_target_ = false;
        // changeFSMExecState(WAIT_TARGET, "FSM");
        // return;
      }
      break;
    }

    case REPLAN_TRAJ: {
      LocalTrajData* info     = &planner_manager_->local_data_;
      ros::Time      time_now = ros::Time::now();
      double         t_cur    = (time_now - info->start_time_).toSec();

      start_pt_  = info->position_traj_.evaluateDeBoorT(t_cur);
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

      start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_cur)[0];

      std_msgs::Empty replan_msg;
      replan_pub_.publish(replan_msg);

      bool success = callKinodynamicReplan();
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }
  }
}

void KinoReplanFSM::checkCollisionCallback(const ros::TimerEvent& e) {
  LocalTrajData* info = &planner_manager_->local_data_;

  if (have_target_) {
    auto edt_env = planner_manager_->edt_environment_;

    double dist = planner_manager_->pp_.dynamic_ ?
        edt_env->evaluateCoarseEDT(end_pt_, /* time to program start + */ info->duration_) :
        edt_env->evaluateCoarseEDT(end_pt_, -1.0);

    if (dist <= 0.3) {
      /* try to find a max distance goal around */
      bool            new_goal = false;
      const double    dr = 0.5, dtheta = 30, dz = 0.3;
      double          new_x, new_y, new_z, max_dist = -1.0;
      Eigen::Vector3d goal;

      for (double r = dr; r <= 5 * dr + 1e-3; r += dr) {
        for (double theta = -90; theta <= 270; theta += dtheta) {
          for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz) {

            new_x = end_pt_(0) + r * cos(theta / 57.3);
            new_y = end_pt_(1) + r * sin(theta / 57.3);
            new_z = end_pt_(2) + nz;

            Eigen::Vector3d new_pt(new_x, new_y, new_z);
            dist = planner_manager_->pp_.dynamic_ ?
                edt_env->evaluateCoarseEDT(new_pt, /* time to program start+ */ info->duration_) :
                edt_env->evaluateCoarseEDT(new_pt, -1.0);

            if (dist > max_dist) {
              /* reset end_pt_ */
              goal(0)  = new_x;
              goal(1)  = new_y;
              goal(2)  = new_z;
              max_dist = dist;
            }
          }
        }
      }

      if (max_dist > 0.3) {
        std::cout << "change goal, replan." << std::endl;
        end_pt_      = goal;
        have_target_ = true;
        end_vel_.setZero();

        if (exec_state_ == EXEC_TRAJ) {
          std::cout << "max_dist > 0.3" << std::endl;

          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }

        visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
      } else {
        // have_target_ = false;
        // std::cout << "Goal near collision, stop." << std::endl;
        // changeFSMExecState(WAIT_TARGET, "SAFETY");
        std::cout << "goal near collision, keep retry" << std::endl;
        changeFSMExecState(REPLAN_TRAJ, "FSM");

        std_msgs::Empty emt;
        replan_pub_.publish(emt);
      }
    }
  }

  /* ---------- check trajectory ---------- */
  if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ) {
    double dist;
    bool   safe = planner_manager_->checkTrajCollision(dist);

    if (!safe) {
      std::cout << "current traj in collision." << std::endl;
      ROS_WARN("current traj in collision.");
      changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    }
  }
}

bool KinoReplanFSM::callKinodynamicReplan() {
  bool plan_success =
      planner_manager_->kinodynamicReplan(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_);

  if (plan_success) {
    std::cout<<"plan success"<<std::endl;
    planner_manager_->planYaw(start_yaw_);
    std::cout<<"plan yaw success"<<std::endl;

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    plan_manage::Bspline bspline;
    bspline.order      = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id    = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getInterval();

    bspline_pub_.publish(bspline);

    /* visulization */
    auto plan_data = &planner_manager_->plan_data_;
    //可视化  青绿色 A*初始算法
    visualization_->drawGeometricPath(plan_data->kino_path_, 0.075, Eigen::Vector4d(1, 1, 0, 0.4));

    //可视化  红色 B样条曲线 粉红色 B样条控制点   
    visualization_->drawBspline(info->position_traj_, 0.1, Eigen::Vector4d(1.0, 0, 0.0, 1), true, 0.2,
                                Eigen::Vector4d(1, 0, 1, 1));
    ROS_INFO("visulization success");
    // visualization_->
    
    return true;

  } else {
    std::cout << "generate new traj fail." << std::endl;
    return false;
  }
}

}  // namespace fast_planner
